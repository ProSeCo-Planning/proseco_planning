#include "proseco_planning/policies/final_selection/finalSelectionMostTrusted.h"

#include <cassert>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <utility>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/cost_model/costModel.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/node.h"

namespace proseco_planning {

/**
 * @brief Returns the best action set for a given node.
 *
 * @param nodeFinalSelection The current node.
 * @return ActionSet The best action set for the given node.
 */
ActionSet FinalSelectionMostTrusted::getBestActionSet(const Node* const nodeFinalSelection) {
  // clear the former bestActionSet
  m_bestActionSet.clear();

  // Determine the maximum cooperative reward for each agent
  auto maxCoopRewards = setMaxCoopRewards(nodeFinalSelection);

  for (const auto& agent : nodeFinalSelection->m_agents) {
    // Indicator? new one
    if (cOpt().policy_options.policy_enhancements.move_grouping.final_decision) {
      std::vector<ActionClass> bestActionClassSet;
      // Find the best action class and append it to the set
      ActionClass bestActionClass{findBestActionClass(agent, maxCoopRewards[agent.m_id])};
      bestActionClassSet.push_back(bestActionClass);

      // Find best action within action class
      ActionPtr bestAction{findBestAction(agent, maxCoopRewards[agent.m_id], bestActionClass)};

      assert(bestAction && "== nullptr");
      m_bestActionSet.push_back(bestAction);
    } else {
      // Append the best action irrespective of the action set
      ActionPtr bestAction{findBestAction(agent, maxCoopRewards[agent.m_id])};

      assert(bestAction && "== nullptr");
      m_bestActionSet.push_back(bestAction);
    }
  }
  return m_bestActionSet;
}

/**
 * @brief Returns the maximum coop reward.
 * Determines maximum cooperative reward based on the maximum ego rewards.
 *
 * @param nodeFinalSelection The current node.
 * @return std::vector<float> The maximum cooperative reward.
 */
std::vector<float> FinalSelectionMostTrusted::setMaxCoopRewards(
    const Node* const nodeFinalSelection) {
  std::vector<float> maxEgoReward;
  std::vector<float> maxCoopReward;

  /*
   * Determine the maximum ego reward for each agent. The maximum reward is obtained if the agent
   * is driving straight on its target lane with its target velocity. There is thus no action cost
   * and no penalty for invalid states/collisions. Since the target state is reached, the potential
   * deviation reward reduces to wLaneDeviation + wLaneCenterDeviation + wVelocityDeviation, which
   * is the maximum possible reward. See the CostExponential CostModel implementation for more
   * details.
   */
  for (const auto& agent : nodeFinalSelection->m_agents) {
    maxEgoReward.push_back(agent.m_costModel->m_wLaneDeviation +
                           agent.m_costModel->m_wLaneCenterDeviation +
                           agent.m_costModel->m_wVelocityDeviation);
  }

  // Calculate the undiscounted reward sum of all agents (needed to calculate the cooperative
  // reward)
  float sum{std::accumulate(maxEgoReward.begin(), maxEgoReward.end(), 0.0f)};

  /*
   * Loop over all agents and calculate their maximum possible cooperative reward.
   * This depends on each agent's individual cooperation factor.
   * r_coop = r_ego + coop_factor*sum_other_agents
   *        = r_ego + coop_factor*(sum_all_agents - ego_reward)
   */
  for (const auto& agent : nodeFinalSelection->m_agents) {
    maxCoopReward.push_back(maxEgoReward[agent.m_id] +
                            agent.m_cooperationFactor * (sum - maxEgoReward[agent.m_id]));
  }
  return maxCoopReward;
}

/**
 * @brief Returns the best action for the specified agent based on the maximum performance
 * criterion.
 *
 * @param agent The specified agent.
 * @param maxCoopReward The maximum cooperative reward.
 * @return ActionPtr The best action.
 */
ActionPtr FinalSelectionMostTrusted::findBestAction(const Agent& agent, float maxCoopReward) const {
  // Temporary storage for max value, performance criterion value and best action
  float maxValue{std::numeric_limits<float>::lowest()};
  ActionPtr bestAction{nullptr};

  // Loop over all actions of the action and return the one with the maximum performance criterion
  for (const auto& [actionPtr, value] : agent.m_actionValues) {
    float performanceIndicator =
        calculatePerformanceIndicator(agent.m_actionVisits.at(actionPtr), value, maxCoopReward);
    if (maxValue < performanceIndicator) {
      maxValue   = performanceIndicator;
      bestAction = actionPtr;
    }
  }
  return bestAction;
}

/**
 * @brief Returns the best action in a given ActionClass for the specified agent based on the
 * maximum performance criterion.
 *
 * @param agent The specified agent.
 * @param maxCoopReward The maximum cooperative reward.
 * @param actionClass The specified ActionClass object.
 * @return ActionPtr The best action in the given ActionClass.
 */
ActionPtr FinalSelectionMostTrusted::findBestAction(const Agent& agent, float maxCoopReward,
                                                    const ActionClass& actionClass) const {
  // Temporary storage for max value, performance criterion value and best action
  float maxValue{std::numeric_limits<float>::lowest()};
  ActionPtr bestAction{nullptr};

  // Loop over all actions and determine the action within the given action class that maximizes the
  // performance criterion
  for (const auto& [actionPtr, value] : agent.m_actionValues) {
    if (actionPtr->m_actionClass == actionClass) {
      float performanceIndicator =
          calculatePerformanceIndicator(agent.m_actionVisits.at(actionPtr), value, maxCoopReward);
      if (maxValue < performanceIndicator) {
        maxValue   = performanceIndicator;
        bestAction = actionPtr;
      }
    }
  }
  return bestAction;
}

/**
 * @brief Returns the best action class for the specified agent based on the performance criterion.
 *
 * @param agent The specified agent.
 * @param maxCoopReward The maximum cooperative reward.
 * @return ActionClass The best action class for the specified agent.
 */
ActionClass FinalSelectionMostTrusted::findBestActionClass(const Agent& agent,
                                                           float maxCoopReward) const {
  // Temporary storage for max value, performance criterion value and best action class
  float maxValue{std::numeric_limits<float>::lowest()};
  ActionClass bestActionClass{ActionClass::DO_NOTHING};

  // Loop over all action classes, calculate the criterion and store the best one for return
  for (const auto& [actionClass, value] : agent.m_actionClassValues) {
    float performanceIndicator = calculatePerformanceIndicator(
        agent.m_actionClassVisits.at(actionClass), value, maxCoopReward);
    if (maxValue < performanceIndicator) {
      maxValue        = performanceIndicator;
      bestActionClass = actionClass;
    }
  }
  return bestActionClass;
}

/**
 * @brief Returns the performance indicator based on the visitCount, actionValue and maxCoopReward.
 *
 * @param visitCount The number of times an action has been visited.
 * @param actionValue The action value of a given action.
 * @param maxCoopReward The maximum cooperative reward.
 * @return float The performance indicator.
 */
float FinalSelectionMostTrusted::calculatePerformanceIndicator(float visitCount, float actionValue,
                                                               float maxCoopReward) const {
  return actionValue / maxCoopReward * visitCount / static_cast<float>(cOpt().n_iterations);
}
}  // namespace proseco_planning
