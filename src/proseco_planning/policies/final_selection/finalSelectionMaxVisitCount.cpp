#include "proseco_planning/policies/final_selection/finalSelectionMaxVisitCount.h"

#include <cassert>
#include <map>
#include <memory>
#include <utility>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/node.h"

namespace proseco_planning {

/**
 * @brief Returns the best action in the given ActionClass for the specified agent based on the
 * visitCount.
 *
 * @param agent The specified agent.
 * @param bestActionClass The best action classto be searched for the best action.
 * @return ActionPtr The best action based on visit count.
 */
ActionPtr FinalSelectionMaxVisitCount::maxVisitCountActionInActionClass(
    const Agent& agent, const ActionClass bestActionClass) {
  float maxVisitCount{0.0f};
  ActionPtr bestAction{nullptr};
  // Iterate over all actions of the agent
  for (const auto& [actionPtr, actionVisits] : agent.m_actionVisits) {
    if (actionPtr->m_actionClass == bestActionClass) {
      if (actionVisits > maxVisitCount) {
        bestAction    = actionPtr;
        maxVisitCount = actionVisits;
      }
    }
  }
  return bestAction;
}

/**
 * @brief Returns the best ActionSet for the specified node based on visitCount.
 *
 * @param nodeFinalSelection The specified node.
 * @return ActionSet The best ActionSet for the specified node.
 */
ActionSet FinalSelectionMaxVisitCount::getBestActionSet(const Node* const nodeFinalSelection) {
  // Clear the action set
  m_bestActionSet.clear();
  // Set the best action to the nullptr
  ActionPtr bestAction{nullptr};

  for (const auto& agent : nodeFinalSelection->m_agents) {
    if (cOpt().policy_options.policy_enhancements.move_grouping.final_decision) {
      // Find the best action based on the action class with the maximum visit count
      bestAction = maxVisitCountActionInActionClass(agent, agent.maxActionVisitsActionClass());

    } else {
      // Find the best action based on the maximum visit count
      bestAction = agent.maxActionVisitsAction();
    }
    assert(bestAction && "== nullptr");
    // Add the best action for each agent
    m_bestActionSet.push_back(bestAction);
  }
  return m_bestActionSet;
}
}  // namespace proseco_planning
