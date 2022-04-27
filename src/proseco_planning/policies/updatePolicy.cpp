#include "proseco_planning/policies/updatePolicy.h"

#include <cassert>
#include <iostream>
#include <map>
#include <utility>

#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/node.h"
#include "proseco_planning/policies/update/updateUCT.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {

/**
 * @brief Returns a pointer to a UCT update policy.
 *
 * @param name String that identifies the update policy.
 * @return std::unique_ptr<UpdatePolicy> Pointer to the update policy.
 */
std::unique_ptr<UpdatePolicy> UpdatePolicy::createPolicy(const std::string& name) {
  if (name == "UCT") {
    return std::make_unique<UpdateUCT>(name);
  } else {
    throw std::invalid_argument("Unknown update selection policy type: " + name);
  }
}

/**
 * @brief Updates the search tree according to the simulation results.
 *
 * @param node Pointer to the node that the simulation was started from.
 * @param agentsRewards Vector that contains for every step along the tree path another vector
 * that stores the reward for each agent.
 * @param simulatedDepth Depth of the simulation.
 */
void UpdatePolicy::updateTree(Node* node, const std::vector<std::vector<float> >& agentsRewards,
                              unsigned int simulatedDepth) {
  while (node->m_parent != nullptr) {
    updateNode(node, agentsRewards, simulatedDepth);
    node = node->m_parent;
  }
  // because the root node is not included in the update (attention: the action
  // maps is updated correctly), the root node visits is incremented here. When
  // the while-loop ends, the node points to the root
  ++node->m_visits;
}

/**
 * @brief Calculates the discounted reward.
 *
 * @param discount_factor The discount factor.
 * @param distance The distance between the current state and the future state the reward was
 * received at.
 * @param reward The received reward.
 * @return float The discounted reward.
 */
float UpdatePolicy::discountedReward(const float discount_factor, const int distance,
                                     const float reward) {
  assert(discount_factor <= 1.f && discount_factor > 0 &&
         "discount factor not within bounds, (0,1]");
  assert(distance >= 0 && "negative distance");
  return std::pow(discount_factor, distance) * reward;
}

/**
 * @brief Calculates the return for a reward sequence for a single agent.
 *
 * @param discount_factor The discount factor.
 * @param node_depth The depth of the rewards vector from where to start the calculation.
 * @param simulation_depth The depth of the rewards vector where to end the calculation.
 * @param rewards The rewards vector for all agents.
 * @param agent_idx The index of the current agent.
 * @return float The return.
 */
float UpdatePolicy::cumulativeDiscountedReward(const float discount_factor, const int node_depth,
                                               const int simulation_depth,
                                               const std::vector<std::vector<float> >& rewards,
                                               const int agent_idx) {
  assert(discount_factor <= 1.f && discount_factor > 0 &&
         "discount factor not within bounds, (0,1]");
  assert(node_depth > 0 && "negative node depth");
  assert(simulation_depth >= node_depth && "simulation depth is smaller than node depth");

  float cumulative_discounted_reward{0.0f};
  // correct for the rewards starting at t_1 with rewards[0] belonging to node_depth = 1
  for (int reward_depth = simulation_depth - 1; reward_depth >= node_depth - 1; reward_depth--) {
    // calculate the distance between the current state and the state where the reward was received
    int distance = reward_depth - node_depth + 1;
    cumulative_discounted_reward +=
        discountedReward(discount_factor, distance, rewards[reward_depth][agent_idx]);
  }

  return cumulative_discounted_reward;
}
/**
 * @brief Calculates the UCT value of all discrete actions for this agent.
 *
 * @param agent The agent.
 */
// calculate the UCT value of all actions for this agent
void UpdatePolicy::updateActionUCT(Agent& agent) {
  // find max and min of action values to normalize the exploitation term to [0,1]
  float maxActionValue{agent.maxActionValue()};
  float minActionValue{agent.minActionValue()};
  float totalVisits{agent.cumulativeActionVisits()};

  for (auto visitsIter = agent.m_actionVisits.begin(), valuesIter = agent.m_actionValues.begin();
       visitsIter != agent.m_actionVisits.end(); ++visitsIter, ++valuesIter) {
    // If the visit count is less than 0.99 the action has not been visited yet, thus the UCT value
    // of the action should be set to the initial UCT value.
    if (visitsIter->second < 0.99 || maxActionValue == minActionValue) {
      agent.m_actionUCT.at(visitsIter->first) = config::ComputeOptions::initial_uct;
    } else {
      // the UCT score calculation is based on the total number of visits of the node
      // this is equal to the sum of all visits of the agent's actions
      agent.m_actionUCT.at(visitsIter->first) =
          math::UCT(math::normalize(valuesIter->second, maxActionValue, minActionValue),
                    visitsIter->second, totalVisits, cOpt().uct_cp);
    }
  }
}
}  // namespace proseco_planning
