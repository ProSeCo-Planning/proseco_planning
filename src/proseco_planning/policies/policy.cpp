#include "proseco_planning/policies/policy.h"

#include <cstddef>
#include <map>
#include <memory>
#include <random>

#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/node.h"

namespace proseco_planning {

Policy::Policy(const std::string& name) : m_name(name) {}

/**
 * @brief Extracts the cooperative reward for each agent from the `node` and stores the values in
 * `agentRewards`.
 * @param node Pointer to the node from which the reward is to be extracted.
 * @param agentsRewards Vector that contains for every step along the tree path another vector
 * that stores the reward for each agent.
 */
void Policy::extractReward(const Node* const node,
                           std::vector<std::vector<float> >& agentsRewards) {
  for (size_t i = 0; i < node->m_agents.size(); ++i) {
    agentsRewards[node->m_depth - 1][i] = node->m_agents[i].m_coopReward;
  }
}

/**
 * @brief Returns a shared action pointer for a random action.
 *
 * @param agent The agent for which the action is to be returned.
 * @return ActionPtr Action pointer for the available action.
 */
// get action from all available actions
ActionPtr Policy::getRandomAction(const Agent& agent) {
  /*
   * Check whether doing nothing has already been visited (always first action in
   * available_actions).
   * If it has not been visited yet, expand it first.
   */
  if (agent.m_actionVisits.at(agent.m_availableActions[0]) <
      config::ComputeOptions::error_tolerance) {
    return agent.m_availableActions[0];
  }
  return math::getRandomElementFromVector(agent.m_availableActions);
}

/**
 * @brief Checks whether a node is terminal, e.g. representing a leaf node that cannot have any
 * children.
 * @param node The node to be checked.
 * @param maxDepth The maximum allowed depth of the tree.
 * @return true If the node is terminal.
 * @return false Otherwise.
 */
bool Policy::isNodeTerminal(const Node* const node, unsigned int maxDepth) {
  return node->m_invalid || node->m_collision || node->m_depth >= maxDepth;
}

/**
 * @brief Sets a random action set based on the available actions of the node's agents.
 *
 * @param node Pointer to the action set.
 */
void Policy::getRandomActionSet(Node* const node) {
  node->m_actionSet.clear();
  for (const auto& agent : node->m_agents) {
    // random selection
    node->m_actionSet.push_back(getRandomAction(agent));
  }
}

}  // namespace proseco_planning
