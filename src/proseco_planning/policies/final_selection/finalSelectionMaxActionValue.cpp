#include "proseco_planning/policies/final_selection/finalSelectionMaxActionValue.h"
#include <cassert>
#include <limits>
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
 * @brief Returns the maximum action value in the given ActionClass for the specified agent based on
 * action value.
 *
 * @param agent The specified agent.
 * @param bestActionClass The best action class to be searched for the best action.
 * @return ActionPtr The best action based on action value.
 */
ActionPtr FinalSelectionMaxActionValue::maxActionValueActionInActionClass(
    const Agent& agent, const ActionClass bestActionClass) {
  float maxActionValue{std::numeric_limits<float>::lowest()};
  ActionPtr bestAction{nullptr};
  // Iterate over all actions of the agent
  for (const auto& [actionPtr, actionValue] : agent.m_actionValues) {
    if (actionPtr->m_actionClass == bestActionClass) {
      if (actionValue > maxActionValue) {
        bestAction     = actionPtr;
        maxActionValue = actionValue;
      }
    }
  }
  return bestAction;
}

/**
 * @brief Returns the best action set for the current node of the final selection process based on
 * action value.
 *
 * @param nodeFinalSelection The current node of the final selection process.
 * @return ActionSet The best Action set based on the action value.
 */
ActionSet FinalSelectionMaxActionValue::getBestActionSet(const Node* const nodeFinalSelection) {
  // Clear the action set
  m_bestActionSet.clear();
  // Set the best action to the nullptr
  ActionPtr bestAction{nullptr};

  for (const auto& agent : nodeFinalSelection->m_agents) {
    if (cOpt().policy_options.policy_enhancements.move_grouping.final_decision) {
      // Find the best action based on the action class with the maximum action value
      bestAction = maxActionValueActionInActionClass(agent, agent.maxActionValueActionClass());

    } else {
      // Find the best action based on the maximum action value
      bestAction = agent.maxActionValueAction();
      // bestAction = maxActionValueAction(agent);
    }
    assert(bestAction && "== nullptr");
    // Add the best action for each agent
    m_bestActionSet.push_back(bestAction);
  }
  return m_bestActionSet;
}
}  // namespace proseco_planning
