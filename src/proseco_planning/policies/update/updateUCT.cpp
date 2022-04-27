#include "proseco_planning/policies/update/updateUCT.h"

#include <cstddef>
#include <map>
#include <memory>
#include <utility>

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/node.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {

/**
 * @brief Updates the node.
 *
 * @param node The node.
 * @param agentsRewards The agent's rewards.
 * @param simulatedDepth The simulated depth.
 */
void UpdateUCT::updateNode(Node* const node, const std::vector<std::vector<float> >& agentsRewards,
                           int simulatedDepth) {
  // Update the nodes visit count
  ++node->m_visits;

  // Standard update of the visit count and expected cumulative discounted reward
  updateStandard(node, agentsRewards, simulatedDepth);

  // Update of the single actions' UCT scores
  for (auto& agent : node->m_parent->m_agents) {
    updateActionUCT(agent);
  }

  // the move grouping statistics are ALWAYS calculated and exported
  // indicator within config specifies if the data is also used within selection
  // policy refresh action class values of all agents
  updateActionClassValues(node);
}

/**
 * @brief Updates the visit count of the action.
 *
 * @param node The current node.
 * @param action The action that led to the node.
 * @param agentIdx The agent index.
 * @param visits The number of visits.
 */
void UpdateUCT::updateVisitCount(Node* const node, const ActionPtr& action, const size_t agentIdx,
                                 const float visits) {
  node->m_parent->m_agents[agentIdx].m_actionVisits.at(action) += visits;
}

/**
 * @brief Updates the action value of the action.
 *
 * @param node The current node.
 * @param action The action that led to the node.
 * @param agentIdx The agent index.
 * @param return_ The return of the action.
 * @param similarity The similarity of the action.
 */
void UpdateUCT::updateActionValue(Node* const node, const ActionPtr& action, const size_t agentIdx,
                                  const float return_, const float similarity) {
  node->m_parent->m_agents[agentIdx].m_actionValues.at(action) +=
      similarity / (node->m_parent->m_agents[agentIdx].m_actionVisits[action]) *
      (return_ - node->m_parent->m_agents[agentIdx].m_actionValues.at(action));
}

/**
 * @brief Standard update method of the expected reward and visit count of each action (node's
 * child).
 *
 * @param node The current node.
 * @param agentsRewards The agent's rewards.
 * @param simulatedDepth The simulated depth.
 */
void UpdateUCT::updateStandard(Node* const node,
                               const std::vector<std::vector<float> >& agentsRewards,
                               const int simulatedDepth) {
  for (size_t agentIdx = 0; agentIdx < node->m_agents.size(); ++agentIdx) {
    // calculate the cumulative discounted return i.e. the return for the node
    auto return_ = cumulativeDiscountedReward(cOpt().discount_factor, node->m_depth, simulatedDepth,
                                              agentsRewards, agentIdx);
    /// @todo remove potentially, currently only used for the tree visualization, the actual update
    /// happens below
    // use the return to update the action values
    node->m_agents[agentIdx].m_actionValue +=
        1.f / float(node->m_visits) * (return_ - node->m_agents[agentIdx].m_actionValue);

    auto action{node->m_actionSet[agentIdx]};
    updateVisitCount(node, action, agentIdx, 1.f);
    updateActionValue(node, action, agentIdx, return_, 1.f);
    if (cOpt().policy_options.policy_enhancements.similarity_update.active) {
      // update similar actions as well
      updateSimilarity(node, action, agentIdx, return_);
    }
  }
}

void UpdateUCT::updateSimilarity(Node* const node, const ActionPtr& executedAction,
                                 const size_t agentIdx, const float return_) {
  // Update all similar actions within the parent node's agent representation
  for (auto& [action, actionValue] : node->m_parent->m_agents[agentIdx].m_actionValues) {
    if (action != executedAction) {
      auto similarity = Action::getSimilarity(executedAction, action);
      // skip calculation for low similarities
      if (similarity > 0.001f) {
        // update the visit count of the similar action
        updateVisitCount(node, action, agentIdx, similarity);
        updateActionValue(node, action, agentIdx, return_, similarity);
      }
    }
  }
}

/**
 * @brief Updates the action classes of each node.
 *
 * @param node The current node.
 */
void UpdateUCT::updateActionClassValues(const Node* const node) {
  // udpate each agent
  for (auto& agent : node->m_parent->m_agents) {
    // Reset maps to default values
    for (const auto& [actionClass, value] : agent.m_actionClassValues) {
      agent.m_actionClassVisits[actionClass] = 0.0f;
      agent.m_actionClassValues[actionClass] = 0.0f;
      agent.m_actionClassUCT[actionClass]    = config::ComputeOptions::initial_uct;
    }

    // iterate over action maps and collect data
    for (const auto& [action, value] : agent.m_actionVisits) {
      // only evaluate actionClass if the information within the group is high
      // enough avoid division by zero
      if (agent.m_actionClassVisits.at(action->m_actionClass) + value > 0.1) {
        agent.m_actionClassValues.at(action->m_actionClass) =
            (agent.m_actionClassVisits.at(action->m_actionClass) *
                 agent.m_actionClassValues.at(action->m_actionClass) +
             value * agent.m_actionValues.at(action)) /
            (agent.m_actionClassVisits.at(action->m_actionClass) + value);

        agent.m_actionClassVisits.at(action->m_actionClass) += value;
      }
    }

    updateActionClassUCT(agent);
  }
}

/**
 * @brief Updates the action classes of each agent.
 *
 * @param agent The current agent.
 */
// calculate the UCT value of all actions for this agent
void UpdateUCT::updateActionClassUCT(Agent& agent) {
  // find max and min of action values to normalize the exploitation term to [0,1]
  float maxActionValue = agent.maxActionClassActionValue();
  float minActionValue = agent.minActionClassActionValue();
  float totalVisits{agent.cumulativeActionClassVisits()};

  for (const auto& [actionClass, value] : agent.m_actionClassUCT) {
    // If the visit count is less than 0.99 the action has not been visited yet, thus the UCT
    // value of the action should be set to the initial UCT value.
    if (agent.m_actionClassVisits.at(actionClass) < 0.99 || maxActionValue == minActionValue) {
      agent.m_actionClassUCT[actionClass] = config::ComputeOptions::initial_uct;
    } else {
      // Calculate the UCT score
      agent.m_actionClassUCT[actionClass] =
          math::UCT(math::normalize(agent.m_actionClassValues.at(actionClass), maxActionValue,
                                    minActionValue),
                    agent.m_actionClassVisits.at(actionClass), totalVisits,
                    cOpt().policy_options.policy_enhancements.move_grouping.cp);
    }
  }
}

}  // namespace proseco_planning
