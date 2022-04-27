#include "proseco_planning/policies/selection/selectionUCTProgressiveWidening.h"

#include <limits>
#include <map>
#include <memory>
#include <utility>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/node.h"
#include "proseco_planning/search_guide/searchGuide.h"

namespace proseco_planning {

/**
 * @brief Constructs a new Selection UCT Progressive Widening object.
 *
 * @param name The name of the selection policy.
 */
SelectionUCTProgressiveWidening::SelectionUCTProgressiveWidening(const std::string& name)
    : SelectionPolicy(name) {
  // specification of progressive widening
  m_progressiveWideningCoefficient =
      cOpt().policy_options.policy_enhancements.progressive_widening.coefficient;
  m_progressiveWideningExponent =
      cOpt().policy_options.policy_enhancements.progressive_widening.exponent;
  // initialize PW parameter
  if (cOpt().policy_options.policy_enhancements.move_grouping.active &&
      cOpt().policy_options.policy_enhancements.move_grouping.move_grouping_criteria_pw.active) {
    m_progressiveWideningExponent =
        cOpt()
            .policy_options.policy_enhancements.move_grouping.move_grouping_criteria_pw.exponent_pw;
    m_progressiveWideningCoefficient = cOpt()
                                           .policy_options.policy_enhancements.move_grouping
                                           .move_grouping_criteria_pw.coefficient_pw;
  }
}
/**
 * @brief Tests whether the progressive widening criteria is met (i.e. actions are a sublinear
 * function of the visit count).
 *
 * @param actions Current number of actions the agent can choose from.
 * @param coefficient Coefficient of the sublinear function.
 * @param visits Current number of visits of this state.
 * @param exponent Exponent of the sublinear function.
 * @return true If a new action should be added, false otherwise.
 */
bool SelectionUCTProgressiveWidening::progressiveWidening(const unsigned int actions,
                                                          const float coefficient,
                                                          const float visits,
                                                          const float exponent) {
  return actions < coefficient * std::pow(visits, exponent);
}

/**
 * @brief Tests whether the preconditions for progressive widening for move grouping are met.
 *
 * @param node The current node.
 * @param agent The agent progressive widening should be checked for.
 * @param agentIdx The index of the agent in m_actionClassSet.
 * @return true If a new action should be added, false otherwise.
 */
bool SelectionUCTProgressiveWidening::meetsMoveGroupingPWCriteria(const Node* const node,
                                                                  const Agent& agent,
                                                                  const size_t agentIdx) const {
  return !agent.m_isPredefined &&
         node->m_depth <
             cOpt().policy_options.policy_enhancements.progressive_widening.max_depth_pw &&
         progressiveWidening(agent.m_actionClassCount.at(m_actionClassSet.at(agentIdx)),
                             m_progressiveWideningCoefficient,
                             agent.m_actionClassVisits.at(m_actionClassSet.at(agentIdx)),
                             m_progressiveWideningExponent);
}

/**
 * @brief Tests whether the preconditions for progressive widening are met.
 * @todo This needs to checked, why is the moveGrouping version agent based and this version node
 * based?
 * @param node The current node.
 * @return true If a new action should be added, false otherwise.
 */
bool SelectionUCTProgressiveWidening::meetsPWCriteria(const Node* const node) const {
  return node->m_depth <
             cOpt().policy_options.policy_enhancements.progressive_widening.max_depth_pw &&
         progressiveWidening(node->m_childMap.size(), m_progressiveWideningCoefficient,
                             node->m_visits, m_progressiveWideningExponent);
}

/**
 * @brief Selects the node that shall be expanded next.
 *
 * @param node The pointer to the node from where to start the selection process.
 * @param actionSet The action set that is used for the expansion (updated by reference).
 * @param agentsRewards The vector that contains for every step along the tree path another vector
 * that stores the reward for each agent; (updated by reference).
 * @return Node* The pointer to the selected node.
 */
Node* SelectionUCTProgressiveWidening::selectNodeForExpansion(
    Node* node, ActionSet& actionSet, std::vector<std::vector<float> >& agentsRewards) {
  while (!node->m_collision &&  // collision occurred -> invalid path
         !node->m_invalid &&    // invalid state occurred -> invalid path
         getBestNode(node) !=
             nullptr)  // getBestNode returns nullPtr if the best action has not been taken yet
  {
    // if progressive widening is applied, continue with the expansion policy
    if (checkForProgressiveWidening(node)) {
      break;
    }

    // check for best action of already chosen ones,
    // not yet chosen ones lead to "infinite" UCT score
    // if infinite UCT score:
    //  - select one of them randomly
    //  - expand them all

    // take the best action and descent down the tree
    node = node->getChild(m_actionSet);
    // collect the reward from the taken action
    extractReward(node, agentsRewards);
  }
  // assign best action for return by reference
  actionSet = m_actionSet;
  return node;
}

/**
 * @brief Return the "best" child node according to UCT. If this node doesn't exist, return
 * nullptr. It sets the member variable `m_actionSet` accordingly. Move Grouping is integrated in
 * here.
 *
 * @param node Pointer to the current node of the descent.
 * @return Pointer to the existing child node or nullptr if the child node does not exist.
 */
Node* SelectionUCTProgressiveWidening::getBestNode(const Node* const node) {
  if (cOpt().policy_options.policy_enhancements.move_grouping.active)
  // determine the best action within the best action group (based on action
  // classes)
  {
    // sets the best actionClass set
    getBestActionClassUCT(node);
    // calculates the best action within the best action class of each agent
    getBestActionUCT(node, m_actionClassSet);
  } else {
    // determine the best action based of all actions
    getBestActionUCT(node);
  }
  // if all agents determine the UCT value base on their own view
  // the action with the best score does not have to be already executed
  // => first (all/a lot of) permutations are explored

  // already explored action
  if (node->m_childMap.count(m_actionSet)) {
    return node->m_childMap.at(m_actionSet).get();
  }
  // not tried permutation of the available actions
  else {
    return nullptr;
  }
}

/**
 * @brief Sets the best action set based on the UCT value.
 *
 * @param node The node for which the best action set is determined.
 */
void SelectionUCTProgressiveWidening::getBestActionUCT(const Node* const node) {
  // Reset the former action set
  m_actionSet.clear();
  // Determine best action for each agent
  for (const auto& agent : node->m_agents) {
    m_actionSet.push_back(agent.maxActionUCTAction());
  }
}

/**
 * @brief Sets the best action class set based on the UCT value.
 *
 * @param node The node for which the best action class set is determined.
 */
void SelectionUCTProgressiveWidening::getBestActionClassUCT(const Node* const node) {
  // Reset the former action set
  m_actionClassSet.clear();
  // determine best action class for each agent
  for (const auto& agent : node->m_agents) {
    m_actionClassSet.push_back(agent.maxActionUCTActionClass());
  }
}

/**
 * @brief Sets the best action set based on the UCT value of the best action class set.
 *
 * @param node The node for which the best action set is determined.
 * @param actionClassSet The action class set for which the best action set is determined.
 */
void SelectionUCTProgressiveWidening::getBestActionUCT(
    const Node* const node, const std::vector<ActionClass>& actionClassSet) {
  // Reset the former action set
  m_actionSet.clear();

  // determine best action within best action class
  ActionPtr bestAction;

  for (size_t i = 0; i < node->m_agents.size(); ++i) {
    // reset the best action and the best UCT score
    bestAction    = nullptr;
    float bestUCT = std::numeric_limits<float>::lowest();

    for (const auto& [actionPtr, value] : node->m_agents[i].m_actionUCT) {
      if (actionPtr->m_actionClass == m_actionClassSet[i]) {
        if (value > bestUCT) {
          bestUCT    = value;
          bestAction = actionPtr;
        }
      }
    }

    m_actionSet.push_back(bestAction);

    ///@todo check this
    // assert(bestAction == nullptr && "best action is a nullptr");
    // assert(bestAction->m_actionClass != m_actionClassSet[i] &&
    //        "best action is not part of the best action class");
  }
}

// applies the Progressive Widening criteria
// and returns bool if criteria for Progressive Widening fulfilled
// Input:
// bestActionSet
// Output:
// indicator if PW was performed
// modified bestActionSet
bool SelectionUCTProgressiveWidening::checkForProgressiveWidening(Node* const node) {
  // for move grouping the selected group is relevant for detecting the necessity for progressive
  // widening
  if (cOpt().policy_options.policy_enhancements.move_grouping.active &&
      cOpt().policy_options.policy_enhancements.move_grouping.move_grouping_criteria_pw.active) {
    bool progressiveWidening = false;
    // Decision on agent level each agent decides if progressive widening should occur or not
    // yes: sample new action and add to own action space
    // no: take already determined bestAction from bestActionSet
    for (size_t agentIdx{}; agentIdx < node->m_agents.size(); ++agentIdx) {
      // check if progressive widening within group is necessary
      auto& agent = node->m_agents[agentIdx];
      if (meetsMoveGroupingPWCriteria(node, agent, agentIdx)) {
        // get action from best action class and manipulate the actionSet
        m_actionSet[agentIdx] = getGuidedActionForProgressiveWidening(agent, agentIdx);
        progressiveWidening   = true;
      }
    }

    if (progressiveWidening) {
      ///@todo check this
      // assert(std::any_of(m_actionSet.begin(), m_actionSet.end(),
      //                    [](ActionPtr a) { return a == nullptr; }) &&
      //        "best action is a nullptr");
      setActionSetForProgressiveWidening(node);
      return true;
    } else {
      return false;
    }
  }
  // Progressive widening based on node statistics
  else {
    // check current node for progressive widening,
    // if yes select action to take and break
    // progressive widening only applied until depth 2
    // Decision on node level
    if (meetsPWCriteria(node)) {
      getActionSetForProgressiveWidening(node);
      setActionSetForProgressiveWidening(node);
      return true;
    } else {
      return false;
    }
  }
}

/**
 * @brief Gets the action set for progressive widening for each agent.
 *
 * @param node The node to get the action set for.
 */
void SelectionUCTProgressiveWidening::getActionSetForProgressiveWidening(const Node* const node) {
  for (size_t agentIdx{}; agentIdx < node->m_agents.size(); ++agentIdx) {
    auto& agent = node->m_agents[agentIdx];
    if (agent.m_isPredefined) {
      // select random action from available actions WITHOUT adding new action
      // to action space
      m_actionSet[agentIdx] = getRandomAction(agent);
    } else {
      m_actionSet[agentIdx] = getGuidedActionForProgressiveWidening(agent, agentIdx);
    }
  }
}

/**
 * @brief Gets a guided action for progressive widening, i.e. an action from the actions space of
 * the agent using the configured search guide.
 *
 * @param agent The agent to get the guided action for.
 * @param agentIdx The index of the agent.
 * @return ActionPtr The guided action.
 */
ActionPtr SelectionUCTProgressiveWidening::getGuidedActionForProgressiveWidening(
    const Agent& agent, const size_t agentIdx) {
  if (cOpt().policy_options.policy_enhancements.move_grouping.active &&
      cOpt().policy_options.policy_enhancements.move_grouping.move_grouping_bias_pw) {
    // sample within best action class
    auto& actionClass = m_actionSet.at(agentIdx)->m_actionClass;
    return agent.m_searchGuide->getBestActionInActionClassForPW(actionClass, *agent.m_actionSpace,
                                                                agent.m_vehicle, agent.m_actionUCT);
  } else {
    // sample within action space
    return agent.m_searchGuide->getBestActionForPW(*agent.m_actionSpace, agent.m_vehicle,
                                                   agent.m_actionUCT);
  }
}

/**
 * @brief Sets the action set for progressive widening. Adds the new actions to the action space of
 * the agents.
 *
 * @param node The node to which the action set should be applied.
 */
void SelectionUCTProgressiveWidening::setActionSetForProgressiveWidening(Node* const node) {
  for (size_t i = 0; i < node->m_agents.size(); ++i) {
    // if one agent did NOT apply progressive widening this time no additional
    // action can be added to the maps since its already in there
    if (!node->m_agents[i].m_actionVisits.contains(m_actionSet.at(i))) {
      // add the generated action to the available actions of the agents
      node->m_agents[i].addAvailableAction(m_actionSet.at(i));
    }
  }
}
}  // namespace proseco_planning
