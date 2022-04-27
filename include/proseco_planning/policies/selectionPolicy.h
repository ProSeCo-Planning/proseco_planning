/**
 * @file selectionPolicy.h
 * @brief The SelectionPolicy class is the base class for all selection policies. It is used to
 * select the best action from the action set.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "policy.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Node;

/**
 * @brief The SelectionPolicy class is the base class for all selection policies. It is used to
 * select the best action from the action set.
 */
class SelectionPolicy : public Policy {
 public:
  /**
   * @brief Construct a new Selection Policy object.
   *
   * @param name The name of the policy.
   */
  explicit SelectionPolicy(const std::string& name) : Policy(name) {}

  /// The virtual destructor
  virtual ~SelectionPolicy() = default;

  static std::unique_ptr<SelectionPolicy> createPolicy(const std::string& name);

  /**
   * @brief Selects the node that shall be expanded next.
   *
   * @param node The pointer to the node from where to start the selection process.
   * @param actionSet The action set that is used for the expansion (updated by reference).
   * @param agentsRewards The vector that contains for every step along the tree path another vector
   * that stores the reward for each agent; (updated by reference).
   * @return Node* The pointer to the selected node.
   */
  virtual Node* selectNodeForExpansion(Node* node, ActionSet& actionSet,
                                       std::vector<std::vector<float> >& agentsRewards) = 0;

 protected:
  /**
   * @brief determine a "best" action set, update the member `m_actionSet` accordingly and descend
   * to the corresponding "best" child node if this node already exists in the search tree.
   *
   * @param node pointer to the current node of the descent
   * @return pointer to the existing child node or nullptr if the child node does not exist
   */
  virtual Node* getBestNode(const Node* const node) = 0;

  /// calculated action set
  ActionSet m_actionSet;
};
}  // namespace proseco_planning