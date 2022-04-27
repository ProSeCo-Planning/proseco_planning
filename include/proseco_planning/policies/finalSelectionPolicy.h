/**
 * @file finalSelectionPolicy.h
 * @brief This file defines the FinalSelectionPolicy class, the base class for all final selection
 * policies. It is used to select the best node.
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
 * @brief The FinalSelectionPolicy class defines the base class for all final selection policies. It
 * is used to select the best node.
 *
 */
class FinalSelectionPolicy : public Policy {
 public:
  /**
   * @brief Construct a new Final Selection Policy object.
   *
   * @param name The name of the policy.
   */
  explicit FinalSelectionPolicy(const std::string& name) : Policy(name) {}

  /// The virtual destructor.
  virtual ~FinalSelectionPolicy() = default;

  static std::shared_ptr<FinalSelectionPolicy> createPolicy(const std::string& name);

  /**
   * @brief Returns the best action set for the current node of the final selection process.
   *
   * @param nodeFinalSelection Pointer to the current node of the final selection process.
   * @return The best action set.
   */
  virtual ActionSet getBestActionSet(const Node* const nodeFinalSelection) = 0;

  ActionSetSequence getBestPlan(const Node* nodeFinalSelection);

 protected:
  /// The selected best action set.
  ActionSet m_bestActionSet;
};
}  // namespace proseco_planning