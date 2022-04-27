/**
 * @file finalSelectionMaxActionValue.h
 * @brief This file defines the FinalSelectionMaxActionValue class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>

#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/policies/finalSelectionPolicy.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Agent;
class Node;

/**
 * @brief The FinalSelectionMaxActionValue class uses the maximum action value as final selection
 * criterion.
 */
class FinalSelectionMaxActionValue : public FinalSelectionPolicy {
 public:
  using FinalSelectionPolicy::FinalSelectionPolicy;

  static ActionPtr maxActionValueActionInActionClass(const Agent& agent,
                                                     const ActionClass bestActionClass);

  ActionSet getBestActionSet(const Node* const nodeFinalSelection) override;
};
}  // namespace proseco_planning