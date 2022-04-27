/**
 * @file finalSelectionMaxVisitCount.h
 * @brief This file defines the FinalSelectionMaxVisitCount class.
 * @copyright Copyright (c) 2021
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
 * @brief The FinalSelectionMaxVisitCount class uses the maximum action visit count as final
 * selection criterion.
 */
class FinalSelectionMaxVisitCount : public FinalSelectionPolicy {
 public:
  using FinalSelectionPolicy::FinalSelectionPolicy;

  static ActionPtr maxVisitCountActionInActionClass(const Agent& agent,
                                                    const ActionClass bestActionClass);

  ActionSet getBestActionSet(const Node* const nodeFinalSelection) override;
};
}  // namespace proseco_planning
