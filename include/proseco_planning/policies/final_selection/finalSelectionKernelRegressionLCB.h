/**
 * @file finalSelectionKernelRegressionLCB.h
 * @brief This file defines the FinalSelectionKernelRegressionLCB class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <string>
#include <vector>

#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/policies/finalSelectionPolicy.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Node;
/**
 * @brief Final selection policy that uses kernel regression lower confidence bound values.
 *
 */
class FinalSelectionKernelRegressionLCB : public FinalSelectionPolicy {
 public:
  using FinalSelectionPolicy::FinalSelectionPolicy;

  explicit FinalSelectionKernelRegressionLCB(const std::string& name);

  ActionSet getBestActionSet(const Node* const nodeFinalSelection) override;

  void setBestActionSet(const Node* const nodeFinalSelection);

  void setBestActionClass(const Node* const nodeFinalSelection);

  float getSimilarity(const ActionClass& centerActionClass,
                      const ActionClass& compareActionClass) const;

  float useManhattanKernel(const ActionClass& centerActionClass,
                           const ActionClass& compareActionClass) const;

  /// The flag that indicates whether moveGrouping is active.
  bool m_moveGrouping{false};

  /// The gamma for action kernel.
  float m_gammaAction;

  /// The gamma for action class kernel.
  float m_gammaActionClass;

  /// The coefficient for the exploration of actions.
  float m_cpAction;

  /// The coefficient for the exploration of action classes.
  float m_cpActionClass;

  /// The best action class set.
  std::vector<ActionClass> m_bestActionClassSet;
};
}  // namespace proseco_planning