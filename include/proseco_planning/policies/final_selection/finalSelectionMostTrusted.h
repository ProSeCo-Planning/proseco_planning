/**
 * @file finalSelectionMostTrusted.h
 * @brief This file defines the FinalSelectionMostTrusted class.
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
 * @brief The final selection policy that uses a combination visit counts and action values to
 * select the best action.
 *
 */
class FinalSelectionMostTrusted : public FinalSelectionPolicy {
 public:
  using FinalSelectionPolicy::FinalSelectionPolicy;

  ActionSet getBestActionSet(const Node* const nodeFinalSelection) override;

 private:
  // Find the best action for an agent given the performance criterion
  ActionPtr findBestAction(const Agent& agent, float maxCoopReward) const;
  // Find the best action for an agent (according to calculatePerformanceIndicator) within a given
  // action class
  ActionPtr findBestAction(const Agent& agent, float maxCoopReward,
                           const ActionClass& actionClass) const;
  // Find the best action set for an agent
  ActionClass findBestActionClass(const Agent& agent, float maxCoopReward) const;
  // Determine the maximum attainable cooperative reward for each agent
  static std::vector<float> setMaxCoopRewards(const Node* const nodeFinalSelection);
  // evaluates criteria preferring both: highVisitCounts and high actionValues
  float calculatePerformanceIndicator(float visitCount, float actionValue,
                                      float maxCoopReward) const;
};
}  // namespace proseco_planning
