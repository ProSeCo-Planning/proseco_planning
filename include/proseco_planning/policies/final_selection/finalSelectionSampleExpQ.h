/**
 * @file finalSelectionSampleExpQ.h
 * @brief This file defines the FinalSelectionSampleExpQ class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>

#include "proseco_planning/policies/finalSelectionPolicy.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Node;

/**
 * @brief The FinalSelectionSampleExpQ class uses exponential action value weighting to create a
 * stochastic final selection policy.
 *
 */
class FinalSelectionSampleExpQ : public FinalSelectionPolicy {
 public:
  using FinalSelectionPolicy::FinalSelectionPolicy;

  ActionSet getBestActionSet(const Node* const node) override;

  static float calculateActionWeight(const float actionValue);

  static std::tuple<unsigned int, float> sampleActionFromWeights(std::vector<float> weights);
};
}  // namespace proseco_planning
