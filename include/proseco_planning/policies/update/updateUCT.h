/**
 * @file updateUCT.h
 * @brief The file defines the UpdateUCT class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once
#include <vector>

#include "proseco_planning/policies/updatePolicy.h"

namespace proseco_planning {
class Agent;
class Node;

/*!
 * @brief The UpdateUCT class is an update policy that uses the UCT algorithm.
 * @details Within the update policy the following steps are performed:
 * 1. The action specific values for the parent nodes are updated.
 * 2. The action specific values for similar nodes are updated using a similarity function.
 * 3. The action class specific values for the parent nodes are updated.
 */
class UpdateUCT : public UpdatePolicy {
 public:
  using UpdatePolicy::UpdatePolicy;

  void updateNode(Node* const node, const std::vector<std::vector<float> >& agentsRewards,
                  int simulatedDepth) override;

 private:
  static void updateVisitCount(Node* const node, const ActionPtr& action, const size_t agentIdx,
                               const float increment);

  static void updateActionValue(Node* const node, const ActionPtr& action, const size_t agentIdx,
                                const float return_, const float similarity);

  static void updateStandard(Node* const node,
                             const std::vector<std::vector<float> >& agentsRewards,
                             int simulatedDepth);

  static void updateSimilarity(Node* const node, const ActionPtr& executedAction,
                               const size_t agentIdx, const float return_);

  static void updateActionClassUCT(Agent& agent);

  static void updateActionClassValues(const Node* const node);
};
}  // namespace proseco_planning
