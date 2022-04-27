/**
 * @file updatePolicy.h
 * @brief This file defines the UpdatePolicy class, the base class for all update policies.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <sys/types.h>
#include <memory>
#include <string>
#include <vector>

#include "policy.h"

namespace proseco_planning {
class Agent;
class Node;

/**
 * @brief The UpdatePolicy class defines the base class for all update policies.
 */
class UpdatePolicy : public Policy {
 public:
  /**
   * @brief Construct a new Update Policy object.
   *
   * @param name The name of the policy.
   */
  explicit UpdatePolicy(const std::string& name) : Policy(name) {}

  /// The virtual destructor.
  virtual ~UpdatePolicy() = default;

  static std::unique_ptr<UpdatePolicy> createPolicy(const std::string& name);

  void updateTree(Node* node, const std::vector<std::vector<float> >& agentsRewards,
                  unsigned int simulatedDepth);

  static float discountedReward(const float discount_factor, const int distance,
                                const float reward);

  static float cumulativeDiscountedReward(const float discount_factor, const int node_depth,
                                          const int simulation_depth,
                                          const std::vector<std::vector<float> >& rewards,
                                          const int agent_idx);

  /**
   * @brief Updates the current node of the search tree ascent.
   *
   * @param node Pointer to the current node of the search tree ascent.
   * @param agentsRewards Vector that contains for every step along the tree path another vector
   * that stores the reward for each agent.
   * @param simulatedDepth Depth of the simulation.
   */
  virtual void updateNode(Node* node, const std::vector<std::vector<float> >& agentsRewards,
                          int simulatedDepth) = 0;

 protected:
  static void updateActionUCT(Agent& agent);
};
}  // namespace proseco_planning