/**
 * @file policy.h
 * @brief This file defines the Policy class, the base class for all policies.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <string>
#include <vector>

#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Agent;
class Node;

/**
 * @brief The Policy class defines the base class for all policies.
 *
 */
class Policy {
 public:
  explicit Policy(const std::string& name);

  static ActionPtr getRandomAction(const Agent& agent);

  static void getRandomActionSet(Node* const node);

  static void extractReward(const Node* const node,
                            std::vector<std::vector<float> >& agentsRewards);

  static bool isNodeTerminal(const Node* const node, unsigned int maxDepth);

  /// The name of the policy.
  std::string m_name;
};
}  // namespace proseco_planning