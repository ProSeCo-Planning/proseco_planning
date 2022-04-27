/**
 * @file expansionUCT.h
 * @brief This file defines the ExpansionUCT class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>

#include "proseco_planning/policies/expansionPolicy.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Node;

/**
 * @brief The ExpansionUCT class expands the tree using an action set selected by the UCT algorithm.
 *
 */
class ExpansionUCT : public ExpansionPolicy {
 public:
  using ExpansionPolicy::ExpansionPolicy;

  Node* expandTree(Node* node, ActionSet& actionSet,
                   std::vector<std::vector<float> >& agentsRewards,
                   const unsigned int maxDepth) override;
};
}  // namespace proseco_planning