/**
 * @file monteCarloTreeSearch.h
 * @brief This file defines the key functions for the Monte Carlo Tree Search.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <vector>

#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/node.h"
#include "proseco_planning/policies/expansionPolicy.h"
#include "proseco_planning/policies/finalSelectionPolicy.h"
#include "proseco_planning/policies/selectionPolicy.h"
#include "proseco_planning/policies/simulationPolicy.h"
#include "proseco_planning/policies/updatePolicy.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Node;

std::unique_ptr<Node> computeTree(std::unique_ptr<Node> root);

ActionSetSequence computeActionSetSequence(std::unique_ptr<Node> rootNode, int step);

void mergeTrees(Node* const master, const Node* const node);

void similarityUpdate(Node* const master, const Node* const node);

ActionSetSequence similarityVoting(const std::vector<std::unique_ptr<Node>>& resultRoots);

ActionSetSequence similarityMerge(const std::vector<std::unique_ptr<Node>>& resultRoots);

}  // namespace proseco_planning