/**
 * @file scenarioEvaluation.h
 * @brief This file defines functions for evaluating a scenario.
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <string>

namespace proseco_planning {
class Node;

bool isScenarioTerminal(const Node* const rootNode);

bool terminal_condition_reached(const float value, const std::string& comparator,
                                const float condition);
}  // namespace proseco_planning
