/**
 * @file alias.h
 * @brief A collection of aliases for common types.
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include <memory>
#include <vector>

namespace proseco_planning {

class Action;
/// An ActionPtr is a shared pointer to an action.
using ActionPtr = std::shared_ptr<Action>;

/// An ActionSet is vector of ActionPtr.
using ActionSet = std::vector<ActionPtr>;

/// An ActionSetSequence is a vector of ActionSet.
using ActionSetSequence = std::vector<ActionSet>;

}  // namespace proseco_planning
