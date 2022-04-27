/**
 * @file actionClass.h
 * @brief This file defines the ActionClass class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

namespace proseco_planning {
/**
 * @brief The action class is an enum to describe the different action classes within the
 * ActionSpace.
 */
enum class ActionClass : int {
  /// The action class is unknown.
  NONE = -1,
  /// The action class if no notable change is expected.
  DO_NOTHING = 0,
  /// The action class if the agent is accelerating.
  ACCELERATE = 1,
  /// The action class if the agent is decelerating.
  DECELERATE = 2,
  /// The action class if the agent is accelerating while steering to the left.
  CHANGE_LEFT_FAST = 3,
  /// The action class if the agent is decelerating while steering to the left.
  CHANGE_LEFT_SLOW = 4,
  /// The action class if the agent is accelerating while steering to the right.
  CHANGE_RIGHT_FAST = 5,
  /// The action class if the agent is decelerating while steering to the right.
  CHANGE_RIGHT_SLOW = 6,
  /// The action class if the agent is steering to the left.
  CHANGE_LEFT = 11,
  /// The action class if the agent is steering to the right.
  CHANGE_RIGHT = 12,
};
}  // namespace proseco_planning
