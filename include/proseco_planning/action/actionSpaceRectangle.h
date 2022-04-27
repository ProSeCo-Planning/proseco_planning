/**
 * @file actionSpaceRectangle.cpp
 * @brief This file defines the ActionSpace class of a rectangle.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/action/actionSpace.h"
#include "proseco_planning/collision_checker/collisionChecker.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Vehicle;

/**
 * @brief Boundary of an ActionSpaceRectangle
 */
struct ActionBoundary {
  /// The default constructor.
  ActionBoundary(){};
  /**
   * @brief Construct a new Action Boundary object.
   *
   * @param minMaxVelocityChange The minimum and maximum velocity change.
   * @param minMaxLateralChange The minimum and maximum lateral change.
   */
  ActionBoundary(const math::MinMaxPair<float>& minMaxVelocityChange,
                 const math::MinMaxPair<float>& minMaxLateralChange)
      : velocityChange(minMaxVelocityChange), lateralChange(minMaxLateralChange){};
  /// The minimum and maximum velocity change
  math::MinMaxPair<float> velocityChange;
  /// The minimum and maximum lateral change
  math::MinMaxPair<float> lateralChange;
};

/**
 * @brief This action space is shaped like a rectangle, which is specified by
 * the maximum velocity change and the maximum lateral change.
 */
class ActionSpaceRectangle : public ActionSpace {
 public:
  ActionSpaceRectangle(const config::ActionSpaceRectangle& config);

  ActionSet getPredefinedActions() const override;

  ActionSet getModerateActions(const Vehicle& vehicle) const override;

  ActionSet getDetailedActions(const Vehicle& vehicle) const override;

  ActionBoundary getActionSpaceBoundary() const;

  ActionClass getActionClass(const Action& action, const Vehicle& vehicle) const override;

  ActionBoundary getActionClassBoundary(const ActionClass actionClass,
                                        const Vehicle& vehicle) const;

  ActionPtr sampleRandomAction(const Vehicle& vehicle) const override;

  ActionPtr sampleRandomActionInActionClass(const ActionClass& actionClass,
                                            const Vehicle& vehicle) const override;

  static ActionPtr sampleRandomActionInBoundary(const ActionBoundary& boundary);

  /// The ActionSpaceRectangle configuration.
  const config::ActionSpaceRectangle m_config;

  /// The boundary of the action space.
  ActionBoundary m_boundary;

  /// The collision checker.
  std::unique_ptr<CollisionChecker> m_collisionChecker;

  /// The trajectory generator.
  std::unique_ptr<TrajectoryGenerator> m_trajectoryGenerator;

  /**
   * @brief Determines whether an action results in a lane change.
   * @details A lane change can only occur if the lateral change is non-zero and larger than the
   * absolute distance to either of the lanes.
   *
   * @param action The action to check.
   * @param distanceToLeftLane The distance to the left lane.
   * @param distanceToRightLane The distance to the right lane.
   * @return true If the action results in a lane change.
   * @return false If the action does not result in a lane change.
   */
  inline bool laneChange(const Action& action, const float distanceToLeftLane,
                         const float distanceToRightLane) const {
    return action.m_lateralChange != 0.0f && (action.m_lateralChange >= distanceToLeftLane ||
                                              action.m_lateralChange <= distanceToRightLane);
  }

  /**
   * @brief Determines whether an action belongs to the do nothing action class based on its change
   * of state.
   *
   * @param distanceToLeftLane The distance to the left lane.
   * @param distanceToRightLane The distance to the right lane.
   * @return True, if the action cause no change at all, false otherwise.
   */
  inline bool doNothingActionClass(const Action& action, const float distanceToLeftLane,
                                   const float distanceToRightLane) const {
    return !laneChange(action, distanceToLeftLane, distanceToRightLane) &&
           std::abs(action.m_velocityChange) < m_config.delta_velocity;
  }

  /**
   * @brief Determines whether an action belongs to the accelerate action class based on its change
   * of state.
   *
   * @param distanceToLeftLane The distance to the left lane.
   * @param distanceToRightLane The distance to the right lane.
   * @return True, if the action cause an acceleration without lateral movement, false otherwise.
   */
  inline bool accelerateActionClass(const Action& action, const float distanceToLeftLane,
                                    const float distanceToRightLane) const {
    return !laneChange(action, distanceToLeftLane, distanceToRightLane) &&
           action.m_velocityChange > m_config.delta_velocity;
  }

  /**
   * @brief Determines whether an action belongs to the decelerate action class based on its change
   * of state.
   *
   * @param distanceToLeftLane The distance to the left lane.
   * @param distanceToRightLane The distance to the right lane.
   * @return True, if the action cause an deceleration without lateral movement, false otherwise.
   */
  inline bool decelerateActionClass(const Action& action, const float distanceToLeftLane,
                                    const float distanceToRightLane) const {
    return !laneChange(action, distanceToLeftLane, distanceToRightLane) &&
           action.m_velocityChange < m_config.delta_velocity;
  }

  /**
   * @brief Determines whether an action belongs to the change left action class based on its change
   * of state.
   *
   * @param distanceToLeftLane The distance to the left lane.
   * @return True, if the action cause a left movement, false otherwise.
   */
  inline static bool changeLeftActionClass(const Action& action, const float distanceToLeftLane) {
    return action.m_lateralChange != 0.0f && action.m_lateralChange >= distanceToLeftLane;
  }

  /**
   * @brief Determines whether an action belongs to the change right action class based on its
   * change of state.
   *
   * @param distanceToLeftLane The distance to the right lane.
   * @return True, if the action cause a right movement, false otherwise.
   */
  inline static bool changeRightActionClass(const Action& action, const float distanceToRightLane) {
    return action.m_lateralChange != 0.0f && action.m_lateralChange <= distanceToRightLane;
  }

  /**
   * @brief Determines whether an action belongs to a fast action class based on its change of
   * state.
   *
   * @return True, if action cause an acceleration, false otherwise.
   */
  inline bool fastActionClass(const Action& action) const {
    return action.m_velocityChange >= m_config.delta_velocity;
  }

  /**
   * @brief Determines whether an action belongs to a slow action class based on its change of
   * state.
   *
   * @return True, if action cause an deceleration, false otherwise.
   */
  inline bool slowActionClass(const Action& action) const {
    return action.m_velocityChange <= -m_config.delta_velocity;
  }
};

}  // namespace proseco_planning