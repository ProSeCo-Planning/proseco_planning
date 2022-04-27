/**
 * @file trajectory.h
 * @brief This file defines the Trajectory class.
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <cstddef>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace proseco_planning {
class Vehicle;

/**
 * @brief The Trajectory class defines the trajectory of a vehicle.
 *
 */
class Trajectory {
 public:
  Trajectory(const float t0, const float initialHeading);

  size_t getFractionIndex() const;

  static float getCurrentFraction();

  void determineLane();

  void determineLaneChange();

  void calculateAverageSpeed();

  void calculateAverageAbsoluteAcceleration();

  bool isValidAction(const Vehicle& vehicle) const;

  bool isValidState(const Vehicle& vehicle) const;

  ///@{
  /// The start and end time of the trajectory.
  float m_t0;
  float m_t1;
  ///@}
  ///@{
  /// The start and end time of the trajectory raised to the power of two.
  float m_t0_2;
  float m_t1_2;
  ///@}

  /// The number of steps.
  size_t m_nSteps;

  ///@{
  /// A vector that contains the values of the trajectory at each time step.
  std::vector<float> m_time;
  std::vector<float> m_sPosition;
  std::vector<float> m_dPosition;
  std::vector<float> m_sVelocity;
  std::vector<float> m_dVelocity;
  std::vector<float> m_sAcceleration;
  std::vector<float> m_dAcceleration;
  std::vector<float> m_curvature;
  std::vector<int> m_lane;
  std::vector<float> m_heading;
  std::vector<float> m_steeringAngle;
  std::vector<float> m_totalVelocity;
  std::vector<float> m_totalAcceleration;
  ///@}

  /// The final state for updating the vehicle's state.
  std::vector<float> m_finalState{std::vector<float>(8)};

  /// The average speed in current scenario -- [m/s].
  float m_averageVelocity{0.0f};

  /// The average absolute acceleration in current scenario -- [m/s^2].
  float m_averageAbsoluteAcceleration{0.0f};

  /// The cumulative squared acceleration of the trajectory in longitudinal direction.
  float m_cumSquaredAccelerationLon{0.0f};

  /// The cumulative squared acceleration of the trajectory in lateral direction.
  float m_cumSquaredAccelerationLat{0.0f};

  /// The flag that determines whether the full trajectory is executed (during MCTS search) or
  /// whether only a fraction is executed (action selected in the "environment"= ProSeCoPlanner).
  static bool useActionFraction;

  /// The amount of lane changes.
  int m_laneChange{0};

  /// The flag indicates whether the trajectory results in an invalid action.
  bool m_invalidAction{false};

  /// The flag indicates whether the trajectory results in an invalid state.
  bool m_invalidState{false};
};
void to_json(json& j, const Trajectory& trajectory);
}  // namespace proseco_planning
