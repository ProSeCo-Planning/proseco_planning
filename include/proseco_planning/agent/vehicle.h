/**
 * @file vehicle.h
 * @brief This file defines the Vehicle class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"

namespace proseco_planning {
/**
 * @brief The Vehicle class represents a state of a vehicle.
 */
class Vehicle {
 public:
  explicit Vehicle(const config::Vehicle& vehicle);

  bool isValid() const;

  bool isValid(float positionY, float heading) const;

  bool isValid(const float totalVelocity, const float totalacceleration,
               const float steeringAngle) const;

  void updateState(const std::vector<float>& finalState);

  void setLane(const int lane);

  float getDistanceToLaneCenter() const;

  float getDistanceToLeftLane() const;

  float getDistanceToRightLane() const;

  float getDistanceToRightLaneCenter() const;

  float getDistanceToLeftLaneCenter() const;

  static int getLane(const float position_Y);

  static bool isOnRoad(const float y);

  /// The longitudinal position x -- [m] with respect to start of the road.
  float m_positionX;

  /// The lateral position y -- [m] with respect to the right side of the road.
  float m_positionY;

  /// The longitudinal velocity -- [m/s].
  float m_velocityX;

  /// The lateral velocity -- [m/s].
  float m_velocityY;

  /// The longitudinal acceleration -- [m/s²].
  float m_accelerationX;

  /// The lateral acceleration -- [m/s²].
  float m_accelerationY;

  /// The heading -- [°].
  float m_heading;

  /// The yaw rate -- [°/s].
  float m_yawRate;

  /// The lane the vehicle is currently on -- [#] starting from zero for the right most lane.
  int m_lane;

  /// The length -- [m] with respect to the center of the vehicle.
  float m_length;

  /// The width -- [m] with respect to the center of the vehicle.
  float m_width;

  /// The wheel base of the vehicle -- [m].
  float m_wheelBase;

  /// The Maximum steering angle -- [rad].
  float m_maxSteeringAngle;

  /// The Maximum speed -- [m/s] can be given as vehicle maximum speed or speed limit.
  float m_maxSpeed;

  /// The (physical) maximum acceleration -- [m/s^2].
  float m_maxAcceleration;
};

void to_json(json& j, const Vehicle& vehicle);
}  // namespace proseco_planning