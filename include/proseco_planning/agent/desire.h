/**
 * @file desire.h
 * @brief This file defines the Desire class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace proseco_planning {
class Vehicle;
namespace config {
struct Desire;
}  // namespace config

/**
 * @brief Desire class: summarizes the desired state of the agent.
 */
class Desire {
 public:
  Desire();

  explicit Desire(const config::Desire& desire);

  bool desiresFulfilled(const Vehicle& vehicle) const;

  bool desireVelocityFulfilled(const Vehicle& vehicle) const;

  bool desireLateralPositionFulfilled(const Vehicle& vehicle) const;

  /// The Lane the agent is desiring to drive on -- [#]
  int m_desiredLane{0};

  /// The desired longitudinal velocity -- [m/s]
  float m_desiredVelocity{0.0f};

  /// The tolerance for accepting a desired velocity to be fulfilled [m/s]
  float m_toleranceVelocity{0.0f};

  /// The tolerance for accepting a desired lateral lane position to be fulfilled [m]
  float m_toleranceLaneCenter{0.0f};
};

void to_json(json& j, const Desire& desire);
}  // namespace proseco_planning