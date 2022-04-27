/**
 * @file collisionChecker.h
 * @brief This file defines the CollisionChecker class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <string>

#include "proseco_planning/config/configuration.h"

namespace proseco_planning {
class Trajectory;
class Vehicle;

/**
 * @brief CollisionChecker base class: includes the checks for collision between all possible
 * objects.
 */
class CollisionChecker {
 public:
  explicit CollisionChecker(const std::string& type, const float safetyDistance);

  /// The virtual destructor.
  virtual ~CollisionChecker() = default;

  static std::unique_ptr<CollisionChecker> createCollisionChecker(
      const std::string& type, const float safetyDistance = cOpt().safety_distance);

  bool collision(const Vehicle& vehicle, const Trajectory& trajectory,
                 const std::vector<config::Obstacle>& obstacles);

  virtual bool collision(const Vehicle& vehicle0, const Trajectory& trajectory0,
                         const Vehicle& vehicle1, const Trajectory& trajectory1) = 0;

  virtual bool collision(const Vehicle& vehicle, const Trajectory& trajectory,
                         const Vehicle& obstacle) = 0;

  virtual bool collision(const Vehicle& vehicle, const Trajectory& trajectory,
                         const config::Obstacle& obstacle) = 0;

  virtual bool collision(const Vehicle& vehicle0, const Vehicle& vehicle1) = 0;

  virtual bool collision(const Vehicle& vehicle, const config::Obstacle& obstacle) = 0;

  /// The name of the collision checker.
  std::string m_name;

  /// The safety margin for collision checks -- [m]
  const float m_safetyDistance;
};
}  // namespace proseco_planning
