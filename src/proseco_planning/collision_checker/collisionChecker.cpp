#include "proseco_planning/collision_checker/collisionChecker.h"

#include <iostream>
#include <utility>

#include "proseco_planning/collision_checker/collisionCheckerCircleApproximation.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"

namespace proseco_planning {
/**
 * @brief Constructs a new CollisionChecker object.
 * @details Is used for inheritance.
 * @param name The name of the collision checker.
 * @param safetyDistance The safety distance, i.e. the minimum distance between two objects.
 */
CollisionChecker::CollisionChecker(const std::string& name, const float safetyDistance)
    : m_name(name), m_safetyDistance(safetyDistance) {}

/**
 * @brief Creates a new CollisionChecker object.
 *
 * @param type The type of the collision checker.
 * @param safetyDistance The safety distance, i.e. the minimum distance between two objects. The
 * default value is specified by the configuration.
 * @return std::unique_ptr<CollisionChecker> The pointer to the collision checker.
 */
std::unique_ptr<CollisionChecker> CollisionChecker::createCollisionChecker(
    const std::string& type, const float safetyDistance) {
  if (type == "circleApproximation") {
    return std::make_unique<CollisionCheckerCircleApproximation>(type, safetyDistance);
  } else {
    throw std::invalid_argument("Unknown collision checker type: " + type);
  }
}

/**
 * @brief Checks for a collision between a vehicle and a list of obstacles.
 *
 * @param vehicle The vehicle.
 * @param trajectory The trajectory of the vehicle.
 * @param obstacles The list of obstacles.
 * @return true If a collision is detected.
 * @return false Otherwise.
 */
bool CollisionChecker::collision(const Vehicle& vehicle, const Trajectory& trajectory,
                                 const std::vector<config::Obstacle>& obstacles) {
  for (const auto& obstacle : obstacles) {
    if (collision(vehicle, trajectory, obstacle)) {
      return true;
    }
  }
  return false;
}

}  // namespace proseco_planning