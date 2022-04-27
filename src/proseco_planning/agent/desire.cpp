#include <map>

#include "nlohmann/json.hpp"
#include "proseco_planning/agent/desire.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/math/mathlib.h"

namespace proseco_planning {
/**
 * @brief Constructs a new Desire object using default values.
 */
Desire::Desire() = default;
/**
 * @brief Constructs a new Desire object using a desire config.
 *
 * @param desire The desire config.
 */
Desire::Desire(const config::Desire& desire)
    : m_desiredLane(desire.lane),
      m_desiredVelocity(desire.velocity),
      m_toleranceVelocity(desire.velocity_tolerance),
      m_toleranceLaneCenter(desire.lane_center_tolerance) {}

/**
 * @brief Checks if the desired state is fulfilled.
 * @param Vehicle The vehicle.
 * @return True if fulfilled, false otherwise.
 */
bool Desire::desiresFulfilled(const Vehicle& vehicle) const {
  // Correct lane
  // Correct velocity
  // Correct y coordinate
  return (m_desiredLane == vehicle.m_lane) && desireVelocityFulfilled(vehicle) &&
         desireLateralPositionFulfilled(vehicle);
}

/**
 * @brief Checks if the desired lateral position is reached.
 * @param Vehicle The vehicle.
 * @return True if reached, false otherwise.
 */
bool Desire::desireLateralPositionFulfilled(const Vehicle& vehicle) const {
  // Correct lateral position
  return math::isEqual(vehicle.getDistanceToLaneCenter(), 0.0f, m_toleranceLaneCenter);
}

/**
 * @brief Checks if the desired Velocity is reached.
 * @param Vehicle The vehicle.
 * @return True if reached, false otherwise.
 */
bool Desire::desireVelocityFulfilled(const Vehicle& vehicle) const {
  // Correct velocity
  return math::isEqual(m_desiredVelocity, vehicle.m_velocityX, m_toleranceVelocity);
}

/**
 * @brief Function to allow conversion of a Desire to a JSON object.
 * @details Gets called by the json constructor of the nlohmann json library.
 *
 * @param j The JSON object to be filled.
 * @param desire The Desire to be converted.
 */
void to_json(json& j, const Desire& desire) {
  j["velocity"]              = desire.m_desiredVelocity;
  j["lane"]                  = desire.m_desiredLane;
  j["velocity_tolerance"]    = desire.m_toleranceVelocity;
  j["lane_center_tolerance"] = desire.m_toleranceLaneCenter;
}
}  // namespace proseco_planning
