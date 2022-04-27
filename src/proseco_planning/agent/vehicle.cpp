#include "proseco_planning/agent/vehicle.h"

#include <sys/types.h>
#include <map>

#include "nlohmann/json.hpp"

namespace proseco_planning {

/**
 * @brief Constructs a new Vehicle object using a vehicle config.
 *
 * @param vehicle The vehicle config.
 */
Vehicle::Vehicle(const config::Vehicle& vehicle)
    : m_positionX(vehicle.position_x),
      m_positionY(vehicle.position_y),
      m_velocityX(vehicle.velocity_x),
      m_velocityY(vehicle.velocity_y),
      m_accelerationX(0),
      m_accelerationY(0),
      m_heading(vehicle.heading),
      m_yawRate(0),
      m_lane(getLane(vehicle.position_y)),
      m_length(vehicle.length),
      m_width(vehicle.width),
      m_wheelBase(vehicle.wheel_base),
      m_maxSteeringAngle(vehicle.max_steering_angle),
      m_maxSpeed(vehicle.max_speed),
      m_maxAcceleration(vehicle.max_acceleration){};
/**
 * @brief Sets the lane of the vehicle, as well as the y-coordinate.
 * @details This method should only be called during the initialization of the scenario as it also
 * updates the y-coordinate of the vehicle to the correct y-coordinate of lane.
 *
 * @param lane The lane the vehicle should be set to.
 */
void Vehicle::setLane(const int lane) {
  m_lane      = lane;
  m_positionY = (lane + 0.5) * sOpt().road.lane_width;
}

/**
 * @brief Calculates the distance from the center of the vehicle's rear axle to the center of the
 * lane of the vehicle.
 *
 * @return float The distance to the center of the lane of the vehicle.
 */
float Vehicle::getDistanceToLaneCenter() const {
  return (m_lane + 0.5) * sOpt().road.lane_width - m_positionY;
}

/**
 * @brief Calculates the distance from the center of the vehicle's rear axle to the next lane left
 * of the vehicle.
 *
 * @return float The distance to the next lane left of the vehicle.
 */
float Vehicle::getDistanceToLeftLane() const {
  return (m_lane + 1) * sOpt().road.lane_width - m_positionY;
}

/**
 * @brief Calculates the distance from the center of the vehicle's rear axle to the next lane right
 * of the vehicle.
 *
 * @return float The distance to the next lane right of the vehicle.
 */
float Vehicle::getDistanceToRightLane() const {
  return -(m_positionY - m_lane * sOpt().road.lane_width);
}

/**
 * @brief Calculates the distance from the center of the vehicle's rear axle to the center of the
 * next lane left of the vehicle.
 *
 * @return float The distance to the center of the next lane left of the vehicle.
 */
float Vehicle::getDistanceToLeftLaneCenter() const {
  return 0.5 * sOpt().road.lane_width + getDistanceToLeftLane();
}

/**
 * @brief Calculates the distance from the center of the vehicle's rear axle to the center of the
 * next lane right of the vehicle.
 *
 * @return float The distance to the center of the next lane right of the vehicle.
 */
float Vehicle::getDistanceToRightLaneCenter() const {
  return -0.5 * sOpt().road.lane_width + getDistanceToRightLane();
}

/**
 * @brief Calculates the lane the vehicle is on givent the y-coordinate.
 *
 * @param position_Y The y-coordinate of the vehicle.
 * @return int The lane of the vehicle.
 */
int Vehicle::getLane(const float position_Y) {
  return static_cast<int>(position_Y / sOpt().road.lane_width);
}

/**
 * @brief Updates the state of the vehicle according to the trajectory.
 *
 * @param finalState
 */
void Vehicle::updateState(const std::vector<float>& finalState) {
  m_positionX     = finalState[0];
  m_positionY     = finalState[1];
  m_velocityX     = finalState[2];
  m_velocityY     = finalState[3];
  m_accelerationX = finalState[4];
  m_accelerationY = finalState[5];
  m_lane          = finalState[6];
  m_heading       = finalState[7];
}

/**
 * @brief Checks whether the current vehicle state is valid.
 *
 * @return true
 * @return false
 */
bool Vehicle::isValid() const { return isValid(m_positionY, m_heading); }

/**
 * @brief Checks whether the vehicle is in a valid state for the given y position and heading, i.e.
 * whether or not the entire vehicle within the boundaries of the road.
 *
 * @param positionY The y position of the vehicle.
 * @param heading The heading of the vehicle.
 * @return true
 * @return false
 */
bool Vehicle::isValid(float positionY, float heading) const {
  /// Check if one of the four corner points leaves the road (only the lateral component is
  /// relevant)
  /// Front right
  if (!isOnRoad(positionY + m_length * std::sin(heading) - m_width / 2.0f * std::cos(heading)))
    return false;
  /// Back right
  if (!isOnRoad(positionY - m_width / 2.0f * std::cos(heading))) return false;
  /// Front left
  if (!isOnRoad(positionY + m_length * std::sin(heading) + m_width / 2.0f * std::cos(heading)))
    return false;
  /// Back left
  if (!isOnRoad(positionY + m_width / 2.0f * std::cos(heading))) return false;

  return true;
}

/**
 * @brief Determines whether the action (trajectory) is valid. i.e. it does not violate physical
 * constraints such as maximum acceleration, velocity or steering angle.
 *
 * @param totalVelocity The magnitude of the velocity of the vehicle.
 * @param totalacceleration The magnitude of the acceleration of the vehicle.
 * @param steeringAngle The steering angle of the vehicle.
 * @return bool True, if the action is valid, false otherwise.
 */
bool Vehicle::isValid(const float totalVelocity, const float totalacceleration,
                      const float steeringAngle) const {
  return std::abs(totalacceleration) < m_maxAcceleration && std::abs(totalVelocity) < m_maxSpeed &&
         std::abs(steeringAngle) < m_maxSteeringAngle;
}

/**
 * @brief Determines whether a given coordinate y is on the road.
 *
 * @param y The y coordinate of a point.
 * @return true
 * @return false
 */
bool Vehicle::isOnRoad(const float y) {
  return y >= 0.0f && y <= sOpt().road.number_lanes * sOpt().road.lane_width;
}

/**
 * @brief Function to allow conversion of a Vehicle to a JSON object.
 * @details Gets called by the json constructor of the nlohmann json library.
 *
 * @param j The JSON object to be filled.
 * @param vehicle The Vehicle to be converted.
 */
void to_json(json& j, const Vehicle& vehicle) {
  j["position_x"]         = vehicle.m_positionX;
  j["position_y"]         = vehicle.m_positionY;
  j["velocity_x"]         = vehicle.m_velocityX;
  j["velocity_y"]         = vehicle.m_velocityY;
  j["acceleration_x"]     = vehicle.m_accelerationX;
  j["acceleration_y"]     = vehicle.m_accelerationY;
  j["lane"]               = vehicle.m_lane;
  j["heading"]            = vehicle.m_heading;
  j["yaw_rate"]           = vehicle.m_yawRate;
  j["width"]              = vehicle.m_width;
  j["length"]             = vehicle.m_length;
  j["wheel_base"]         = vehicle.m_wheelBase;
  j["max_steering_angle"] = vehicle.m_maxSteeringAngle;
  j["max_speed"]          = vehicle.m_maxSpeed;
  j["max_acceleration"]   = vehicle.m_maxAcceleration;
}
}  // namespace proseco_planning
