#include "proseco_planning/trajectory/trajectory.h"

#include <cmath>
#include <numeric>

#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/math/mathlib.h"

namespace proseco_planning {

/** The flag indicating whether the complete trajectory is executed (false), or only a fraction of
 * it (true).*/
bool Trajectory::useActionFraction{false};

/**
 * @brief Constructs a new Trajectory object.
 *
 * @param t0 The start time of the trajectory.
 * @param initialHeading The initial heading of the vehicle.
 */
Trajectory::Trajectory(const float t0, const float initialHeading) {
  // initialize characteristic values
  m_t0     = t0;
  m_t1     = t0 + cOpt().action_duration;
  m_t0_2   = m_t0 * m_t0;
  m_t1_2   = m_t1 * m_t1;
  m_nSteps = (m_t1 - m_t0) / cOpt().delta_t + 1;

  // initialize vectors
  m_time.reserve(m_nSteps);

  m_sPosition.resize(m_nSteps, 0);
  m_dPosition.resize(m_nSteps, 0);
  m_sVelocity.resize(m_nSteps, 0);
  m_dVelocity.resize(m_nSteps, 0);
  m_sAcceleration.resize(m_nSteps, 0);
  m_dAcceleration.resize(m_nSteps, 0);
  m_lane.resize(m_nSteps, 0);
  m_heading.resize(m_nSteps, 0);
  m_steeringAngle.resize(m_nSteps, 0);
  m_curvature.resize(m_nSteps, 0);
  m_totalVelocity.resize(m_nSteps, 0);
  m_totalAcceleration.resize(m_nSteps, 0);
}

/**
 * @brief Determines the lane at each step of the trajectory.
 * @note Assigns a lane if the vehicle is on the road (i.e. whether the center of the rear axle is
 * on the road), otherwise assigns a lane of -1.
 *
 */
void Trajectory::determineLane() {
  const auto roadWidth = sOpt().road.number_lanes * sOpt().road.lane_width;
  for (size_t i = 0; i <= getFractionIndex(); ++i) {
    if (m_dPosition[i] < 0 || m_dPosition[i] > roadWidth) {
      m_lane[i] = -1;
    } else {
      m_lane[i] = Vehicle::getLane(m_dPosition[i]);
    }
  }
}

/**
 * @brief Checks whether the trajectory obeys all physical constraints.
 *
 * @param vehicle The vehicle that tracks the trajectory.
 * @return true If the trajectory obeys all physical constraints.
 * @return false Otherwise.
 */
bool Trajectory::isValidAction(const Vehicle& vehicle) const {
  for (size_t i = 0; i <= getFractionIndex(); ++i) {
    if (!vehicle.isValid(m_totalVelocity[i], m_totalAcceleration[i], m_steeringAngle[i]))
      return false;
  }
  return true;
}

/**
 * @brief Checks whether all states of the trajectory are valid.
 *
 * @param vehicle The vehicle that tracks the trajectory.
 * @return true If all states are valid.
 * @return false Otherwise.
 */
bool Trajectory::isValidState(const Vehicle& vehicle) const {
  for (size_t i = 0; i <= getFractionIndex(); ++i) {
    if (!vehicle.isValid(m_dPosition[i], m_heading[i])) return false;
  }
  return true;
}

/**
 * @brief Determines the amount of lane change between the start and the end of the trajectory.
 */
void Trajectory::determineLaneChange() { m_laneChange = m_lane.back() - m_lane.front(); }

/**
 * @brief Calculates the average speed of the trajectory.
 */
void Trajectory::calculateAverageSpeed() {
  const auto fractionVector = math::getSubvectorFromVector(m_totalVelocity, 0, getFractionIndex());
  m_averageVelocity =
      std::accumulate(fractionVector.begin(), fractionVector.end(), 0.0f) / fractionVector.size();
}

/**
 * @brief Calculates the average absolute acceleration of the trajectory.
 */
void Trajectory::calculateAverageAbsoluteAcceleration() {
  const auto fractionVector =
      math::getSubvectorFromVector(m_totalAcceleration, 0, getFractionIndex());
  m_averageAbsoluteAcceleration = math::absSum(fractionVector) / fractionVector.size();
}

/**
 * @brief Gets the index up to which the trajectory is executed. Based on the action execution
 * fraction parameter and the action fraction flag.
 *
 * @return size_t
 */
size_t Trajectory::getFractionIndex() const {
  // index: e.g. 0-20 -> m_nSteps=21
  // 50%: index = 10
  // 100%: index = 20
  return useActionFraction
             ? cOpt().policy_options.policy_enhancements.action_execution_fraction * (m_nSteps - 1)
             : m_nSteps - 1;
}

/**
 * @brief Returns the action execution fraction parameter that is currently being applied for this
 * trajectory object based on the action fraction flag.
 *
 * @return float
 */
float Trajectory::getCurrentFraction() {
  // When useFraction is false, the complete trajectory is executed -> Return 1.0.
  return useActionFraction ? cOpt().policy_options.policy_enhancements.action_execution_fraction
                           : 1.0f;
}

/**
 * @brief Function to allow conversion of a Trajectory to a JSON object.
 * @details Gets called by the json constructor of the nlohmann json library.
 *
 * @param j The JSON object to be filled.
 * @param trajectory The Trajectory to be converted.
 */
void to_json(json& j, const Trajectory& trajectory) {
  j["sPosition"]                   = trajectory.m_sPosition;
  j["dPosition"]                   = trajectory.m_dPosition;
  j["sVelocity"]                   = trajectory.m_sVelocity;
  j["dVelocity"]                   = trajectory.m_dVelocity;
  j["sAcceleration"]               = trajectory.m_sAcceleration;
  j["dAcceleration"]               = trajectory.m_dAcceleration;
  j["curvature"]                   = trajectory.m_curvature;
  j["lane"]                        = trajectory.m_lane;
  j["heading"]                     = trajectory.m_heading;
  j["steeringAngle"]               = trajectory.m_steeringAngle;
  j["laneChange"]                  = trajectory.m_laneChange;
  j["totalVelocity"]               = trajectory.m_totalVelocity;
  j["totalAcceleration"]           = trajectory.m_totalAcceleration;
  j["averageVelocity"]             = trajectory.m_averageVelocity;
  j["averageAbsoluteAcceleration"] = trajectory.m_averageAbsoluteAcceleration;
  j["cumSquaredAccelerationLon"]   = trajectory.m_cumSquaredAccelerationLon;
  j["cumSquaredAccelerationLat"]   = trajectory.m_cumSquaredAccelerationLat;
  j["finalState"]                  = trajectory.m_finalState;
  j["invalidAction"]               = trajectory.m_invalidAction;
  j["invalidState"]                = trajectory.m_invalidState;
  j["useActionFraction"]           = trajectory.useActionFraction;
}
}  // namespace proseco_planning
