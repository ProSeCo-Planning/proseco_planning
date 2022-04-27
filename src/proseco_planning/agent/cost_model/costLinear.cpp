#include "proseco_planning/agent/cost_model/costLinear.h"

#include <algorithm>
#include <cstdlib>

#include "proseco_planning/agent/cost_model/costModel.h"
#include "proseco_planning/agent/desire.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/trajectory/trajectory.h"

namespace proseco_planning {

/**
 * @brief Constructs a new Cost Linear object
 *
 * @param costModel The config including the weights for the calculation of the cost model.
 */
CostLinear::CostLinear(const config::CostModel& costModel) : CostModel(costModel) {}

/**
 * @brief Calculates the cost of the trajectory based on the cost model.
 *
 * @param trajectory The trajectory generated for the action and used for the calculation.
 * @return float The costs used for the evaluation of the action.
 */
float CostLinear::calculateActionCost(const Trajectory& trajectory) {
  return costAccelerationY(trajectory) + costInvalidAction(trajectory);
}

/**
 * @brief Calculates the costs for leaving the trajectory in form of a straight line. The feature is
 * positive as long as the acceleration is smaller than 0.25 g.
 *
 * @param trajectory
 * @return float
 */
float CostLinear::costAccelerationY(const Trajectory& trajectory) const {
  assert(trajectory.m_cumSquaredAccelerationLat >= 0 &&
         "Error the squared sum of the acceleration is negative.");
  const auto gY{std::sqrt(trajectory.m_cumSquaredAccelerationLat / cOpt().action_duration) /
                config::ComputeOptions::gravity};
  return m_wAccelerationY * std::max(1.0f - 4 * gY, -1.0f) / episodeLength;
}

/**
 * @brief Calculates the costs for an invalid action.
 *
 * @param trajectory
 * @return float
 */
float CostLinear::costInvalidAction(const Trajectory& trajectory) const {
  if (trajectory.m_invalidAction) {
    return m_costInvalidAction / episodeLength;
  } else {
    return 0.0f;
  }
}

/**
 * @brief Calculates the state costs based on the cost model.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @param collision The flag for collision checking.
 * @param invalid The flag for invalid state checking.
 * @return float The combined state costs.
 */
float CostLinear::calculateStateCost(const Desire& desire, const Vehicle& vehicle, bool collision,
                                     bool invalid) {
  return costVelocityDeviation(desire, vehicle) + costLaneDeviation(desire, vehicle) +
         costLaneCenterDeviation(desire, vehicle) + costInvalid(invalid) + costCollision(collision);
}

/**
 * @brief Calculates the potential difference based on the velocity deviation.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The difference.
 */
float CostLinear::costVelocityDeviation(const Desire& desire, const Vehicle& vehicle) const {
  return m_wVelocityDeviation *
         std::max(1.0f - std::abs(desire.m_desiredVelocity - vehicle.m_velocityX) /
                             (desire.m_desiredVelocity / 10.0f),
                  -1.0f) /
         episodeLength;
}

/**
 * @brief Calculates the potential difference based on the lane deviation.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The difference.
 */
float CostLinear::costLaneDeviation(const Desire& desire, const Vehicle& vehicle) const {
  return m_wLaneDeviation *
         std::max(1.0f - std::abs(vehicle.m_lane - desire.m_desiredLane), -1.0f) / episodeLength;
}

/**
 * @brief Calculates the potential difference based on the deviation of the lane center. The feature
 * is positive as long as the deviation is smaller than a quarter of the lane width.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The difference.
 */
float CostLinear::costLaneCenterDeviation(const Desire& desire, const Vehicle& vehicle) const {
  return m_wLaneCenterDeviation *
         std::max(
             1.0f - std::abs(vehicle.getDistanceToLaneCenter()) / (sOpt().road.lane_width / 4.0f),
             -1.0f) /
         episodeLength;
}

/**
 * @brief Checks for collision and if true, returns the costs for a collision.
 *
 * @param collided The flag for collision checking.
 * @return float The costs for a collision.
 */
float CostLinear::costCollision(bool collided) const {
  if (collided) {
    return m_costCollision;
  }
  return 0.0f;
}

/**
 * @brief Checks for invalid state and if true, returns the costs for an invalid state.
 *
 * @param invalid Flag for invalid state checking.
 * @return float The costs for an invalid state.
 */
float CostLinear::costInvalid(bool invalid) const {
  if (invalid) {
    return m_costInvalidState;
  }
  return 0.0f;
}

/**
 * @brief Calculates the difference between the desire and the state of the vehicle.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The deviation from the desired state.
 */
float CostLinear::calculatePotentialDeviation(const Desire& desire, const Vehicle& vehicle) const {
  return 0.0f;
}

/**
 * @brief Calculates the cooperative cost of the agent.
 *
 * @param desire
 * @param vehicle
 * @param trajectory
 * @param collision
 * @param invalid
 * @param numberOfAgents
 * @param egoReward
 * @param cooperationFactor
 * @return float The cooperative cost of the agent, which is 0.
 */
float CostLinear::calculateCooperativeCost(const Desire& desire, const Vehicle& vehicle,
                                           const Trajectory& trajectory, bool collision,
                                           bool invalid, int numberOfAgents, float egoReward,
                                           float cooperationFactor) const {
  return 0.0f;
}
}  // namespace proseco_planning
