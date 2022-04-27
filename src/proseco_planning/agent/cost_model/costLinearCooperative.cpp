#include "proseco_planning/agent/cost_model/costLinearCooperative.h"

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
 * @brief Constructs a new Cost Linear Cooperative object.
 *
 * @param costModel The config including the weights for the calculation of the cost model.
 */
CostLinearCooperative::CostLinearCooperative(const config::CostModel& costModel)
    : CostModel(costModel) {
  m_wAccelerationYCooperative       = costModel.w_acceleration_y_cooperative;
  m_wLaneDeviationCooperative       = costModel.w_lane_deviation_cooperative;
  m_wLaneCenterDeviationCooperative = costModel.w_lane_center_deviation_cooperative;
  m_wVelocityDeviationCooperative   = costModel.w_velocity_deviation_cooperative;
  m_costCollisionCooperative        = costModel.cost_collision_cooperative;
  m_costInvalidStateCooperative     = costModel.cost_invalid_state_cooperative;
  m_costInvalidActionCooperative    = costModel.cost_invalid_action_cooperative;
}

/**
 * @brief Calculates the cost of the agent.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @param vehiclePreviousStep The previous state of the vehicle.
 * @param collision Flag for collision checking.
 * @param invalid Flag for invalid state checking.
 * @param trajectory The current trajectory of the vehicle.
 * @return float The cost.
 */
float CostLinearCooperative::calculateCost(const Desire& desire, const Vehicle& vehicle,
                                           const Vehicle& vehiclePreviousStep, bool collision,
                                           bool invalid, const Trajectory& trajectory) {
  float reward = m_wVelocityDeviation * featureVelocityDeviation(desire, vehicle) +
                 m_wLaneDeviation * featureLaneDeviation(desire, vehicle) +
                 m_wLaneCenterDeviation * featureLaneCenterDeviation(desire, vehicle) +
                 m_costCollision * featureCollision(collision) +
                 m_costInvalidState * featureInvalid(invalid) +
                 m_costInvalidAction * featureInvalidAction(trajectory) +
                 m_wAccelerationY * featureAccelerationY(trajectory);

  return reward;
}

/**
 * @brief Calculates the potential difference based on the velocity deviation.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The difference.
 */
float CostLinearCooperative::featureVelocityDeviation(const Desire& desire,
                                                      const Vehicle& vehicle) const {
  return std::max(1.0f - std::abs(desire.m_desiredVelocity - vehicle.m_velocityX) /
                             (std::abs(desire.m_desiredVelocity) / 10.0f),
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
float CostLinearCooperative::featureLaneDeviation(const Desire& desire,
                                                  const Vehicle& vehicle) const {
  return std::max(1.0f - std::abs(vehicle.m_lane - desire.m_desiredLane), -1.0f) / episodeLength;
}

/**
 * @brief Calculates the potential difference based on the deviation of the lane center.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The difference.
 */
float CostLinearCooperative::featureLaneCenterDeviation(const Desire& desire,
                                                        const Vehicle& vehicle) const {
  return std::max(
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
float CostLinearCooperative::featureCollision(bool collided) {
  if (collided) {
    return 1.0f;
  }
  return 0.0f;
}

/**
 * @brief Checks for invalid state and if true, returns the costs for an invalid state.
 *
 * @param invalid Flag for invalid state checking.
 * @return float The costs for an invalid state.
 */
float CostLinearCooperative::featureInvalid(bool invalid) {
  if (invalid) {
    return 1.0f;
  }
  return 0.0f;
}

/**
 * @brief Calculates the costs for leaving the trajectory in form of a straight line.
 *
 * @param trajectory
 * @return float
 */
float CostLinearCooperative::featureAccelerationY(const Trajectory& trajectory) const {
  float costAccLat = {trajectory.m_cumSquaredAccelerationLat};
  if (costAccLat < 0.0f) {
    costAccLat = 0.0f;
  }

  float sqrtAccy{std::sqrt(0.5f * costAccLat) / (0.25f * config::ComputeOptions::gravity)};
  return std::max(1.0f - std::pow(sqrtAccy, 2.0f), -1.0f) / episodeLength;
}

/**
 * @brief Calculates the costs for an invalid action.
 *
 * @param trajectory
 * @return float
 */
float CostLinearCooperative::featureInvalidAction(const Trajectory& trajectory) const {
  if (trajectory.m_invalidAction) {
    return (1.0f / episodeLength);
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
float CostLinearCooperative::calculateStateCost(const Desire& desire, const Vehicle& vehicle,
                                                bool collision, bool invalid) {
  return 0.0f;
}

/**
 * @brief Calculates the cost of the trajectory based on the cost model.
 *
 * @param trajectory The trajectory generated for the action and used for the calculation.
 * @return float The costs used for the evaluation of the action.
 */
float CostLinearCooperative::calculateActionCost(const Trajectory& trajectory) { return 0.0f; }

/**
 * @brief Calculates the difference between the desire and the state of the vehicle.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The deviation from the desired state.
 */
float CostLinearCooperative::calculatePotentialDeviation(const Desire& desire,
                                                         const Vehicle& vehicle) const {
  return 0.0f;
}

/**
 * @brief Calculates the cooperative cost of the agent.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @param trajectory The trajectory of the agent.
 * @param collision Flag for collision checking.
 * @param invalid Flag for invalid state checking.
 * @param numberOfAgents Number of agents.
 * @param egoReward The cost of ego vehicle.
 * @param cooperationFactor The coefficient that weights the incorporation of the rewards of other
 * agents.
 * @return float The cooperative cost of the agent.
 */
float CostLinearCooperative::calculateCooperativeCost(const Desire& desire, const Vehicle& vehicle,
                                                      const Trajectory& trajectory, bool collision,
                                                      bool invalid, int numberOfAgents,
                                                      float egoReward,
                                                      float cooperationFactor) const {
  float reward{m_wVelocityDeviationCooperative * featureVelocityDeviation(desire, vehicle) +
               m_wLaneDeviationCooperative * featureLaneDeviation(desire, vehicle) +
               m_wLaneCenterDeviationCooperative * featureLaneCenterDeviation(desire, vehicle) +
               m_costCollisionCooperative * featureCollision(collision) +
               m_costInvalidStateCooperative * featureInvalid(invalid) +
               m_costInvalidActionCooperative * featureInvalidAction(trajectory) +
               m_wAccelerationYCooperative * featureAccelerationY(trajectory)};

  return reward * (1.0f / (numberOfAgents - 1.0f));
}

}  // namespace proseco_planning
