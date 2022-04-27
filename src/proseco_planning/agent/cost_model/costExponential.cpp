#include "proseco_planning/agent/cost_model/costExponential.h"

#include <cstdlib>

#include "proseco_planning/agent/cost_model/costModel.h"
#include "proseco_planning/agent/desire.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"

namespace proseco_planning {
class Trajectory;

/**
 * @brief Constructs a new Cost Exponential object
 *
 * @param costModel The config including the weights for the calculation of the cost model.
 */
CostExponential::CostExponential(const config::CostModel& costModel) : CostModel(costModel) {}

/**
 * @brief Updates the the state potential based on the deviation to the desired state.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The deviation from the desired state.
 */
float CostExponential::updateStatePotential(const Desire& desire, const Vehicle& vehicle) {
  return calculatePotentialDeviation(desire, vehicle);
}

/**
 * @brief Calculates the cost of the state based on the cost model.
 * @note CalculateStateCost is (currently) not used together with cost exponential model.
 * Therefore this will only return 0.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @param collision The check for collision.
 * @param invalid The check for invalid action.
 * @return float Cost of the state.
 */
float CostExponential::calculateStateCost(const Desire& desire, const Vehicle& vehicle,
                                          bool collision, bool invalid) {
  return 0;
}

/**
 * @brief Calculates the cost of the trajectory based on the cost model.
 *
 * @param trajectory The trajectory generated for the action and used for the calculation.
 * @return float The costs used for the evaluation of the action.
 */
float CostExponential::calculateActionCost(const Trajectory& trajectory) {
  return costLaneChange(trajectory) + costAccelerationX(trajectory) +
         costAccelerationY(trajectory) + costInvalidAction(trajectory);
}

/**
 * @brief Calculates the difference between the desire and the state of the vehicle.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The deviation from the desired state.
 */
float CostExponential::calculatePotentialDeviation(const Desire& desire,
                                                   const Vehicle& vehicle) const {
  return potentialVelocityDeviation(desire, vehicle) + potentialLaneDeviation(desire, vehicle) +
         potentialLaneCenterDeviation(desire, vehicle);
}

/**
 * @brief The deviation in velocity from the desired state based on an exponential function..
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The deviation in velocity.
 */
float CostExponential::potentialVelocityDeviation(const Desire& desire,
                                                  const Vehicle& vehicle) const {
  return 2.0f * m_wVelocityDeviation *
             std::exp(-0.00745f * std::pow(desire.m_desiredVelocity - vehicle.m_velocityX, 2.0f)) -
         1.0f * m_wVelocityDeviation;
}

/**
 * @brief The deviation in lane from the desired state based on an exponential function..
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The deviation in lane.
 */
float CostExponential::potentialLaneDeviation(const Desire& desire, const Vehicle& vehicle) const {
  // Exponential function
  // return m_wLaneDeviation * std::exp(-0.2f * std::pow(vehicle.m_lane -
  // desire.m_desiredLane, 2.0f));
  return m_wLaneDeviation -
         1.1f * m_wLaneCenterDeviation * std::abs(vehicle.m_lane - desire.m_desiredLane);
}

/**
 * @brief The deviation in lane center from the desired state based on an exponential function.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The deviation in lane center.
 */
float CostExponential::potentialLaneCenterDeviation(const Desire& desire,
                                                    const Vehicle& vehicle) const {
  return m_wLaneCenterDeviation * std::exp(-5.0f * std::pow((vehicle.getDistanceToLaneCenter()) /
                                                                sOpt().road.lane_width * 2.0f,
                                                            2.0f));
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
 * @param egoReward The cost of ego vehicle.
 * @param cooperationFactor The coefficient that weights the incorporation of the rewards of other
 * agents.
 * @return float The cooperative cost of the agent.
 */
float CostExponential::calculateCooperativeCost(const Desire& desire, const Vehicle& vehicle,
                                                const Trajectory& trajectory, bool collision,
                                                bool invalid, int numberOfAgents, float egoReward,
                                                float cooperationFactor) const {
  return cooperationFactor * egoReward;
}
}  // namespace proseco_planning
