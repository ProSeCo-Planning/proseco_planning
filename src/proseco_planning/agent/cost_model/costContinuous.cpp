#include "proseco_planning/agent/cost_model/costContinuous.h"

#include "proseco_planning/agent/cost_model/costModel.h"

namespace proseco_planning {
class Desire;
class Trajectory;
class Vehicle;
namespace config {
struct CostModel;
}  // namespace config

/**
 * @brief Constructs a new Cost Continuous object.
 *
 * @param costModel The config including the weights for the calculation of the cost model.
 */
CostContinuous::CostContinuous(const config::CostModel& costModel) : CostModel(costModel) {}

/**
 * @brief Calculates the cost of the trajectory based on the cost model.
 *
 * @param trajectory The trajectory generated for the action and used for the calculation.
 * @return float The costs used for the evaluation of the action.
 */
float CostContinuous::calculateActionCost(const Trajectory& trajectory) {
  return costLaneChange(trajectory) + costAccelerationX(trajectory) +
         costAccelerationY(trajectory) + costInvalidAction(trajectory);
}
/**
 * @brief Calculates the cost of the state based on the cost model.
 * @note CalculateStateCost is (currently) not used together with cost continuous model.
 * Therefore this will only return 0.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @param collision The check for collision.
 * @param invalid The check for invalid action.
 * @return float Cost of the state.
 */
float CostContinuous::calculateStateCost(const Desire& desire, const Vehicle& vehicle,
                                         bool collision, bool invalid) {
  return 0;
}

/**
 * @brief Calculates the difference between the desire and the state of the vehicle.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The deviation from the desired state.
 */
float CostContinuous::calculatePotentialDeviation(const Desire& desire,
                                                  const Vehicle& vehicle) const {
  return potentialVelocityDeviation(desire, vehicle) + potentialLaneDeviation(desire, vehicle) +
         potentialLaneCenterDeviation(desire, vehicle);
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
float CostContinuous::calculateCooperativeCost(const Desire& desire, const Vehicle& vehicle,
                                               const Trajectory& trajectory, bool collision,
                                               bool invalid, int numberOfAgents, float egoReward,
                                               float cooperationFactor) const {
  return cooperationFactor * egoReward;
}
}  // namespace proseco_planning
