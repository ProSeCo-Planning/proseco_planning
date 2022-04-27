#include "proseco_planning/agent/cost_model/costModel.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

#include "proseco_planning/agent/cost_model/costContinuous.h"
#include "proseco_planning/agent/cost_model/costExponential.h"
#include "proseco_planning/agent/cost_model/costLinear.h"
#include "proseco_planning/agent/cost_model/costLinearCooperative.h"
#include "proseco_planning/agent/cost_model/costNonLinear.h"
#include "proseco_planning/agent/desire.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/trajectory/trajectory.h"

namespace proseco_planning {

/**
 * @brief Constructs a new Cost Model object.
 *
 * @param costModel The config including the weights for the calculation of the cost model.
 */
CostModel::CostModel(const config::CostModel& costModel) {
  m_type                 = costModel.name;
  m_wLaneChange          = costModel.w_lane_change;
  m_wAccelerationX       = costModel.w_acceleration_x;
  m_wAccelerationY       = costModel.w_acceleration_y;
  m_wLaneDeviation       = costModel.w_lane_deviation;
  m_wLaneCenterDeviation = costModel.w_lane_center_deviation;
  m_wVelocityDeviation   = costModel.w_velocity_deviation;
  m_costCollision        = costModel.cost_collision;
  m_costInvalidState     = costModel.cost_invalid_state;
  m_costInvalidAction    = costModel.cost_invalid_action;
  m_rewardTerminal       = costModel.reward_terminal;
  m_costEnterSafeRange   = costModel.cost_enter_safe_range;
}

/**
 * @brief Creates a cost model.
 *
 * @param costModel The config including the weights for the calculation of the cost model.
 * @return std::shared_ptr<CostModel>
 */
std::shared_ptr<CostModel> CostModel::createCostModel(const config::CostModel& costModel) {
  auto type = costModel.name;
  if (type == "costContinuous") {
    return std::make_shared<CostContinuous>(costModel);
  } else if (type == "costExponential") {
    return std::make_shared<CostExponential>(costModel);
  } else if (type == "costLinear") {
    return std::make_shared<CostLinear>(costModel);
  } else if (type == "costNonLinear") {
    return std::make_shared<CostNonLinear>(costModel);
  } else if (type == "costLinearCooperative") {
    return std::make_shared<CostLinearCooperative>(costModel);
  } else {
    throw std::invalid_argument("Unknown cost model type: " + type);
  }
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
float CostModel::calculateCost(const Desire& desire, const Vehicle& vehicle,
                               const Vehicle& vehiclePreviousStep, bool collision, bool invalid,
                               const Trajectory& trajectory) {
  /// @todo What is happening here? This needs to be looked at!
  return 0;
}

/**
 * @brief Calculates the potential deviation of the current state to the desired state.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 */
void CostModel::initializeMaximumStatePotential(const Desire& desire, const Vehicle& vehicle) {
  m_finalPotential = calculatePotentialDeviation(desire, vehicle);
}

/**
 * @brief Calculates the potential of the current state.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The potential.
 */
float CostModel::updateStatePotential(const Desire& desire, const Vehicle& vehicle) {
  float stateDeviation = calculatePotentialDeviation(desire, vehicle);

  return m_finalPotential - stateDeviation;
}

/**
 * @brief Updates the state reward and improvement of state potential respectively.
 *
 * @param currentPotential The current potential of an agent.
 * @param oldPotential The previous potential of an agent.
 * @return float The updated state reward.
 */
float CostModel::updateStateReward(float currentPotential, float oldPotential) {
  return cOpt().discount_factor * currentPotential - oldPotential;
}

/**
 * @brief Calculates the cost for acceleration in x direction.
 *
 * @param trajectory The trajectory generated for the action and used for the calculation.
 * @return float The cost.
 */
float CostModel::costAccelerationX(const Trajectory& trajectory) const {
  return m_wAccelerationX * trajectory.m_cumSquaredAccelerationLon;
}

/**
 * @brief Calculates the cost for acceleration in y direction.
 *
 * @param trajectory The trajectory generated for the action and used for the calculation.
 * @return float The cost.
 */
float CostModel::costAccelerationY(const Trajectory& trajectory) const {
  return m_wAccelerationY * trajectory.m_cumSquaredAccelerationLat;
}

/**
 * @brief Calculates cost for lane change.
 *
 * @param trajectory The trajectory generated for the action and used for the calculation.
 * @return float The cost.
 */
float CostModel::costLaneChange(const Trajectory& trajectory) const {
  return m_wLaneChange * trajectory.m_laneChange * trajectory.m_laneChange;
}

/**
 * @brief Gets cost if action is invalid.
 *
 * @param trajectory The trajectory generated for the action and used for the calculation.
 * @return float The cost.
 */
float CostModel::costInvalidAction(const Trajectory& trajectory) const {
  return trajectory.m_invalidAction ? m_costInvalidAction : 0;
}

/**
 * @brief Calculates the potential difference based on the desired longitudinal velocity.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The potential difference.
 */
float CostModel::potentialVelocityDeviation(const Desire& desire, const Vehicle& vehicle) const {
  return m_wVelocityDeviation * std::abs(desire.m_desiredVelocity - vehicle.m_velocityX);
}

/**
 * @brief Calculates the potential difference based on the desired lane.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The potential difference.
 */
float CostModel::potentialLaneDeviation(const Desire& desire, const Vehicle& vehicle) const {
  return m_wLaneDeviation * std::abs(vehicle.m_lane - desire.m_desiredLane);
}

/**
 * @brief Calculates the potential difference based on the deviation from the desired lane center.
 *
 * @param desire The desire of the agent.
 * @param vehicle The vehicle of the agent.
 * @return float The potential difference.
 */
float CostModel::potentialLaneCenterDeviation(const Desire& desire, const Vehicle& vehicle) const {
  return m_wLaneCenterDeviation * std::abs(vehicle.getDistanceToLaneCenter()) /
         sOpt().road.lane_width;
}

/**
 * @brief Calculates the cost for trespassing the safe range of the other vehicle.
 *
 * @param vehicle0 The vehicle of the agent.
 * @param vehicle1 Another vehicle.
 * @return float The cost.
 */
float CostModel::costSafeRange(const Vehicle& vehicle0, const Vehicle& vehicle1) const {
  float dist{0};

  float v0{vehicle0.m_velocityX};
  float v1{vehicle1.m_velocityX};

  float s0{vehicle0.m_positionX};
  float s1{vehicle1.m_positionX};

  if (s1 - s0 > 0) {
    dist = s1 - s0 - vehicle0.m_length / 2 - vehicle1.m_length / 2;
  } else {
    dist = s1 - s0 + vehicle0.m_length / 2 + vehicle1.m_length / 2;
  }

  float ttc = dist / (v0 - v1);

  // if they are not in the same lane OR will never collide, return 0
  if (vehicle0.m_lane != vehicle1.m_lane || ttc < 0 || ttc >= cOpt().action_duration) {
    return 0;
  } else {
    float lambda =
        -std::pow(cOpt().action_duration, 2) / std::log(m_costEnterSafeRange / m_costCollision);
    return m_costCollision * std::exp(-std::pow(ttc, 2) / lambda);
  }
}

}  // namespace proseco_planning
