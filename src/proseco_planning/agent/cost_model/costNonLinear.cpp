#include "proseco_planning/agent/cost_model/costNonLinear.h"

#include <algorithm>
#include <cstdlib>
#include <eigen3/Eigen/Core>

#include "proseco_planning/agent/desire.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/trajectory/trajectory.h"

namespace proseco_planning {
/**
 * @brief Constructs a new Cost Non Linear object.
 *
 * @param costModel The config including the weights for the calculation of the cost model.
 */
CostNonLinear::CostNonLinear(const config::CostModel& costModel) : CostModel(costModel) {
  m_W1 = costModel.w1;
  m_W2 = costModel.w2;
}

/**
 * @brief Calculates the cost of the agent.
 *
 * @param desire The desire of the agent.
 * @param vehicle The state of the vehicle.
 * @param vehiclePreviousStep The previous state of the vehicle.
 * @param collision Flag for collision checking.
 * @param invalid Flag for invalid state checking.
 * @param trajectory The current trajectory of the vehicle.
 * @return float The cost.
 */
float CostNonLinear::calculateCost(const Desire& desire, const Vehicle& vehicle,
                                   const Vehicle& vehiclePreviousStep, bool collision, bool invalid,
                                   const Trajectory& trajectory) {
  Eigen::VectorXd input(10);
  input(0) = featureVelocityDeviation(desire, vehicle);
  input(1) = featureLaneDeviation(desire, vehicle);
  input(2) = featureLaneCenterDeviation(desire, vehicle);
  input(3) = featureCollision(collision);
  input(4) = featureInvalid(invalid);
  input(5) = featureInvalidAction(trajectory);
  input(6) = featureAccelerationY(trajectory);
  input(7) = featureVelocityDeviation(desire, vehiclePreviousStep);
  input(8) = featureLaneDeviation(desire, vehiclePreviousStep);
  input(9) = featureLaneCenterDeviation(desire, vehiclePreviousStep);

  return forwardPass(input);
}

/**
 * @brief Runs the forward pass of the neural network.
 *
 * @param input The input vector.
 * @return float The cost.
 */
float CostNonLinear::forwardPass(const Eigen::VectorXd& input) const {
  const auto hiddenLayerOut = ReLU(input.transpose() * m_W1);
  return (hiddenLayerOut.transpose() * m_W2)(0);
}

/**
 * @brief Applies the ReLU function to the input vector.
 *
 * @param input The input vector.
 * @return Eigen::VectorXd The output vector.
 */
Eigen::VectorXd CostNonLinear::ReLU(const Eigen::VectorXd& input) const {
  return input.array().cwiseMax(0.0);
}

/**
 * @brief Applies the Leaky ReLU function to the input vector.
 *
 * @param input The input vector.
 * @return Eigen::VectorXd The output vector.
 */
Eigen::VectorXd CostNonLinear::LeakyReLU(const Eigen::VectorXd& input) const {
  return input.array().cwiseMax(input.array() * 0.01);
}

/**
 * @brief Calculates the potential difference based on the velocity deviation.
 *
 * @param desire The desire of the agent.
 * @param vehicle The state of the vehicle.
 * @return float The difference.
 */
float CostNonLinear::featureVelocityDeviation(const Desire& desire, const Vehicle& vehicle) const {
  return std::max(1.0f - std::abs(desire.m_desiredVelocity - vehicle.m_velocityX) /
                             (std::abs(desire.m_desiredVelocity) / 10.0f),
                  -1.0f) /
         episodeLength;
}

/**
 * @brief Calculates the potential difference based on the lane deviation.
 *
 * @param desire The desire of the agent.
 * @param vehicle The state of the vehicle.
 * @return float The difference.
 */
float CostNonLinear::featureLaneDeviation(const Desire& desire, const Vehicle& vehicle) const {
  return std::max(1.0f - std::abs(vehicle.m_lane - desire.m_desiredLane), -1.0f) / episodeLength;
}

/**
 * @brief Calculates the potential difference based on the deviation of the lane center.
 *
 * @param desire The desire of the agent.
 * @param vehicle The state of the vehicle.
 * @return float The difference.
 */
float CostNonLinear::featureLaneCenterDeviation(const Desire& desire,
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
float CostNonLinear::featureCollision(bool collided) {
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
float CostNonLinear::featureInvalid(bool invalid) {
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
float CostNonLinear::featureAccelerationY(const Trajectory& trajectory) const {
  float costAccLat{trajectory.m_cumSquaredAccelerationLat};
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
float CostNonLinear::featureInvalidAction(const Trajectory& trajectory) const {
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
 * @param vehicle The state of the vehicle.
 * @param collision The flag for collision checking.
 * @param invalid The flag for invalid state checking.
 * @return float The combined state costs.
 */
float CostNonLinear::calculateStateCost(const Desire& desire, const Vehicle& vehicle,
                                        bool collision, bool invalid) {
  return 0.0f;
}

/**
 * @brief Calculates the cost of the trajectory based on the cost model.
 *
 * @param trajectory The trajectory generated for the action and used for the calculation.
 * @return float The costs used for the evaluation of the action.
 */
float CostNonLinear::calculateActionCost(const Trajectory& trajectory) { return 0.0f; }

/**
 * @brief Calculates the difference between the desire and the state of the vehicle.
 *
 * @param desire The desire of the agent.
 * @param vehicle The state of the vehicle.
 * @return float The deviation from the desired state.
 */
float CostNonLinear::calculatePotentialDeviation(const Desire& desire,
                                                 const Vehicle& vehicle) const {
  return 0.0f;
}

/**
 * @brief Calculates the cooperative cost of the agent.
 *
 * @param desire The desire of the agent.
 * @param vehicle The state of the vehicle.
 * @param trajectory The trajectory of the agent.
 * @param collision Flag for collision checking.
 * @param invalid Flag for invalid state checking.
 * @param numberOfAgents Number of agents.
 * @param egoReward The cost of ego vehicle.
 * @param cooperationFactor The coefficient that weights the incorporation of the rewards of other
 * agents.
 * @return float The cooperative cost of the agent.
 */
float CostNonLinear::calculateCooperativeCost(const Desire& desire, const Vehicle& vehicle,
                                              const Trajectory& trajectory, bool collision,
                                              bool invalid, int numberOfAgents, float egoReward,
                                              float cooperationFactor) const {
  return 0.0f;
}
}  // namespace proseco_planning
