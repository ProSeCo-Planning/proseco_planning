/**
 * @file costNonLinear.h
 * @brief This file defines the calculation of costs for the MCTS with a non linear function.
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <eigen3/Eigen/Core>
#include "proseco_planning/agent/cost_model/costModel.h"

namespace proseco_planning {

class Desire;
class Trajectory;
class Vehicle;
namespace config {
struct CostModel;
}  // namespace config

/**
 * @brief CostNonLinear class: Calculates the costs for the MCTS with a non linear function.
 *
 */
class CostNonLinear : public CostModel {
 public:
  using CostModel::CostModel;

  explicit CostNonLinear(const config::CostModel& costModel);

  float calculateCost(const Desire& desire, const Vehicle& vehicle,
                      const Vehicle& vehiclePreviousStep, bool collision, bool invalid,
                      const Trajectory& trajectory) override;

  float calculateStateCost(const Desire& desire, const Vehicle& vehicle, bool collision,
                           bool invalid) override;

  float calculateActionCost(const Trajectory& trajectory) override;

  float calculatePotentialDeviation(const Desire& desire, const Vehicle& vehicle) const override;

  float calculateCooperativeCost(const Desire& desire, const Vehicle& vehicle,
                                 const Trajectory& trajectory, bool collision, bool invalid,
                                 int numberOfAgents, float egoReward,
                                 float cooperationFactor) const override;

  float featureVelocityDeviation(const Desire& desire, const Vehicle& vehicle) const;

  float featureLaneDeviation(const Desire& desire, const Vehicle& vehicle) const;

  float featureLaneCenterDeviation(const Desire& desire, const Vehicle& vehicle) const;

  float featureAccelerationY(const Trajectory& trajectory) const;

  float featureInvalidAction(const Trajectory& trajectory) const;

  static float featureCollision(bool collided);

  static float featureInvalid(bool invalid);

  Eigen::VectorXd ReLU(const Eigen::VectorXd& input) const;

  Eigen::VectorXd LeakyReLU(const Eigen::VectorXd& input) const;

  float forwardPass(const Eigen::VectorXd& input) const;

  /// The first layer weights of the neural network for the cost model. This is used in IRL.
  Eigen::MatrixXd m_W1;

  /// The second layer weights of the neural network for the cost model. This is used in IRL.
  Eigen::MatrixXd m_W2;

  /**
   * @brief The length of an episode in steps.
   * @note This is currently only used for the IRL algorithm.
   */
  static constexpr float episodeLength{13.0f};
};
}  // namespace proseco_planning