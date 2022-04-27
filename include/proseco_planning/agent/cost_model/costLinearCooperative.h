/**
 * @file costLinearCooperative.h
 * @brief This file defines the calculation of costs for the MCTS with a linear function as well as
 * Cooperation.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once
#include "costModel.h"

namespace proseco_planning {

class Desire;
class Trajectory;
class Vehicle;
namespace config {
struct CostModel;
}  // namespace config

/**
 * @brief CostLinearCooperative class: Calculates the costs for the MCTS with a linear function as
 * well as Cooperation.
 */
class CostLinearCooperative : public CostModel {
 public:
  using CostModel::CostModel;

  explicit CostLinearCooperative(const config::CostModel& costModel);

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

  static float featureCollision(bool collided);

  static float featureInvalid(bool invalid);

  float featureAccelerationY(const Trajectory& trajectory) const;

  float featureInvalidAction(const Trajectory& trajectory) const;

  ///@{
  /// Cooperative cost paramaters for costLinearCooperative (learned by the IRL procedure).*/
  float m_wAccelerationYCooperative{0.0f};
  float m_wLaneDeviationCooperative{0.0f};
  float m_wLaneCenterDeviationCooperative{0.0f};
  float m_wVelocityDeviationCooperative{0.0f};
  float m_costCollisionCooperative{0.0f};
  float m_costInvalidStateCooperative{0.0f};
  float m_costInvalidActionCooperative{0.0f};
  ///@}

  /**
   * @brief The length of an episode in steps.
   * @note This is currently only used for the IRL algorithm.
   *
   */
  static constexpr float episodeLength{13.0f};
};
}  // namespace proseco_planning
