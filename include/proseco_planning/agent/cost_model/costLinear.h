/**
 * @file costLinear.h
 * @brief This file defines the calculation of costs for the MCTS with a linear function.
 *
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
 * @brief CostLinear class: Calculates the costs for the MCTS with a linear function.
 *
 */
class CostLinear : public CostModel {
 public:
  using CostModel::CostModel;

  explicit CostLinear(const config::CostModel& costModel);

  float calculateActionCost(const Trajectory& trajectory) override;

  float calculatePotentialDeviation(const Desire& desire, const Vehicle& vehicle) const override;

  float calculateStateCost(const Desire& desire, const Vehicle& vehicle, bool collision,
                           bool invalid) override;

  float calculateCooperativeCost(const Desire& desire, const Vehicle& vehicle,
                                 const Trajectory& trajectory, bool collision, bool invalid,
                                 int numberOfAgents, float egoReward,
                                 float cooperationFactor) const override;

  float costVelocityDeviation(const Desire& desire, const Vehicle& vehicle) const;

  float costLaneDeviation(const Desire& desire, const Vehicle& vehicle) const;

  float costLaneCenterDeviation(const Desire& desire, const Vehicle& vehicle) const;

  float costCollision(bool collided) const;

  float costInvalid(bool invalid) const;

  float costAccelerationY(const Trajectory& trajectory) const override;

  float costInvalidAction(const Trajectory& trajectory) const override;

  /**
   * @brief The length of an episode in steps.
   * @note This is currently only used for the IRL algorithm.
   *
   */
  static constexpr float episodeLength{13.0f};
};
}  // namespace proseco_planning
