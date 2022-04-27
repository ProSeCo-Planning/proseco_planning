/**
 * @file costContinuous.h
 * @brief This file defines the calculation of the costs for the MCTS.
 *
 * @copyright Copyright (c) 2021
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
 * @brief CostContinuous class: Calculates the costs for the continuous MCTS.
 */

class CostContinuous : public CostModel {
 public:
  using CostModel::CostModel;

  explicit CostContinuous(const config::CostModel& costModel);

  float calculateActionCost(const Trajectory& trajectory) override;

  float calculatePotentialDeviation(const Desire& desire, const Vehicle& vehicle) const override;

  float calculateStateCost(const Desire& desire, const Vehicle& vehicle, bool collision,
                           bool invalid) override;

  float calculateCooperativeCost(const Desire& desire, const Vehicle& vehicle,
                                 const Trajectory& trajectory, bool collision, bool invalid,
                                 int numberOfAgents, float egoReward,
                                 float cooperationFactor) const override;
};
}  // namespace proseco_planning
