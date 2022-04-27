/**
 * @file costExponential.h
 * @brief This file defines the calculation of costs for the MCTS with an exp function.
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
 * @brief CostContinuous class:
 * Calculates the detailed cost for the continuous MCTS search
 * implemented with exp functions
 */

class CostExponential : public CostModel {
 public:
  using CostModel::CostModel;

  explicit CostExponential(const config::CostModel& costModel);

  float updateStatePotential(const Desire& desire, const Vehicle& vehicle) override;

  float calculateActionCost(const Trajectory& trajectory) override;

  float calculatePotentialDeviation(const Desire& desire, const Vehicle& vehicle) const override;

  float calculateStateCost(const Desire& desire, const Vehicle& vehicle, bool collision,
                           bool invalid) override;

  float calculateCooperativeCost(const Desire& desire, const Vehicle& vehicle,
                                 const Trajectory& trajectory, bool collision, bool invalid,
                                 int numberOfAgents, float egoReward,
                                 float cooperationFactor) const override;

 private:
  float potentialVelocityDeviation(const Desire& desire, const Vehicle& vehicle) const override;

  float potentialLaneDeviation(const Desire& desire, const Vehicle& vehicle) const override;

  float potentialLaneCenterDeviation(const Desire& desire, const Vehicle& vehicle) const override;
};
}  // namespace proseco_planning
