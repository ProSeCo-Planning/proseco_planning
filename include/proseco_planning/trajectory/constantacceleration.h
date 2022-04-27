/**
 * @file constantacceleration.h
 * @brief This file defines the ConstantAcceleration class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <tuple>
#include <vector>

#include "proseco_planning/util/alias.h"
#include "trajectorygenerator.h"

namespace proseco_planning {
class Trajectory;
class Vehicle;

/*
 * @brief The ConstantAcceleration class defines the constant acceleration trajectory generator.
 */
class ConstantAcceleration : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

 private:
  std::tuple<BoundaryCondition, BoundaryCondition, BoundaryCondition, BoundaryCondition>
  createBoundaryConditions(ActionPtr action, const Vehicle& vehicle) const override;

  Trajectory calculateTrajectory(const Vehicle& vehicle, const float t0,
                                 const BoundaryCondition& startS, const BoundaryCondition& startD,
                                 const BoundaryCondition& endS,
                                 const BoundaryCondition& endD) const override;

  void updateFinalState(Trajectory& trajectory) const override;

  void calculateCumulativeAcceleration(Trajectory& trajectory, const BoundaryCondition& startS,
                                       const BoundaryCondition& startD,
                                       const BoundaryCondition& endS,
                                       const BoundaryCondition& endD) const override;

  float position(float time, float position, float velocity, float acceleration) const;

  float velocity(float time, float velocity, float acceleration) const;
};
}  // namespace proseco_planning