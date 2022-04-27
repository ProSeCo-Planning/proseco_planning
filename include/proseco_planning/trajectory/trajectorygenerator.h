/**
 * @file trajectorygenerator.h
 * @brief This file defines the TrajectoryGenerator class, it is the base class for all trajectory
 * generators.
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Trajectory;
class Vehicle;

/**
 * @brief The boundary condition defines the constraints for the trajectory generation.
 */
struct BoundaryCondition {
  /// The position that constraints the trajectory.
  float position{0.0f};

  /// The velocity that constraints the trajectory.
  float velocity{0.0f};

  /// The acceleration that constraints the trajectory.
  float acceleration{0.0f};
};

/*
 * @brief The TrajectoryGenerator class defines the base class for all trajectory generators.
 */

class TrajectoryGenerator {
 public:
  explicit TrajectoryGenerator(const std::string& name);

  /// The virtual destructor.
  virtual ~TrajectoryGenerator() = default;

  static std::unique_ptr<TrajectoryGenerator> createTrajectoryGenerator(const std::string& name);

  Trajectory createTrajectory(float t0, ActionPtr action, const Vehicle& vehicle) const;

  /// The name of the method for trajectory generation.
  std::string m_name;

 protected:
  virtual std::tuple<BoundaryCondition, BoundaryCondition, BoundaryCondition, BoundaryCondition>
  createBoundaryConditions(ActionPtr action, const Vehicle& vehicle) const = 0;

  virtual Trajectory calculateTrajectory(const Vehicle& vehicle, const float t0,
                                         const BoundaryCondition& startS,
                                         const BoundaryCondition& startD,
                                         const BoundaryCondition& endS,
                                         const BoundaryCondition& endD) const = 0;

  virtual void updateFinalState(Trajectory& trajectory) const = 0;

  virtual void evaluateTrajectory(Trajectory& trajectory, const BoundaryCondition& startS,
                                  const BoundaryCondition& startD, const BoundaryCondition& endS,
                                  const BoundaryCondition& endD) const;

  virtual void calculateCumulativeAcceleration(Trajectory& trajectory,
                                               const BoundaryCondition& startS,
                                               const BoundaryCondition& startD,
                                               const BoundaryCondition& endS,
                                               const BoundaryCondition& endD) const = 0;

  void detectLaneChange(Trajectory& trajectory) const;
};
}  // namespace proseco_planning