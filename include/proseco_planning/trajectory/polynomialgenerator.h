/**
 * @file polynomialgenerator.h
 * @brief This file defines the PolynomialGenerator class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <cstddef>
#include <tuple>
#include <vector>

#include <eigen3/Eigen/Core>
#include "proseco_planning/util/alias.h"
#include "trajectorygenerator.h"

namespace proseco_planning {
class Trajectory;
class Vehicle;
/*
 * @brief The PolynomialGenerator class defines the polynomial trajectory generator.
 * @details It uses 5th order polynomials in both lateral and longitudinal direction.
 */

class PolynomialGenerator : public TrajectoryGenerator {
 public:
  using TrajectoryGenerator::TrajectoryGenerator;

 private:
  Eigen::Matrix<float, 1, 6> calculate_coefficients(const Eigen::Matrix<float, 1, 6>& constraints,
                                                    float t_0, float t_1) const;

  Eigen::Matrix<float, 1, 6> boundaryConditionsToEigenMatrix(const BoundaryCondition& start,
                                                             const BoundaryCondition& end) const;

  float calculate_position(const Eigen::Matrix<float, 1, 6>& coeff, const float t) const;

  float calculate_velocity(const Eigen::Matrix<float, 1, 6>& coeff, const float t) const;

  float calculate_acceleration(const Eigen::Matrix<float, 1, 6>& coeff, const float t) const;

  float calculate_heading(const Trajectory& trajectory, const size_t i,
                          const Vehicle& vehicle) const;

  float calculate_curvature(const Trajectory& trajectory, const size_t i) const;

  float calculate_steering_angle(const Trajectory& trajectory, const size_t i,
                                 const Vehicle& vehicle) const;

  float calculate_total_velocity(const Trajectory& trajectory, const size_t i) const;

  float calculate_total_acceleration(const Trajectory& trajectory, const size_t i) const;

  float squared_acceleration_integral(const BoundaryCondition& start, const BoundaryCondition& end,
                                      const float t_0, const float t_1) const;

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
};
}  // namespace proseco_planning