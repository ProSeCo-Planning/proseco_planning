#include "proseco_planning/trajectory/polynomialgenerator.h"

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <memory>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/trajectory/trajectory.h"
namespace proseco_planning {

/**
 * @brief Calculates the coefficients of the fifth order polynomial for a given set of constraints.
 *
 * @param constraints The start as well as end constraints for position, velocity and acceleration.
 * @param t_0 The start time of the trajectory.
 * @param t_1 The end time of the trajectory.
 * @return Eigen::Matrix<float, 1, 6> The coefficients of the fifth order polynomial.
 */
Eigen::Matrix<float, 1, 6> PolynomialGenerator::calculate_coefficients(
    const Eigen::Matrix<float, 1, 6>& constraints, const float t_0, const float t_1) const {
  Eigen::Matrix<float, 6, 6> M;

  // t_0 x - position
  M(0, 0) = 1;
  M(0, 1) = t_0;
  M(0, 2) = t_0 * t_0;
  M(0, 3) = t_0 * t_0 * t_0;
  M(0, 4) = t_0 * t_0 * t_0 * t_0;
  M(0, 5) = t_0 * t_0 * t_0 * t_0 * t_0;
  // t_0 x_dot - velocity
  M(1, 0) = 0;
  M(1, 1) = 1;
  M(1, 2) = 2 * t_0;
  M(1, 3) = 3 * t_0 * t_0;
  M(1, 4) = 4 * t_0 * t_0 * t_0;
  M(1, 5) = 5 * t_0 * t_0 * t_0 * t_0;
  // t_0 x_dot_dot - acceleration
  M(2, 0) = 0;
  M(2, 1) = 0;
  M(2, 2) = 2;
  M(2, 3) = 6 * t_0;
  M(2, 4) = 12 * t_0 * t_0;
  M(2, 5) = 20 * t_0 * t_0 * t_0;

  // t_1 x - position
  M(3, 0) = 1;
  M(3, 1) = t_1;
  M(3, 2) = t_1 * t_1;
  M(3, 3) = t_1 * t_1 * t_1;
  M(3, 4) = t_1 * t_1 * t_1 * t_1;
  M(3, 5) = t_1 * t_1 * t_1 * t_1 * t_1;
  // t_1 x_dot - velocity
  M(4, 0) = 0;
  M(4, 1) = 1;
  M(4, 2) = 2 * t_1;
  M(4, 3) = 3 * t_1 * t_1;
  M(4, 4) = 4 * t_1 * t_1 * t_1;
  M(4, 5) = 5 * t_1 * t_1 * t_1 * t_1;
  // t_1  x_dot_dot - acceleration
  M(5, 0) = 0;
  M(5, 1) = 0;
  M(5, 2) = 2;
  M(5, 3) = 6 * t_1;
  M(5, 4) = 12 * t_1 * t_1;
  M(5, 5) = 20 * t_1 * t_1 * t_1;

  return M.partialPivLu().solve(constraints.transpose()).transpose();
}

/**
 * @brief Converts the boundary conditions to an Eigen::Matrix.
 *
 * @param start The start boundary condition.
 * @param end The end boundary condition.
 * @return Eigen::Matrix<float, 1, 6> The boundary conditions as an Eigen::Matrix.
 */
Eigen::Matrix<float, 1, 6> PolynomialGenerator::boundaryConditionsToEigenMatrix(
    const BoundaryCondition& start, const BoundaryCondition& end) const {
  Eigen::Matrix<float, 1, 6> constraints;
  constraints(0, 0) = start.position;
  constraints(0, 1) = start.velocity;
  constraints(0, 2) = start.acceleration;
  constraints(0, 3) = end.position;
  constraints(0, 4) = end.velocity;
  constraints(0, 5) = end.acceleration;
  return constraints;
}

/**
 * @brief Calculates the position of the fifth order polynomial at a given time.
 *
 * @param coeff The coefficients of the fifth order polynomial.
 * @param t The time at which the position should be calculated.
 * @return float The position at the given time.
 */
float PolynomialGenerator::calculate_position(const Eigen::Matrix<float, 1, 6>& coeff,
                                              const float t) const {
  return coeff(0) + coeff(1) * t + coeff(2) * t * t + coeff(3) * t * t * t +
         coeff(4) * t * t * t * t + coeff(5) * t * t * t * t * t;
}

/**
 * @brief Calculates the velocity of the fifth order polynomial at a given time.
 *
 * @param coeff The coefficients of the fifth order polynomial.
 * @param t The time at which the velocity should be calculated.
 * @return float The velocity at the given time.
 */
float PolynomialGenerator::calculate_velocity(const Eigen::Matrix<float, 1, 6>& coeff,
                                              const float t) const {
  return coeff(1) + 2 * coeff(2) * t + 3 * coeff(3) * t * t + 4 * coeff(4) * t * t * t +
         5 * coeff(5) * t * t * t * t;
}

/**
 * @brief Calculates the acceleration of the fifth order polynomial at a given time.
 *
 * @param coeff The coefficients of the fifth order polynomial.
 * @param t The time at which the acceleration should be calculated.
 * @return float The acceleration at the given time.
 */
float PolynomialGenerator::calculate_acceleration(const Eigen::Matrix<float, 1, 6>& coeff,
                                                  const float t) const {
  return 2 * coeff(2) + 6 * coeff(3) * t + 12 * coeff(4) * t * t + 20 * coeff(5) * t * t * t;
}

/**
 * @brief Calculates the heading of the trajectory using the combination of the polynomials in the
 * longitudinal and lateral direction.
 *
 * @param trajectory The trajectory for which the heading should be calculated.
 * @param i The index at which the heading should be calculated.
 * @return float The heading at the given index.
 */
float PolynomialGenerator::calculate_heading(const Trajectory& trajectory, const size_t i,
                                             const Vehicle& vehicle) const {
  // catch the case of velocities approaching 0
  if (std::abs(trajectory.m_sVelocity[i]) < 0.0001f &&
      std::abs(trajectory.m_dVelocity[i]) < 0.0001f) {
    // keep the last known heading if it exists
    if (i != 0) {
      return trajectory.m_heading[i - 1];
    } else {
      return vehicle.m_heading;
    }
  } else {
    float heading = std::atan2(trajectory.m_dVelocity[i], trajectory.m_sVelocity[i]);

    // avoid vehicle to turn if it drives backwards
    if (std::abs(vehicle.m_heading) < M_PI_2 && std::abs(heading) > M_PI_2) {
      if (heading > 0) {
        return heading - M_PI;
      } else {
        return heading + M_PI;
      }
    } else if (std::abs(vehicle.m_heading) > M_PI_2 && std::abs(heading) < M_PI_2) {
      if (heading > 0) {
        return heading - M_PI;
      } else {
        return heading + M_PI;
      }
    } else {
      return heading;
    }
  }
}

/**
 * @brief Calculates the curvature of the trajectory using the combination of the polynomials in the
 * longitudinal and lateral direction.
 *
 * @param trajectory The trajectory for which the curvature should be calculated.
 * @param i The index at which the curvature should be calculated.
 * @return float The curvature at the given index.
 */
float PolynomialGenerator::calculate_curvature(const Trajectory& trajectory, const size_t i) const {
  // https://en.wikipedia.org/wiki/Curvature
  // catch the case of velocities approaching 0
  if (std::abs(trajectory.m_dVelocity[i]) < 0.0001f &&
      std::abs(trajectory.m_sVelocity[i]) < 0.0001f) {
    return 0;
  } else {
    return (trajectory.m_dAcceleration[i] * trajectory.m_sVelocity[i] -
            trajectory.m_dVelocity[i] * trajectory.m_sAcceleration[i]) /
           std::pow(trajectory.m_dVelocity[i] * trajectory.m_dVelocity[i] +
                        trajectory.m_sVelocity[i] * trajectory.m_sVelocity[i],
                    1.5f);
  }
}

/**
 * @brief Calculates the steering angle of the trajectory using the combination of the polynomials
 * in the longitudinal and lateral direction.
 *
 * @param trajectory The trajectory for which the steering angle should be calculated.
 * @param i The index at which the steering angle should be calculated.
 * @return float The steering angle at the given index.
 */
float PolynomialGenerator::calculate_steering_angle(const Trajectory& trajectory, const size_t i,
                                                    const Vehicle& vehicle) const {
  return std::atan2(vehicle.m_wheelBase * trajectory.m_curvature[i], 1.0f);
}

/**
 * @brief Calculates the total acceleration of the trajectory using the combination of the
 * polynomials in the longitudinal and lateral direction.
 *
 * @param trajectory The trajectory for which the total acceleration should be calculated.
 * @param i The index at which the total acceleration should be calculated.
 * @return float The total acceleration at the given index.
 */
float PolynomialGenerator::calculate_total_acceleration(const Trajectory& trajectory,
                                                        const size_t i) const {
  return math::magnitude(trajectory.m_sAcceleration[i], trajectory.m_dAcceleration[i]);
}

/**
 * @brief Calculates the total velocity of the trajectory using the combination of the polynomials
 * in the longitudinal and lateral direction.
 *
 * @param trajectory The trajectory for which the total velocity should be calculated.
 * @param i The index at which the total velocity should be calculated.
 * @return float The total velocity at the given index.
 */
float PolynomialGenerator::calculate_total_velocity(const Trajectory& trajectory,
                                                    const size_t i) const {
  return math::magnitude(trajectory.m_sVelocity[i], trajectory.m_dVelocity[i]);
}

/**
 * @brief Calculates the integral of the squared acceleration of the fifth order polynomial.
 *
 * @param start The start constraint of the fifth order polynomial.
 * @param end The end constraint of the fifth order polynomial.
 * @param t_0 The start time to calculate the integral.
 * @param t_1 The end time to calculate the integral.
 * @return float The integral of the squared acceleration.
 */
float PolynomialGenerator::squared_acceleration_integral(const BoundaryCondition& start,
                                                         const BoundaryCondition& end,
                                                         const float t_0, const float t_1) const {
  auto constraints = boundaryConditionsToEigenMatrix(start, end);
  auto coeff       = calculate_coefficients(constraints, t_0, t_1);

  return 400.0f / 7 * std::pow(coeff(5), 2) * (-std::pow(t_0, 7) + std::pow(t_1, 7)) +
         80 * coeff(4) * coeff(5) * (-std::pow(t_0, 6) + std::pow(t_1, 6)) +
         (0.240e3 * coeff(3) * coeff(5) + 144 * std::pow(coeff(4), 2)) *
             (-t_0 * t_0 * t_0 * t_0 * t_0 + t_1 * t_1 * t_1 * t_1 * t_1) / 5 +
         (80 * coeff(2) * coeff(5) + 144 * coeff(3) * coeff(4)) *
             (-t_0 * t_0 * t_0 * t_0 + t_1 * t_1 * t_1 * t_1) / 4 +
         (48 * coeff(2) * coeff(4) + 36 * std::pow(coeff(3), 2)) *
             (-t_0 * t_0 * t_0 + t_1 * t_1 * t_1) / 3 +
         12 * coeff(2) * coeff(3) * (-t_0 * t_0 + t_1 * t_1) +
         4 * std::pow(coeff(2), 2) * (-t_0 + t_1);
}

/**
 * @brief Creates the boundary conditions for the trajectory generation.
 *
 * @param action The action to be executed.
 * @param trajectory The trajectory for which the boundary conditions should be created.
 * @param vehicle The current state of the vehicle.
 * @return std::tuple<BoundaryCondition, BoundaryCondition, BoundaryCondition, BoundaryCondition>
 * The boundary conditions, starting with the start conditions in longitudinal and lateral direction
 * and then the end conditions in longitudinal and lateral direction.
 */
std::tuple<BoundaryCondition, BoundaryCondition, BoundaryCondition, BoundaryCondition>
PolynomialGenerator::createBoundaryConditions(ActionPtr action, const Vehicle& vehicle) const {
  BoundaryCondition startS;
  BoundaryCondition startD;
  BoundaryCondition endS;
  BoundaryCondition endD;

  // Update boundary conditions of polynomial based on current state of vehicle
  float deltaVs{action->m_velocityChange};
  float deltaD{action->m_lateralChange};

  // Initial/End Conditions
  startS.position     = vehicle.m_positionX;
  startS.velocity     = vehicle.m_velocityX;
  startS.acceleration = vehicle.m_accelerationX;

  startD.position = vehicle.m_positionY;
  // velocity y is dependent on current velocity in x direction and orientation
  // of vehicle
  startD.velocity     = vehicle.m_velocityX * std::tan(vehicle.m_heading);
  startD.acceleration = vehicle.m_accelerationY;

  // {distance, velocity, acceleration}
  // the postion change in the longitudinal direction is the average velocity times the duration
  float deltaS{(startS.velocity + startS.velocity + deltaVs) / 2 * cOpt().action_duration};
  endS.position = startS.position + deltaS;
  endS.velocity = startS.velocity + deltaVs;
  // always equal to zero
  endS.acceleration = 0;
  endD.position     = startD.position + deltaD;
  // always equal to zero
  endD.velocity = 0;
  // always equal to zero
  endD.acceleration = 0;

  return {startS, startD, endS, endD};
}

/**
 * @brief Calculates the trajectory based on constraints for polynomial acceleration.
 *
 * @param trajectory The generated trajectory.
 * @param startS The boundary condition related to the longitudinal direction the start.
 * @param startD The boundary condition related to the lateral direction at the start.
 * @param endS The bound condition related to the longitudinal direction at the end.
 * @param endD The boundary condition related to the lateral direction at the end.
 */
Trajectory PolynomialGenerator::calculateTrajectory(const Vehicle& vehicle, const float t0,
                                                    const BoundaryCondition& startS,
                                                    const BoundaryCondition& startD,
                                                    const BoundaryCondition& endS,
                                                    const BoundaryCondition& endD) const {
  Trajectory trajectory(t0, vehicle.m_heading);
  trajectory.m_t0 = t0;
  trajectory.m_t1 = t0 + cOpt().action_duration;

  auto constraints_s = boundaryConditionsToEigenMatrix(startS, endS);
  auto coeff_s       = calculate_coefficients(constraints_s, trajectory.m_t0, trajectory.m_t1);

  auto constraints_d = boundaryConditionsToEigenMatrix(startD, endD);
  auto coeff_d       = calculate_coefficients(constraints_d, trajectory.m_t0, trajectory.m_t1);

  for (size_t i = 0; i < trajectory.m_nSteps; ++i) {
    const auto t = t0 + i * cOpt().delta_t;

    trajectory.m_time[i]              = t;
    trajectory.m_sPosition[i]         = calculate_position(coeff_s, t);
    trajectory.m_dPosition[i]         = calculate_position(coeff_d, t);
    trajectory.m_sVelocity[i]         = calculate_velocity(coeff_s, t);
    trajectory.m_dVelocity[i]         = calculate_velocity(coeff_d, t);
    trajectory.m_sAcceleration[i]     = calculate_acceleration(coeff_s, t);
    trajectory.m_dAcceleration[i]     = calculate_acceleration(coeff_d, t);
    trajectory.m_heading[i]           = calculate_heading(trajectory, i, vehicle);
    trajectory.m_curvature[i]         = calculate_curvature(trajectory, i);
    trajectory.m_steeringAngle[i]     = calculate_steering_angle(trajectory, i, vehicle);
    trajectory.m_totalVelocity[i]     = calculate_total_velocity(trajectory, i);
    trajectory.m_totalAcceleration[i] = calculate_total_acceleration(trajectory, i);
  }
  trajectory.m_invalidAction = !trajectory.isValidAction(vehicle);
  trajectory.m_invalidState  = !trajectory.isValidState(vehicle);
  return trajectory;
}

/**
 * @brief Updates the final state of the trajectory for updating the vehicle's state.
 *
 * @param trajectory The trajectory for the update.
 */
void PolynomialGenerator::updateFinalState(Trajectory& trajectory) const {
  size_t i                   = trajectory.getFractionIndex();
  trajectory.m_finalState[0] = trajectory.m_sPosition[i];
  trajectory.m_finalState[1] = trajectory.m_dPosition[i];
  trajectory.m_finalState[2] = trajectory.m_sVelocity[i];
  trajectory.m_finalState[3] = trajectory.m_dVelocity[i];
  trajectory.m_finalState[4] = trajectory.m_sAcceleration[i];
  trajectory.m_finalState[5] = trajectory.m_dAcceleration[i];
  trajectory.m_finalState[6] = trajectory.m_lane[i];
  trajectory.m_finalState[7] = trajectory.m_heading[i];
}

/**
 * @brief Calculates the costs of acceleration based on the boundary conditions.
 *
 * @param trajectory The trajectory for  which the costs should be calculated.
 * @param startS The boundary condition related to the longitudinal direction the start.
 * @param startD The boundary condition related to the lateral direction at the start.
 * @param endS The bound condition related to the longitudinal direction at the end.
 * @param endD The boundary condition related to the lateral direction at the end.
 */
void PolynomialGenerator::calculateCumulativeAcceleration(Trajectory& trajectory,
                                                          const BoundaryCondition& startS,
                                                          const BoundaryCondition& startD,
                                                          const BoundaryCondition& endS,
                                                          const BoundaryCondition& endD) const {
  // don't modify the "global" end constraint! the trajectory is calculated for a
  // length of DeltaT
  const auto t1 = Trajectory::getCurrentFraction() * trajectory.m_t1;

  trajectory.m_cumSquaredAccelerationLon =
      squared_acceleration_integral(startS, endS, trajectory.m_t0, t1);
  trajectory.m_cumSquaredAccelerationLat =
      squared_acceleration_integral(startD, endD, trajectory.m_t0, t1);
}
}  // namespace proseco_planning
