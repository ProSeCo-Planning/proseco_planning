#include "proseco_planning/trajectory/trajectorygenerator.h"

#include <cstddef>
#include <iostream>
#include <memory>
#include <tuple>

#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/trajectory/constantacceleration.h"
#include "proseco_planning/trajectory/polynomialgenerator.h"
#include "proseco_planning/trajectory/trajectory.h"

namespace proseco_planning {
class Vehicle;

/**
 * @brief Constructs a new Trajectory Generator object.
 *
 * @param name The method for trajectory generation.
 */
TrajectoryGenerator::TrajectoryGenerator(const std::string& name) : m_name(name) {}

/**
 * @brief Creates the trajectory generator (factory method).
 *
 * @param name The method for trajectory generation.
 * @return std::unique_ptr<TrajectoryGenerator>  The trajectory generator.
 */
std::unique_ptr<TrajectoryGenerator> TrajectoryGenerator::createTrajectoryGenerator(
    const std::string& name) {
  if (name == "jerkOptimal") {
    return std::make_unique<PolynomialGenerator>(name);
  } else if (name == "constantAcceleration") {
    return std::make_unique<ConstantAcceleration>(name);
  } else {
    throw std::invalid_argument("Unknown trajectory generation type: " + name);
  }
}

/**
 * @brief Updates the boundary conditions and the final state of the trajectory based on the chosen
 * action.
 * @details This method also calculates and evaluates the trajectory.
 *
 * @param t0 The start time of the trajectory.
 * @param trajectory The trajectory for which the final state should be updated.
 * @param action The chosen action.
 * @param vehicle The current state of the vehicle.
 */

Trajectory TrajectoryGenerator::createTrajectory(float t0, ActionPtr action,
                                                 const Vehicle& vehicle) const {
  // Calculate boundary conditions with action = [dLat, dV] + currentState
  const auto [startS, startD, endS, endD] = createBoundaryConditions(action, vehicle);

  // Calculates the discrete representation of the trajectory
  auto trajectory = calculateTrajectory(vehicle, t0, startS, startD, endS, endD);
  // Calculates the cost associated with the action
  evaluateTrajectory(trajectory, startS, startD, endS, endD);

  // Compact representation for updating the vehicle state without integration of the kinematic
  // model
  updateFinalState(trajectory);
  return trajectory;
}

/**
 * @brief Calculates the costs of acceleration based on the boundary conditions.
 *
 * @param trajectory The trajectory to be evaluated.
 * @param startS The boundary condition related to the longitudinal direction the start.
 * @param startD The boundary condition related to the lateral direction at the start.
 * @param endS The bound condition related to the longitudinal direction at the end.
 * @param endD The boundary condition related to the lateral direction at the end.
 */
void TrajectoryGenerator::evaluateTrajectory(Trajectory& trajectory,
                                             const BoundaryCondition& startS,
                                             const BoundaryCondition& startD,
                                             const BoundaryCondition& endS,
                                             const BoundaryCondition& endD) const {
  calculateCumulativeAcceleration(trajectory, startS, startD, endS, endD);

  TrajectoryGenerator::detectLaneChange(trajectory);
  trajectory.calculateAverageSpeed();
  trajectory.calculateAverageAbsoluteAcceleration();
}

/**
 * @brief Checks for lane change in trajectory and updates the information in the trajectory
 * accordingly.
 *
 * @param trajectory The trajectory to check for lane change.
 */
void TrajectoryGenerator::detectLaneChange(Trajectory& trajectory) const {
  trajectory.determineLane();
  trajectory.determineLaneChange();
}

}  // namespace proseco_planning
