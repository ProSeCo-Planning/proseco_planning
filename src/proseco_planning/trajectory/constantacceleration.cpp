#include "proseco_planning/trajectory/constantacceleration.h"

#include <cstddef>
#include <memory>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/trajectory/trajectory.h"

namespace proseco_planning {

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
ConstantAcceleration::createBoundaryConditions(ActionPtr action, const Vehicle& vehicle) const {
  // Convert Action: delta Vs, delta d to acceleration
  BoundaryCondition startS;
  BoundaryCondition startD;
  BoundaryCondition endS;
  BoundaryCondition endD;

  // Initial
  // Initial position
  startS.position = vehicle.m_positionX;
  startD.position = vehicle.m_positionY;
  // initial velocity
  startS.velocity = vehicle.m_velocityX;
  startD.velocity = vehicle.m_velocityY;
  // constant acceleration
  // avoid driving backwards
  float veloChangeX = action->m_velocityChange;
  if ((vehicle.m_velocityX + veloChangeX) * vehicle.m_velocityX < 0.0f) {
    if (vehicle.m_velocityX > 0) {
      veloChangeX = -(vehicle.m_velocityX - 0.01f);
    } else {
      veloChangeX = -(vehicle.m_velocityX + 0.01f);
    }
  }
  startS.acceleration = veloChangeX / cOpt().action_duration;
  // given values: delta d, d0, vd0 + assumption constant acceleration =>
  // acceleration longitudinal
  startD.acceleration = 2 / cOpt().action_duration / cOpt().action_duration *
                        (action->m_lateralChange - vehicle.m_velocityY * cOpt().action_duration);

  return {startS, startD, endS, endD};
}

/**
 * @brief Calculates the position using constant acceleration for a given time.
 *
 * @param time The time for which the position should be calculated.
 * @param position The position at the start of the time interval.
 * @param velocity The velocity at the start of the time interval.
 * @param acceleration The acceleration during the time interval.
 * @return float The position at the given time.
 */
float ConstantAcceleration::position(float time, float position, float velocity,
                                     float acceleration) const {
  return 0.5 * time * time * acceleration + time * velocity + position;
};

/**
 * @brief Calculates the velocity using constant acceleration for a given time.
 *
 * @param time The time for which the velocity should be calculated.
 * @param velocity The velocity at the start of the time interval.
 * @param acceleration The acceleration during the time interval.
 * @return float The velocity at the given time.
 */
float ConstantAcceleration::velocity(float time, float velocity, float acceleration) const {
  return time * acceleration + velocity;
};

/**
 * @brief Calculates the trajectory based on constraints for constant acceleration.

 * @param trajectory The generated trajectory.
 * @param startS The boundary condition related to the longitudinal direction the start.
 * @param startD The boundary condition related to the lateral direction at the start.
 * @param endS The bound condition related to the longitudinal direction at the end.
 * @param endD The boundary condition related to the lateral direction at the end.
 *
 */
Trajectory ConstantAcceleration::calculateTrajectory(const Vehicle& vehicle, const float t0,
                                                     const BoundaryCondition& startS,
                                                     const BoundaryCondition& startD,
                                                     const BoundaryCondition& endS,
                                                     const BoundaryCondition& endD) const {
  Trajectory trajectory(t0, vehicle.m_heading);
  trajectory.m_t0 = t0;
  trajectory.m_t1 = t0 + cOpt().action_duration;

  for (size_t i = 0; i < trajectory.m_nSteps; ++i) {
    const auto t = t0 + i * cOpt().delta_t;

    trajectory.m_time[i]      = t;
    trajectory.m_sPosition[i] = position(t, startS.position, startS.velocity, startS.acceleration);
    trajectory.m_dPosition[i] = position(t, startD.position, startD.velocity, startD.acceleration);
    trajectory.m_sVelocity[i] = velocity(t, startS.velocity, startS.acceleration);
    trajectory.m_dVelocity[i] = velocity(t, startD.velocity, startD.acceleration);
    trajectory.m_sAcceleration[i] = startS.acceleration;
    trajectory.m_dAcceleration[i] = startD.acceleration;
    // heading - not influenced since already set within initialization
  }
  return trajectory;
}

/**
 * @brief Updates the final state of the trajectory for updating the vehicle's state.
 *
 * @param trajectory The trajectory for the update.
 */
void ConstantAcceleration::updateFinalState(Trajectory& trajectory) const {
  const auto i               = trajectory.getFractionIndex();
  trajectory.m_finalState[0] = trajectory.m_sPosition[i];
  trajectory.m_finalState[1] = trajectory.m_dPosition[i];
  trajectory.m_finalState[2] = trajectory.m_sVelocity[i];
  trajectory.m_finalState[3] = 0.0f;
  trajectory.m_finalState[4] = 0.0f;
  trajectory.m_finalState[5] = 0.0f;
  trajectory.m_finalState[6] = trajectory.m_lane[i];
  trajectory.m_finalState[7] = 0.0f;
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
void ConstantAcceleration::calculateCumulativeAcceleration(Trajectory& trajectory,
                                                           const BoundaryCondition& startS,
                                                           const BoundaryCondition& startD,
                                                           const BoundaryCondition& endS,
                                                           const BoundaryCondition& endD) const {
  const auto duration = Trajectory::getCurrentFraction() * trajectory.m_t1;
  // Squared value of the calculated constant acceleration
  trajectory.m_cumSquaredAccelerationLon = startS.acceleration * startS.acceleration * duration;
  trajectory.m_cumSquaredAccelerationLat = startD.acceleration * startD.acceleration * duration;
}
}  // namespace proseco_planning
