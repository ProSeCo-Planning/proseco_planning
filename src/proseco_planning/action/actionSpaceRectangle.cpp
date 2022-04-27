#include "proseco_planning/action/actionSpaceRectangle.h"

#include <memory>
#include <random>
#include <stdexcept>

#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/math/mathlib.h"

namespace proseco_planning {

/**
 * @brief Constructs a new ActionSpaceRectangle object.
 *
 * @param config The ActionSpaceRectangle configuration.
 */
ActionSpaceRectangle::ActionSpaceRectangle(const config::ActionSpaceRectangle& config)
    : ActionSpace(config::ActionSpaceRectangle::TYPE), m_config(config) {
  math::MinMaxPair<float> velocityChange{-m_config.max_velocity_change,
                                         m_config.max_velocity_change};
  math::MinMaxPair<float> lateralchange{-m_config.max_lateral_change, m_config.max_lateral_change};
  m_boundary = ActionBoundary(velocityChange, lateralchange);

  m_collisionChecker    = CollisionChecker::createCollisionChecker(cOpt().collision_checker);
  m_trajectoryGenerator = TrajectoryGenerator::createTrajectoryGenerator(cOpt().trajectory_type);
}

/**
 * @brief Gets the action(s) for a predefined agent.
 * @details Predefined implies that the agent is not changing its velocity or lateral position,
 * hence the action is DO_NOTHING.
 * @note The first action of the action set is generally explored first.
 *
 * @return ActionSet The predefined action(s).
 */
ActionSet ActionSpaceRectangle::getPredefinedActions() const {
  return {std::make_shared<Action>(ActionClass::DO_NOTHING)};
}

/**
 * @brief Gets a detailed action set, i.e., a total of nine actions, spaced out over the action
 * space.
 *
 * @note The first action of the action set is generally explored first.
 *
 * @param vehicle The vehicle.
 * @return ActionSet The detailed actions.
 */
ActionSet ActionSpaceRectangle::getDetailedActions(const Vehicle& vehicle) const {
  const auto distanceToLeftLaneCenter{vehicle.getDistanceToLeftLaneCenter()};
  const auto distanceToRightLaneCenter{vehicle.getDistanceToRightLaneCenter()};

  ActionSet availableActions;
  availableActions.reserve(9);

  // first set the sparse action space representation then extend
  availableActions = getModerateActions(vehicle);
  // change right and decelerate
  availableActions.push_back(
      std::make_shared<Action>(-m_config.max_velocity_change / 2.0f, distanceToRightLaneCenter));
  // change right and accelerate
  availableActions.push_back(
      std::make_shared<Action>(m_config.max_velocity_change / 2.0f, distanceToRightLaneCenter));
  // change left and decelerate
  availableActions.push_back(
      std::make_shared<Action>(m_config.max_velocity_change / 2.0f, distanceToLeftLaneCenter));
  // change left and accelerate
  availableActions.push_back(
      std::make_shared<Action>(-m_config.max_velocity_change / 2.0f, distanceToLeftLaneCenter));
  return availableActions;
}

/**
 * @brief Gets a moderate action set, i.e., a total of five actions, spaced out over the action
 * space that are used for the search below the max depth for progressive widening.
 *
 * @note The first action of the returned vector is generally explored first.
 *
 * @param vehicle The vehicle.
 * @return ActionSet The moderate actions.
 */
ActionSet ActionSpaceRectangle::getModerateActions(const Vehicle& vehicle) const {
  ActionSet availableActions;
  availableActions.reserve(5);

  // initialize moderate actions - only using the basic action classes
  // do nothing
  availableActions.push_back(std::make_shared<Action>(0.0f, 0.0f));
  // accelerate
  availableActions.push_back(std::make_shared<Action>(m_config.max_velocity_change / 2.0f, 0.0f));
  // decelerate
  availableActions.push_back(std::make_shared<Action>(-m_config.max_velocity_change / 2.0f, 0.0f));
  // change left
  availableActions.push_back(std::make_shared<Action>(0.0f, vehicle.getDistanceToLeftLaneCenter()));
  // change right
  availableActions.push_back(
      std::make_shared<Action>(0.0f, vehicle.getDistanceToRightLaneCenter()));
  return availableActions;
}

/**
 * @brief Gets the action class of an action given the the vehicle and its action
 * space.
 * @details The action space is divided into five areas corresponding to the five basic action
 * classes.
 * + : accelerate
 * - : decelerate
 * 0 : Keep constant velocity and lateral position
 * R : Change lateral position to the right
 * L : Change lateral position to the left
 * @note The division of the action space is not tuned and solely uses the points of the trajectory
 * and not the entire rectangle of the vehicle
 *
 * @param action The action to classify.
 * @param vehicle The vehicle.
 * @return ActionClass The action class of the action.
 */
ActionClass ActionSpaceRectangle::getActionClass(const Action& action,
                                                 const Vehicle& vehicle) const {
  const auto distanceToLeftLane  = vehicle.getDistanceToLeftLane();
  const auto distanceToRightLane = vehicle.getDistanceToRightLane();

  if (doNothingActionClass(action, distanceToLeftLane, distanceToRightLane))
    return ActionClass::DO_NOTHING;
  else if (changeLeftActionClass(action, distanceToLeftLane)) {
    if (fastActionClass(action))
      return ActionClass::CHANGE_LEFT_FAST;
    else if (slowActionClass(action))
      return ActionClass::CHANGE_LEFT_SLOW;
    else
      return ActionClass::CHANGE_LEFT;
  } else if (changeRightActionClass(action, distanceToRightLane)) {
    if (fastActionClass(action))
      return ActionClass::CHANGE_RIGHT_FAST;
    else if (slowActionClass(action))
      return ActionClass::CHANGE_RIGHT_SLOW;
    else
      return ActionClass::CHANGE_RIGHT;
  } else if (accelerateActionClass(action, distanceToLeftLane, distanceToRightLane)) {
    return ActionClass::ACCELERATE;
  } else if (decelerateActionClass(action, distanceToLeftLane, distanceToRightLane)) {
    return ActionClass::DECELERATE;
  } else {
    throw std::runtime_error("Action class could not be determined.");
  }
}

/**
 * @brief Gets the boundaries of an action class.
 *
 * @param actionClass The action class to get the boundaries of.
 * @param vehicle The vehicle.
 * @return ActionBoundary The boundaries of the action class.
 */
ActionBoundary ActionSpaceRectangle::getActionClassBoundary(const ActionClass actionClass,
                                                            const Vehicle& vehicle) const {
  const auto distanceToLeftLane  = vehicle.getDistanceToLeftLane();
  const auto distanceToRightLane = vehicle.getDistanceToRightLane();

  ActionBoundary boundary;

  switch (actionClass) {
    case ActionClass::DO_NOTHING:
      boundary.velocityChange.min = -m_config.delta_velocity;
      boundary.velocityChange.max = m_config.delta_velocity;
      boundary.lateralChange.min  = distanceToRightLane;
      boundary.lateralChange.max  = distanceToLeftLane;
      break;

    case ActionClass::CHANGE_LEFT:
      boundary.velocityChange.min = -m_config.delta_velocity;
      boundary.velocityChange.max = m_config.delta_velocity;
      boundary.lateralChange.min  = distanceToLeftLane;
      boundary.lateralChange.max  = m_config.max_lateral_change;
      break;

    case ActionClass::CHANGE_RIGHT:
      boundary.velocityChange.min = -m_config.delta_velocity;
      boundary.velocityChange.max = m_config.delta_velocity;
      boundary.lateralChange.min  = -m_config.max_lateral_change;
      boundary.lateralChange.max  = distanceToRightLane;
      break;

    case ActionClass::ACCELERATE:
      boundary.velocityChange.min = m_config.delta_velocity;
      boundary.velocityChange.max = m_config.max_velocity_change;
      boundary.lateralChange.min  = distanceToRightLane;
      boundary.lateralChange.max  = distanceToLeftLane;
      break;

    case ActionClass::DECELERATE:
      boundary.velocityChange.min = -m_config.max_velocity_change;
      boundary.velocityChange.max = -m_config.delta_velocity;
      boundary.lateralChange.min  = distanceToRightLane;
      boundary.lateralChange.max  = distanceToLeftLane;
      break;

    case ActionClass::CHANGE_LEFT_FAST:
      boundary.velocityChange.min = m_config.delta_velocity;
      boundary.velocityChange.max = m_config.max_velocity_change;
      boundary.lateralChange.min  = distanceToLeftLane;
      boundary.lateralChange.max  = m_config.max_lateral_change;
      break;

    case ActionClass::CHANGE_LEFT_SLOW:
      boundary.velocityChange.min = -m_config.max_velocity_change;
      boundary.velocityChange.max = -m_config.delta_velocity;
      boundary.lateralChange.min  = distanceToLeftLane;
      boundary.lateralChange.max  = m_config.max_lateral_change;
      break;

    case ActionClass::CHANGE_RIGHT_FAST:
      boundary.velocityChange.min = m_config.delta_velocity;
      boundary.velocityChange.max = m_config.max_velocity_change;
      boundary.lateralChange.min  = -m_config.max_lateral_change;
      boundary.lateralChange.max  = distanceToRightLane;
      break;

    case ActionClass::CHANGE_RIGHT_SLOW:
      boundary.velocityChange.min = -m_config.max_velocity_change;
      boundary.velocityChange.max = -m_config.delta_velocity;
      boundary.lateralChange.min  = -m_config.max_lateral_change;
      boundary.lateralChange.max  = distanceToRightLane;
      break;

    case ActionClass::NONE:
      throw std::invalid_argument("Action class must be initialized.");
  }

  return boundary;
}

/**
 * @brief Gets the boundaries of the entire action space.
 *
 * @return ActionBoundary The boundaries of the action space.
 */

ActionBoundary ActionSpaceRectangle::getActionSpaceBoundary() const {
  return {{-m_config.max_velocity_change, m_config.max_velocity_change},
          {-m_config.max_lateral_change, m_config.max_lateral_change}};
}

/**
 * @brief Samples a random action whithin the action boundary.
 *
 * @param boundary The action boundary to sample within.
 * @return ActionPtr The sampled action.
 */
ActionPtr ActionSpaceRectangle::sampleRandomActionInBoundary(const ActionBoundary& boundary) {
  auto velocityChange =
      math::getRandomNumberInInterval(boundary.velocityChange.min, boundary.velocityChange.max);
  auto lateralChange =
      math::getRandomNumberInInterval(boundary.lateralChange.min, boundary.lateralChange.max);
  return std::make_shared<Action>(velocityChange, lateralChange);
}

/**
 * @brief Sample a random action within the action boundary of the action space.
 *
 * @param vehicle The vehicle.
 * @return ActionPtr The sampled action.
 */
ActionPtr ActionSpaceRectangle::sampleRandomAction(const Vehicle& vehicle) const {
  return sampleValidAction(
      vehicle, std::bind(&ActionSpaceRectangle::sampleRandomActionInBoundary, m_boundary),
      *m_collisionChecker, *m_trajectoryGenerator);
}

/**
 * @brief Sample a random action whithin the specified action class.
 *
 * @param actionClass The action class.
 * @param vehicle The vehicle.
 * @return ActionPtr The sampled action.
 */
ActionPtr ActionSpaceRectangle::sampleRandomActionInActionClass(const ActionClass& actionClass,
                                                                const Vehicle& vehicle) const {
  auto classBoundary = getActionClassBoundary(actionClass, vehicle);
  return sampleValidAction(
      vehicle, std::bind(&ActionSpaceRectangle::sampleRandomActionInBoundary, classBoundary),
      *m_collisionChecker, *m_trajectoryGenerator);
};

}  // namespace proseco_planning
