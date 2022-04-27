#include "proseco_planning/action/actionSpace.h"

#include <cstddef>
#include <random>
#include <stdexcept>
#include <variant>

#include "proseco_planning/action/actionSpaceRectangle.h"
#include "proseco_planning/collision_checker/collisionChecker.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/trajectory/trajectory.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"

namespace proseco_planning {
class Vehicle;

const std::map<ActionClass, std::string> ActionSpace::ACTION_CLASS_NAME_MAP{
    {ActionClass::DO_NOTHING, "0"},         {ActionClass::ACCELERATE, "+"},
    {ActionClass::DECELERATE, "-"},         {ActionClass::CHANGE_LEFT, "L"},
    {ActionClass::CHANGE_RIGHT, "R"},       {ActionClass::CHANGE_LEFT_FAST, "L+"},
    {ActionClass::CHANGE_LEFT_SLOW, "L-"},  {ActionClass::CHANGE_RIGHT_FAST, "R+"},
    {ActionClass::CHANGE_RIGHT_SLOW, "R-"},
};

/**
 * @brief Constructs a new ActionSpace object.
 *
 * @param type Action space type.
 */
ActionSpace::ActionSpace(const std::string& type) : m_type(type) {}

/**
 * @brief Factory method that creates an action space according to the specified action space
 * variant type.
 *
 * @param variant The action space variant type.
 * @return std::shared_ptr<ActionSpace>m The pointer to the action space.
 */
std::shared_ptr<ActionSpace> ActionSpace::createActionSpace(
    const config::ActionSpace::variant_t& variant) {
  if (std::holds_alternative<config::ActionSpaceRectangle>(variant)) {
    const auto& config = std::get<config::ActionSpaceRectangle>(variant);
    return std::make_shared<ActionSpaceRectangle>(config);
  } else {
    throw std::invalid_argument("Unknown action space variant");
  }
}

/**
 * @brief Sample an action from the moderate actions, i.e., from the sparse representation of this
 * action space.
 *
 * @param vehicle Current state of the vehicle.
 * @return ActionPtr
 */
ActionPtr ActionSpace::sampleModerateAction(const Vehicle& vehicle) const {
  auto moderateActions = getModerateActions(vehicle);
  return math::getRandomElementFromVector(moderateActions);
}

ActionPtr ActionSpace::sampleValidAction(const Vehicle& vehicle,
                                         const std::function<ActionPtr()>& samplingFunction,
                                         CollisionChecker& collisionChecker,
                                         const TrajectoryGenerator& trajectoryGenerator) {
  auto action                   = samplingFunction();
  auto trajectory               = trajectoryGenerator.createTrajectory(0.0, action, vehicle);
  Trajectory::useActionFraction = false;
  uint sample{0};
  // ensure that the action and state are valid, and that no collision occurs with obstacles
  while ((!trajectory.isValidAction(vehicle) || !trajectory.isValidState(vehicle) ||
          collisionChecker.collision(vehicle, trajectory, sOpt().obstacles)) &&
         sample < cOpt().max_invalid_action_samples) {
    action     = samplingFunction();
    trajectory = trajectoryGenerator.createTrajectory(0.0, action, vehicle);
    sample++;
  }
  return action;
};

}  // namespace proseco_planning
