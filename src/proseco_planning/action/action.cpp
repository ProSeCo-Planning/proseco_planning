#include "proseco_planning/action/action.h"

#include <map>
#include <string>

#include "nlohmann/json.hpp"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/action/actionSpace.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"

namespace proseco_planning {

/**
 * @brief Constructs a new Action object using an action class.
 * @details Delegates to standard constructor with default values (default constructor).
 * @param actionClass The action class of the action.
 */
Action::Action(ActionClass actionClass)
    : m_actionClass(actionClass),
      m_velocityChange(0),
      m_lateralChange(0),
      m_accelerationX(0),
      m_accelerationY(0) {}

/**
 * @brief Constructs a new Action object using accelerations.
 * @details Converts the inputs to the corresponding velocity in longitudinal direction and position
 * change in lateral direction assuming constant acceleration and zero initial velocity the lateral
 * direction.
 * @param actionClass The action class of the action.
 * @param accelerationX Acceleration in longitudinal direction.
 * @param accelerationY Acceleration in lateral direction.
 */
Action::Action(ActionClass actionClass, float accelerationX, float accelerationY)
    : m_actionClass(actionClass),
      m_velocityChange(accelerationX * cOpt().action_duration),
      m_lateralChange(accelerationY * 0.5f * cOpt().action_duration * cOpt().action_duration),
      m_accelerationX(accelerationX),
      m_accelerationY(accelerationY) {}

/**
 * @brief Constructs a new Action object using the longitudinal velocity change and lateral position
 * change.
 * @details Converts the inputs to the corresponding acceleration in longitudinal and lateral
 * direction assuming constant acceleration and zero initial velocity the lateral direction.
 * @param velocityChange Change in velocity.
 * @param lateralChange Change in lateral position.
 */
Action::Action(float velocityChange, float lateralChange)
    : m_actionClass(ActionClass::NONE),
      m_velocityChange(velocityChange),
      m_lateralChange(lateralChange),
      m_accelerationX(m_velocityChange / cOpt().action_duration),
      m_accelerationY((2 / (cOpt().action_duration * cOpt().action_duration)) * m_lateralChange) {}

/**
 * @brief Updates `m_actionClass` according to the action space and the current state of the
 * vehicle.
 *
 * @param actionSpace Action space.
 * @param vehicle Current state of the vehicle.
 */
void Action::updateActionClass(const ActionSpace& actionSpace, const Vehicle& vehicle) {
  m_actionClass = actionSpace.getActionClass(*this, vehicle);
}

/**
 * @brief Calculates the similarity of action x and action y using the radial basis
 * function kernel.
 * @param x The first action.
 * @param y The second action.
 * @param gamma The gamma for the similarity function.
 * @return float The similarity of the two actions.
 */
float Action::getSimilarity(const ActionPtr& x, const ActionPtr& y, const float gamma) {
  return std::exp(-gamma * (x->getSquaredDistance(y)));
}

/**
 * @brief Calculates the similarity of action x and action y using the radial basis
 * function kernel and retrieving gamma from the configuration.
 * @note Overload
 * @param x The first action.
 * @param y The second action.
 * @return float The similarity of the two actions.
 */
float Action::getSimilarity(const ActionPtr& x, const ActionPtr& y) {
  return getSimilarity(x, y, cOpt().parallelization_options.similarity_gamma);
}

/**
 * @brief Calculates the squared euclidean distance to the action, if none is given it defaults to
 * the origin of the action space.
 * @details The origin is assumed to be at [0,0]
 *
 * @param action The action to calculate the distance to.
 * @return float The squared euclidean distance of the two actions.
 */
float Action::getSquaredDistance(const ActionPtr& action) const {
  if (action == nullptr) {
    return m_lateralChange * m_lateralChange + m_velocityChange * m_velocityChange;
  } else {
    auto distLateral  = m_lateralChange - action->m_lateralChange;
    auto distVelocity = m_velocityChange - action->m_velocityChange;
    return distLateral * distLateral + distVelocity * distVelocity;
  }
}

/**
 * @brief Calculates the euclidean distance of to the action, if none is given it defaults to the
 * origin of the action space.
 * @details The origin is assumed to be at [0,0]
 *
 * @param action The action to calculate the distance to.
 * @return float The euclidean distance between the the two actions.
 */
float Action::getDistance(const ActionPtr& action) const {
  return std::sqrt(getSquaredDistance(action));
}

/**
 * @brief Function to allow conversion of an Action to a JSON object.
 * @details Gets called by the json constructor of the nlohmann json library.
 *
 * @param j The JSON object to be filled.
 * @param action The Action to be converted.
 */
void to_json(json& j, const Action& action) {
  j["class"]           = ActionSpace::ACTION_CLASS_NAME_MAP.at(action.m_actionClass);
  j["acceleration_x"]  = action.m_accelerationX;
  j["acceleration_y"]  = action.m_accelerationY;
  j["velocity_change"] = action.m_velocityChange;
  j["lateral_change"]  = action.m_lateralChange;
}
}  // namespace proseco_planning
