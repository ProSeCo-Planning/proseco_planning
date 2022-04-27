#include "proseco_planning/search_guide/searchGuideRandom.h"

#include "proseco_planning/action/actionSpace.h"

namespace proseco_planning {
class Vehicle;

/**
 * @brief Returns a random action within the action space for progressive widening.
 *
 * @param actionSpace The action space for the agent.
 * @param vehicle The current state of the vehicle.
 * @param actionUCT The UCT values for all available actions.
 * @return ActionPtr To the best action for progressive widening.
 */
ActionPtr SearchGuideRandom::getBestActionForPW(const ActionSpace& actionSpace,
                                                const Vehicle& vehicle,
                                                const std::map<ActionPtr, float>& actionUCT) const {
  // sample whithin the entire action space
  return actionSpace.sampleRandomAction(vehicle);
}

/**
 * @brief Returns a random action within the specified action class for progressive widening.
 *
 * @param actionClass The action class within to sample.
 * @param actionSpace The action space for the agent.
 * @param vehicle The current state of the vehicle.
 * @param actionUCT The UCT values for all available actions.
 * @return ActionPtr To the best action for progressive widening.
 */
ActionPtr SearchGuideRandom::getBestActionInActionClassForPW(
    const ActionClass& actionClass, const ActionSpace& actionSpace, const Vehicle& vehicle,
    const std::map<ActionPtr, float>& actionUCT) const {
  // sample whithin the specified action class
  return actionSpace.sampleRandomActionInActionClass(actionClass, vehicle);
}

}  // namespace proseco_planning
