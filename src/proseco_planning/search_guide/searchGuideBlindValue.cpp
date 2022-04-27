#include "proseco_planning/search_guide/searchGuideBlindValue.h"

#include <sys/types.h>
#include <algorithm>
#include <memory>
#include <numeric>
#include <utility>

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionSpace.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/search_guide/searchGuide.h"

namespace proseco_planning {
class Vehicle;

/**
 * @brief Constructs a new Search Guide Blind Value object.
 *
 * @param type The type of the search guide.
 */
SearchGuideBlindValue::SearchGuideBlindValue(const std::string& type) : SearchGuide(type) {}

/**
 * @brief Returns the action with the maximum "blind value" for progressive widening.
 * @details If the argument for parameter `actionClass` is `ActionClass::NONE`, an action whithin
 * the action space is created and returned; otherwise, an action within the specified action class.
 *
 * @param actionClass The action class within to sample.
 * @param actionSpace The action space for the agent.
 * @param vehicle The current state of the vehicle.
 * @param actionUCT The UCT values for all available actions.
 * @return ActionPtr To the best action for progressive widening.
 */
ActionPtr SearchGuideBlindValue::getBestActionForPW(
    const ActionClass& actionClass, const ActionSpace& actionSpace, const Vehicle& vehicle,
    const std::map<ActionPtr, float>& actionUCT) const {
  // create random actions to evaluate the Blind Value
  auto randomActions{sampleRandomActions(actionClass, actionSpace, vehicle)};

  // Calculate the adaption coefficient based on the already explored actions
  // and the newly drawn actions
  auto adaptionCoefficient{calculateAdaptionCoefficient(actionUCT, randomActions)};

  // Calculate Blind Values for each new action
  for (auto& [action, blindValue] : randomActions) {
    blindValue = calculateBlindValue(adaptionCoefficient, action, actionUCT);
  }

  // Select action by blind value: action with max Blind Value
  return math::max_map_element(randomActions);
}

/**
 * @brief Sample random actions in the actions space.
 *
 * @param actionClass The action class within to sample. If `ActionClass::NONE` is specified, sample
 * in the entire action space.
 * @param actionSpace The action space to sample from.
 * @param vehicle The vehicle for which to sample.
 * @return std::map<ActionPtr, float> The random actions.
 */
std::map<ActionPtr, float> SearchGuideBlindValue::sampleRandomActions(
    const ActionClass& actionClass, const ActionSpace& actionSpace, const Vehicle& vehicle) {
  std::map<ActionPtr, float> randomActions;
  for (unsigned int i = 0; i < cOpt().policy_options.policy_enhancements.search_guide.n_samples;
       ++i) {
    if (actionClass == ActionClass::NONE) {
      randomActions.insert(std::make_pair(actionSpace.sampleRandomAction(vehicle), 0.0f));
    } else {
      randomActions.insert(
          std::make_pair(actionSpace.sampleRandomActionInActionClass(actionClass, vehicle), 0.0f));
    }
  }
  return randomActions;
}

/**
 * @brief Calculates the blind value for the new action.
 *
 * @param adaptionCoefficient The adaption coefficient related to the explored actions.
 * @param newAction The new action.
 * @param exploredActions The explored actions.
 * @return float The calculated blind value.
 */
float SearchGuideBlindValue::calculateBlindValue(
    const float adaptionCoefficient, const ActionPtr& newAction,
    const std::map<ActionPtr, float>& exploredActions) {
  std::vector<float> blindValues;
  blindValues.reserve(exploredActions.size());

  for (const auto& [action, uctValue] : exploredActions) {
    blindValues.push_back(uctValue + adaptionCoefficient * newAction->getDistance(action));
  }

  return *std::min_element(blindValues.begin(), blindValues.end());
}

/**
 * @brief Calculates the adaption coefficient based on the already explored actions
 * and the newly drawn actions.
 *
 * @param actionUCT The explored actions.
 * @param randomActions The randomly sampled actions.
 * @return float The adaption coefficient.
 */
float SearchGuideBlindValue::calculateAdaptionCoefficient(
    const std::map<ActionPtr, float>& actionUCT, const std::map<ActionPtr, float>& randomActions) {
  // Standard deviation of the UCT values of already explored actions
  auto uctValues = math::mapValuesToVector(actionUCT);

  // Standard deviation of the distance to the origin of random actions
  std::vector<float> distancesToOrigin;
  distancesToOrigin.reserve(randomActions.size());
  for (const auto& [action, _] : randomActions) {
    distancesToOrigin.push_back(action->getDistance());
  }

  return math::stdFromVector(uctValues) / math::stdFromVector(distancesToOrigin);
}
}  // namespace proseco_planning
