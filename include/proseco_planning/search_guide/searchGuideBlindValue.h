/**
 * @file searchGuideBlindValue.h
 * @brief This files defines the SearchGuideBlindValue class.
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <cassert>
#include <map>
#include <string>
#include <vector>

#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/util/alias.h"
#include "searchGuide.h"

namespace proseco_planning {
class ActionSpace;
class Vehicle;

/**
 * @brief The SearchGuideBlindValue class determines the best action for progressive widening
 * based on "blind values", which are calculated with kernel regression.
 */
class SearchGuideBlindValue : public SearchGuide {
 public:
  explicit SearchGuideBlindValue(const std::string& type);

  /**
   * @brief Gets the best action for progressive widening.
   *
   * @param actionSpace The action space of the agent.
   * @param vehicle The vehicle.
   * @param actionUCT The UCT values for all available actions.
   * @return ActionPtr The best action for progressive widening.
   */
  inline ActionPtr getBestActionForPW(const ActionSpace& actionSpace, const Vehicle& vehicle,
                                      const std::map<ActionPtr, float>& actionUCT) const override {
    // `ActionClass::NONE` as the argument for `actionClass` means that the entire action space is
    // considered
    return getBestActionForPW(ActionClass::NONE, actionSpace, vehicle, actionUCT);
  };

  /**
   * @brief Gets the best action for progressive widening within an action class.
   *
   * @param actionClass The action class.
   * @param actionSpace The action space of the agent.
   * @param vehicle The vehicle.
   * @param actionUCT The UCT values for all available actions.
   * @return ActionPtr The best action for progressive widening.
   */
  inline ActionPtr getBestActionInActionClassForPW(
      const ActionClass& actionClass, const ActionSpace& actionSpace, const Vehicle& vehicle,
      const std::map<ActionPtr, float>& actionUCT) const override {
    assert((actionClass != ActionClass::NONE) && "actionClass is ActionClass::NONE");
    // `actionClass` is forwarded so that only the this action class is considered
    return getBestActionForPW(actionClass, actionSpace, vehicle, actionUCT);
  };

 protected:
  ActionPtr getBestActionForPW(const ActionClass& actionClass, const ActionSpace& actionSpace,
                               const Vehicle& vehicle,
                               const std::map<ActionPtr, float>& actionUCT) const;

  static float calculateBlindValue(const float adaptionCoefficient, const ActionPtr& newAction,
                                   const std::map<ActionPtr, float>& exploredActions);

  static float calculateAdaptionCoefficient(const std::map<ActionPtr, float>& actionUCT,
                                            const std::map<ActionPtr, float>& newActions);

  static std::map<ActionPtr, float> sampleRandomActions(const ActionClass& actionClass,
                                                        const ActionSpace& actionSpace,
                                                        const Vehicle& vehicle);
};
}  // namespace proseco_planning