/**
 * @file searchGuideRandom.h
 * @brief This file defines the SearchGuideRandom class.
 *
 * @copyright Copyright (c) 2021
 */

#pragma once

#include <map>

#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/util/alias.h"
#include "searchGuide.h"

namespace proseco_planning {
class ActionSpace;
class Vehicle;

/**
 * @brief The SearchGuideRandom class determines the best action for progressive widening using
 * random sampling.
 */
class SearchGuideRandom : public SearchGuide {
 public:
  using SearchGuide::SearchGuide;

  ActionPtr getBestActionForPW(const ActionSpace& actionSpace, const Vehicle& vehicle,
                               const std::map<ActionPtr, float>& actionUCT) const override;

  ActionPtr getBestActionInActionClassForPW(
      const ActionClass& actionClass, const ActionSpace& actionSpace, const Vehicle& vehicle,
      const std::map<ActionPtr, float>& actionUCT) const override;
};
}  // namespace proseco_planning
