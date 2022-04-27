/**
 * @file searchGuide.h
 * @brief This file defines the SearchGuide class, the base class for all search guides.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <map>
#include <memory>
#include <string>

#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class ActionSpace;
class Vehicle;

/**
 * @brief The SearchGuide class defines the base class for all search guides. The search guide
 * guides the search to a new actions that shall be appended to the available actions due to
 * progressive widening.
 * @details Currently implemented: Random and Blind Values.
 */
class SearchGuide {
 public:
  explicit SearchGuide(const std::string& type);

  /// The virtual destructor.
  virtual ~SearchGuide() = default;

  static std::shared_ptr<SearchGuide> createSearchGuide(const std::string& type);

  /**
   * @brief Gets the best action whithin an action space that shall be appended to the available
   * actions due to progressive widening.

   * @param actionSpace The action space.
   * @param vehicle The current state of the vehicle.
   * @param actionUCT UCT values for all available actions.
   * @return ActionPtr To the best action for progressive widening.
   *
   */
  virtual ActionPtr getBestActionForPW(const ActionSpace& actionSpace, const Vehicle& vehicle,
                                       const std::map<ActionPtr, float>& actionUCT) const = 0;

  /**
   * @brief Get the best action whithin an specified action class that shall be appended to the
   * available actions due to progressive widening.

   * @param actionClass The action class.
   * @param actionSpace The action space.
   * @param vehicle The current state of the vehicle.
   * @param actionUCT UCT values for all available actions.
   * @return ActionPtr To the best action for progressive widening.
   *
   */
  virtual ActionPtr getBestActionInActionClassForPW(
      const ActionClass& actionClass, const ActionSpace& actionSpace, const Vehicle& vehicle,
      const std::map<ActionPtr, float>& actionUCT) const = 0;

  /// The type of search guide.
  std::string m_type;
};
}  // namespace proseco_planning