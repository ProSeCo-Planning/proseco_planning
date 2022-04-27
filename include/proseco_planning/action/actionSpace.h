/**
 * @file actionSpace.h
 * @brief This file defines the ActionSpace class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Action;
class Vehicle;
class TrajectoryGenerator;
class CollisionChecker;

/**
 * @brief An action space defines the space of all possible actions that an agent can take.
 * Furthermore, it specifies the action classes and it is responsible for action sampling.
 */
class ActionSpace {
 public:
  explicit ActionSpace(const std::string& type);

  static std::shared_ptr<ActionSpace> createActionSpace(
      const config::ActionSpace::variant_t& variant);

  /// The virtual destructor.
  virtual ~ActionSpace() = default;

  ActionPtr sampleModerateAction(const Vehicle& vehicle) const;

  static ActionPtr sampleValidAction(const Vehicle& vehicle,
                                     const std::function<ActionPtr()>& samplingFunction,
                                     CollisionChecker& collisionChecker,
                                     const TrajectoryGenerator& trajectoryGenerator);

  virtual ActionSet getPredefinedActions() const = 0;

  virtual ActionSet getModerateActions(const Vehicle& vehicle) const = 0;

  virtual ActionSet getDetailedActions(const Vehicle& vehicle) const = 0;

  virtual ActionClass getActionClass(const Action& action, const Vehicle& vehicle) const = 0;

  virtual ActionPtr sampleRandomAction(const Vehicle& vehicle) const = 0;

  virtual ActionPtr sampleRandomActionInActionClass(const ActionClass& actionClass,
                                                    const Vehicle& vehicle) const = 0;

  /// The map that assigns a name string to each action class.
  static const std::map<ActionClass, std::string> ACTION_CLASS_NAME_MAP;

  /// The action space type.
  const std::string m_type;
};

}  // namespace proseco_planning