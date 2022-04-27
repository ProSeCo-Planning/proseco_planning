/**
 * @file agent.h
 * @brief This file defines the Agent class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <map>
#include <memory>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/agent/desire.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/trajectory/trajectory.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class ActionSpace;
class CostModel;
class SearchGuide;
class TrajectoryGenerator;
namespace config {
struct Agent;
}  // namespace config

/**
 * @brief Agent class
 */
class Agent {
 public:
  explicit Agent(const config::Agent& agent);

  void addAvailableAction(ActionPtr action);

  void setAvailableActions(unsigned int depth);

  void clearActionMaps();

  void addActionToMaps(const ActionPtr& action);

  void addActionsToMaps(const ActionSet& actions);

  void updateActionClasses();

  void setAction(ActionPtr action, const TrajectoryGenerator& trajectoryGenerator);

  void simulate();

  void calculateCosts(const Vehicle& vehiclePreviousStep, const float beforePotential);

  void costCollision();

  void costInvalidState();

  void rewardTerminal();

  bool desiresFulfilled() const;

  float cumulativeActionVisits() const;

  float cumulativeActionClassVisits() const;

  json trajectoryStepToJSON(const size_t index) const;

  ActionPtr maxActionVisitsAction() const;

  ActionClass maxActionVisitsActionClass() const;

  float maxActionValue() const;

  float minActionValue() const;

  float maxActionClassActionValue() const;

  float minActionClassActionValue() const;

  ActionPtr maxActionValueAction() const;

  ActionClass maxActionValueActionClass() const;

  float maxActionUCT() const;

  float minActionUCT() const;

  ActionPtr maxActionUCTAction() const;

  ActionClass maxActionUCTActionClass() const;

  /// The action space.
  std::shared_ptr<ActionSpace> m_actionSpace;

  /// The visits of each action.
  std::map<ActionPtr, float> m_actionVisits;

  /// The value of each action.
  std::map<ActionPtr, float> m_actionValues;

  /// The UCT value of each action.
  std::map<ActionPtr, float> m_actionUCT;

  /// The total visits of each action class.
  std::map<ActionClass, float> m_actionClassVisits;

  /// The average action value within each action class.
  std::map<ActionClass, float> m_actionClassValues;

  /// The average UCT score within each action class.
  std::map<ActionClass, float> m_actionClassUCT;

  /// The number of actions within each action class.
  std::map<ActionClass, int> m_actionClassCount;

  /// The desired state of agent
  Desire m_desire;

  /// The vehicle the agent is controlling
  Vehicle m_vehicle;

  /// The cooperation factor of the agent
  const float m_cooperationFactor;

  /// The cost of ego vehicle so far
  float m_egoReward{0.0f};

  /// The cooperative cost for the agent
  float m_coopReward{0.0f};

  /// The action cost: acc, lane change, accumulative, for the calculation of egoReward
  float m_actionCost{0.0f};

  /// The cost for entering the safe range
  float m_safeRangeCost{0.0f};

  /// The action value: expected return of this action, which is actually value for actionSet
  float m_actionValue{0.0f};

  /// The object for translating Action object to trajectory
  Trajectory m_trajectory;

  /// The model for guiding the search
  std::shared_ptr<SearchGuide> m_searchGuide{nullptr};

  /// The id of the agent
  unsigned int m_id;

  /// Used for determining whether data of this agent should be included in export files
  bool is_ego{true};

  /// The available actions - for execution
  ActionSet m_availableActions;

  /// Represents the agent executing an action leading to a collision
  bool m_collision{false};

  /// Represents the agent executing an invalid action
  bool m_invalid{false};

  /// If tagged no Progressive Widening should occur
  bool m_isPredefined{false};

  /// The cost model of the agent
  std::shared_ptr<CostModel> m_costModel;

  /// The potential of final state, for the calculation of state reward
  float m_finalPotential{0.0f};

  /// The potential of current state
  float m_currentPotential{0.0f};

  /// The state cost: potential based reward shaping
  float m_stateReward{0.0f};
};
void to_json(json& j, const Agent& agent);
}  // namespace proseco_planning
