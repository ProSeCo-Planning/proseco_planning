#include "proseco_planning/agent/agent.h"

#include <cassert>
#include <numeric>
#include <string>
#include <type_traits>
#include <utility>

#include "nlohmann/json.hpp"
#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionSpace.h"
#include "proseco_planning/agent/cost_model/costModel.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/search_guide/searchGuide.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"
#include "proseco_planning/util/json.h"

namespace proseco_planning {
/**
 * @brief Constructs a new Agent object using an agent config.
 *
 * @param agent The agent config.
 */
Agent::Agent(const config::Agent& agent)
    : m_desire(agent.desire),
      m_vehicle(agent.vehicle),
      m_cooperationFactor(agent.cooperation_factor),
      m_egoReward(0.0f),
      m_coopReward(0.0f),
      m_actionValue(0.0f),
      m_trajectory(Trajectory(0.0f, agent.vehicle.heading)),
      m_id(agent.id),
      m_currentPotential(0) {
  m_actionSpace = ActionSpace::createActionSpace(agent.action_space);
  m_costModel   = CostModel::createCostModel(agent.cost_model);

  // Create SearchGuide
  m_searchGuide =
      SearchGuide::createSearchGuide(cOpt().policy_options.policy_enhancements.search_guide.type);

  // initialize the final potential value, this value won't be changed later because this
  // constructor is only called in the main function
  m_costModel->initializeMaximumStatePotential(m_desire, m_vehicle);
}

/**
 * @brief Clears all action maps (i.e. action and action class maps).
 */
void Agent::clearActionMaps() {
  // reset action maps
  m_actionVisits.clear();
  m_actionValues.clear();
  m_actionUCT.clear();
  // reset action class maps
  m_actionClassUCT.clear();
  m_actionClassValues.clear();
  m_actionClassVisits.clear();
  m_actionClassCount.clear();
}

/**
 * @brief Sets all available actions in current situation.
 * @note This function is executed after the action_execution.
 * @note Currently takes all actions.
 *
 * @param depth Depth of the node.
 */
void Agent::setAvailableActions(unsigned int depth) {
  // clear all action maps
  clearActionMaps();

  // predefined agents only have a very limited set of actions
  // e.g., only "do_nothing" if ActionSpaceRectangle is used
  if (m_isPredefined) {
    m_availableActions = m_actionSpace->getPredefinedActions();
  }
  // if a certain tree depth is reached the agent has only the basic maneuver
  // operations (moderate actions) left
  // e.g., +,-,0, L (to the center of the lane), R (to the center of the lane) if the
  // ActionSpaceRectangle is used
  else if (depth < cOpt().policy_options.policy_enhancements.progressive_widening.max_depth_pw) {
    // All detailed actions are available for the agent
    m_availableActions = m_actionSpace->getDetailedActions(m_vehicle);
  } else {
    // only moderate actions are available for the agent to reduce the action space exploration
    m_availableActions = m_actionSpace->getModerateActions(m_vehicle);
  }

  updateActionClasses();

  addActionsToMaps(m_availableActions);
}

/**
 * @brief Adds an action to the set of available actions.
 * @note This is used in progressive widening.
 *
 * @param action Action to add.
 */
void Agent::addAvailableAction(ActionPtr action) {
  action->updateActionClass(*m_actionSpace, m_vehicle);

  m_availableActions.push_back(action);

  // update the maps
  addActionToMaps(action);
}

/**
 * @brief Adds the action set to all action maps.
 *
 * @param actions The actions to add.
 */
void Agent::addActionsToMaps(const ActionSet& actions) {
  for (const auto& action : actions) {
    addActionToMaps(action);
  }
}

/**
 * @brief Add an action to all action maps.
 *
 * @param action The action to add.
 */
void Agent::addActionToMaps(const ActionPtr& action) {
  // add the new action to the maps using default values
  m_actionVisits.insert(std::make_pair(action, 0.0f));
  m_actionValues.insert(std::make_pair(action, 0.0f));
  m_actionUCT.insert(std::make_pair(action, config::ComputeOptions::initial_uct));

  m_actionClassVisits.insert(std::make_pair(action->m_actionClass, 0.0f));
  m_actionClassValues.insert(std::make_pair(action->m_actionClass, 0.0f));
  m_actionClassUCT.insert(
      std::make_pair(action->m_actionClass, config::ComputeOptions::initial_uct));
  auto [_, inserted] = m_actionClassCount.insert(std::make_pair(action->m_actionClass, 1));
  if (!inserted) {
    // !inserted, because the action class already exists in the map, therefore, increment the
    // action class count by one.
    ++m_actionClassCount.at(action->m_actionClass);
  }
}

/**
 * @brief Updates the action classes of the actions based on the resulting change of state of the
 * action.
 */
void Agent::updateActionClasses() {
  for (const auto& action : m_availableActions) {
    action->updateActionClass(*m_actionSpace, m_vehicle);
  }
}

/**
 * @brief Sets an action of the agents action space.
 * @param action Action used for trajectory calculation and evaluation with agent's cost model.
 * @param trajectoryGenerator Trajectory generator to create the trajectories with.
 */
void Agent::setAction(ActionPtr action, const TrajectoryGenerator& trajectoryGenerator) {
  /// 0. reset reward values
  m_actionCost    = 0.0f;
  m_stateReward   = 0.0f;
  m_egoReward     = 0.0f;
  m_coopReward    = 0.0f;
  m_safeRangeCost = 0.0f;
  // 1. calculate trajectory according to chosen action
  // t0: important for simulation or only for export

  m_trajectory = trajectoryGenerator.createTrajectory(0.0f, action, m_vehicle);

  // 2. Evaluate chosen action with agent's cost model
  m_actionCost = m_costModel->calculateActionCost(m_trajectory);
}

void Agent::calculateCosts(const Vehicle& vehiclePreviousStep, const float beforePotential) {
  if ("costExponential" == m_costModel->m_type) {
    m_stateReward = m_costModel->updateStatePotential(m_desire, m_vehicle);
    m_egoReward   = m_stateReward + m_actionCost;

    // collision/invalid cost update
    if (m_collision) {
      costCollision();
    }
    if (m_invalid) {
      costInvalidState();
    }
  } else if ("costLinear" == m_costModel->m_type) {
    m_stateReward = m_costModel->calculateStateCost(m_desire, m_vehicle, m_collision, m_invalid);
    m_egoReward   = m_stateReward + m_actionCost;
  } else if ("costNonLinear" == m_costModel->m_type ||
             "costLinearCooperative" == m_costModel->m_type) {
    m_egoReward = m_costModel->calculateCost(m_desire, m_vehicle, vehiclePreviousStep, m_collision,
                                             m_invalid, m_trajectory);
  } else {
    // update current potential
    m_currentPotential = m_costModel->updateStatePotential(m_desire, m_vehicle);

    // update the reward (= improvement of the state)
    m_stateReward = m_costModel->updateStateReward(m_currentPotential, beforePotential);

    m_egoReward = m_stateReward + m_actionCost + m_safeRangeCost;
    // collision/invalid cost update
    if (m_collision) {
      costCollision();
    }
    if (m_invalid) {
      costInvalidState();
    }
  }
}

/**
 * @brief Updates the cost for a collision.
 */
void Agent::costCollision() { m_egoReward += m_costModel->m_costCollision; }

/**
 * @brief Updates the reward for a terminal action.
 */
void Agent::rewardTerminal() { m_egoReward += m_costModel->m_rewardTerminal; }

/**
 * @brief Updates the cost for an invalid action.
 */
void Agent::costInvalidState() { m_egoReward += m_costModel->m_costInvalidState; }

/**
 * @brief Simulates the agent for the duration of one step using the trajectory generator.
 * @details Method specified in constructor of TrajectoryGenerator.
 */
void Agent::simulate() {
  // save potential of old state for updating the current state Reward
  float beforePotential{m_currentPotential};

  Vehicle vehiclePreviousStep(m_vehicle);

  // simulate the vehicle using the action and the vehicle model
  // Approach using the trajectory generator
  m_vehicle.updateState(m_trajectory.m_finalState);

  calculateCosts(vehiclePreviousStep, beforePotential);
}

/**
 * @brief Indicates whether the agent has reached its terminal state, or not.
 *
 * @return True, if all desires are fulfilled, false otherwise.
 */
bool Agent::desiresFulfilled() const { return m_desire.desiresFulfilled(m_vehicle); }

/**
 * @brief Calculates the total action visits over all actions.
 *
 * @return float Total action visits over all actions.
 */
float Agent::cumulativeActionVisits() const {
  return std::accumulate(
      m_actionVisits.begin(), m_actionVisits.end(), 0.0f,
      [](float sum, const std::pair<ActionPtr, float>& action) { return (sum + action.second); });
}

/**
 * @brief Calculates the total action visits over all action classes.
 *
 * @return float Total action visits over all action classes.
 */
float Agent::cumulativeActionClassVisits() const {
  return std::accumulate(m_actionClassVisits.begin(), m_actionClassVisits.end(), 0.0f,
                         [](float sum, const std::pair<ActionClass, float>& actionClass) {
                           return (sum + actionClass.second);
                         });
}

/**
 * @brief Returns the action with the maximum visit count of any action.
 *
 * @return ActionPtr The action with maximum visit count.
 */
ActionPtr Agent::maxActionVisitsAction() const { return math::max_map_element(m_actionVisits); }

/**
 * @brief Returns the action class with the maximum visit count of any action class.
 *
 * @return ActionClass The action class with maximum visit count.
 */
ActionClass Agent::maxActionVisitsActionClass() const {
  return math::max_map_element(m_actionClassVisits);
}

/**
 * @brief Returns the maximum action value of any action.
 *
 * @return float Maximum action value.
 */
float Agent::maxActionValue() const { return math::max_map_value(m_actionValues); }

/**
 * @brief Returns the minimum action value of any action.
 *
 * @return float Minimum action value.
 */
float Agent::minActionValue() const { return math::min_map_value(m_actionValues); }

/**
 * @brief Returns the action with the maximum action value of any action.
 *
 * @return ActionPtr Maximum action value action.
 */
ActionPtr Agent::maxActionValueAction() const { return math::max_map_element(m_actionValues); }

/**
 * @brief Returns the maximum action value of any action class.
 *
 * @return float Maximum action value.
 */
float Agent::maxActionClassActionValue() const { return math::max_map_value(m_actionClassValues); }

/**
 * @brief Returns the minimum action value of any action class.
 *
 * @return float Minimum action value.
 */
float Agent::minActionClassActionValue() const { return math::min_map_value(m_actionClassValues); }

/**
 * @brief Returns the action class with the maximum action value of any action class.
 *
 * @return ActionClass Maximum action value action class.
 */
ActionClass Agent::maxActionValueActionClass() const {
  return math::max_map_element(m_actionClassValues);
}

/**
 * @brief Returns the maximum UCT value of any action.
 *
 * @return float Maximum UCT value.
 */
float Agent::maxActionUCT() const { return math::max_map_value(m_actionUCT); }

/**
 * @brief Returns the minimum UCT value of any action.
 *
 * @return float Minimum UCT value.
 */
float Agent::minActionUCT() const { return math::min_map_value(m_actionUCT); }

/**
 * @brief Returns the action with the maximum UCT value of any action.
 *
 * @return ActionPtr Maximum UCT value action.
 */
ActionPtr Agent::maxActionUCTAction() const { return math::max_map_element(m_actionUCT); }

/**
 * @brief Returns the action class with the maximum UCT value of any action class.
 *
 * @return ActionClass Maximum UCT value action class.
 */
ActionClass Agent::maxActionUCTActionClass() const {
  assert(m_actionClassUCT.count(ActionClass::NONE) == 0 &&
         "Action classes must be initialized before usage.");
  return math::max_map_element(m_actionClassUCT);
}

/**
 * @brief Exports information for a specific trajectory step to JSON.
 * @return json The information for a specific trajectory step.
 */
json Agent::trajectoryStepToJSON(const size_t index) const {
  json jStep;
  jStep["ego_reward"]         = m_egoReward;
  jStep["coop_reward"]        = m_coopReward;
  jStep["position_x"]         = m_trajectory.m_sPosition[index];
  jStep["position_y"]         = m_trajectory.m_dPosition[index];
  jStep["velocity_x"]         = m_trajectory.m_sVelocity[index];
  jStep["velocity_y"]         = m_trajectory.m_dVelocity[index];
  jStep["acceleration_x"]     = m_trajectory.m_sAcceleration[index];
  jStep["acceleration_y"]     = m_trajectory.m_dAcceleration[index];
  jStep["total_velocity"]     = m_trajectory.m_totalVelocity[index];
  jStep["total_acceleration"] = m_trajectory.m_totalAcceleration[index];
  jStep["lane"]               = m_trajectory.m_lane[index];
  jStep["heading"]            = m_trajectory.m_heading[index];
  return jStep;
}

/**
 * @brief Function to allow conversion of an Agent to a JSON object.
 * @details Gets called by the json constructor of the nlohmann json library.
 *
 * @param j The JSON object to be filled.
 * @param agent The Agent to be converted.
 */
void to_json(json& j, const Agent& agent) {
  j["m_actionSpace"]       = config::ActionSpace::toJSON(sOpt().agents[agent.m_id].action_space);
  j["m_searchGuide"]       = cOpt().policy_options.policy_enhancements.search_guide.toJSON();
  j["m_costModel"]         = sOpt().agents[agent.m_id].cost_model.toJSON();
  j["vehicle"]             = agent.m_vehicle;
  j["m_actionVisits"]      = agent.m_actionVisits;
  j["m_actionValues"]      = agent.m_actionValues;
  j["m_actionUCT"]         = agent.m_actionUCT;
  j["m_actionClassVisits"] = agent.m_actionClassVisits;
  j["m_actionClassValues"] = agent.m_actionClassValues;
  j["m_actionClassUCT"]    = agent.m_actionClassUCT;
  j["m_actionClassCount"]  = agent.m_actionClassCount;
  j["m_desire"]            = agent.m_desire;
  j["m_cooperationFactor"] = agent.m_cooperationFactor;
  j["m_egoReward"]         = agent.m_egoReward;
  j["m_actionCost"]        = agent.m_actionCost;
  j["m_safeRangeCost"]     = agent.m_safeRangeCost;
  j["m_actionValue"]       = agent.m_actionValue;
  j["m_trajectory"]        = agent.m_trajectory;
  j["m_id"]                = agent.m_id;
  j["is_ego"]              = agent.is_ego;
  j["m_availableActions"]  = agent.m_availableActions;
  j["m_collision"]         = agent.m_collision;
  j["m_invalid"]           = agent.m_invalid;
  j["m_isPredefined"]      = agent.m_isPredefined;
  j["m_currentPotential"]  = agent.m_currentPotential;
  j["m_stateReward"]       = agent.m_stateReward;
  j["m_currentPotential"]  = agent.m_currentPotential;
}

}  // namespace proseco_planning
