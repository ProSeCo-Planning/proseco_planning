#include "proseco_planning/node.h"

#include <algorithm>
#include <cstddef>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

#include "nlohmann/json.hpp"
#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/action/actionSpace.h"
#include "proseco_planning/action/actionSpaceRectangle.h"
#include "proseco_planning/agent/cost_model/costModel.h"
#include "proseco_planning/collision_checker/collisionChecker.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/outputOptions.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/trajectory/trajectory.h"
#include "proseco_planning/util/json.h"
#include "proseco_planning/util/utilities.h"

namespace proseco_planning {
class TrajectoryGenerator;

Node::Node() : m_parent(nullptr) {}

/**
 * @brief Root node constructor.
 *
 * @param agents Vector of agents.
 */
Node::Node(std::vector<Agent> agents)
    : m_parent(nullptr), m_agents(agents), m_visits(0), m_depth(0) {}

/**
 * @brief Root node constructor.
 *
 * @param agents Vector of agent configurations.
 */
Node::Node(const std::vector<config::Agent>& agents) : m_parent(nullptr), m_visits(0), m_depth(0) {
  for (const auto& agent : agents) {
    m_agents.emplace_back(agent);
  }
}

/**
 * @brief Child node constructor.
 * In general, a new child node should be created and added to the search tree with the method
 * `Node::addChild`.
 *
 * @param actionSet actionSet that leads from the parent node to this child node.
 * @param parent Pointer to the parent node.
 */
// child node constructor
Node::Node(const ActionSet& actionSet, Node* const parent)

    : m_actionSet(actionSet),
      m_parent(parent),
      m_agents(parent->m_agents),
      m_visits(0),
      m_depth(parent->m_depth + 1) {}

/**
 * @brief Copy constructor that is used for the creation of a `simulationNode`.
 *
 * @param node Pointer to the node that shall be copied.
 */
// copy constructor for simulationNode
Node::Node(const Node* node)
    : m_actionSet(node->m_actionSet),
      m_parent(node->m_parent),
      m_agents(node->m_agents),
      m_visits(node->m_visits),
      m_depth(node->m_depth),
      m_collision(node->m_collision),
      m_invalid(node->m_invalid),
      m_terminal(node->m_terminal) {}

/**
 * @brief Checks whether the node has children.
 *
 * @return true If it has children.
 * @return false If it does not have children.
 */
bool Node::hasChildren() const { return !m_childMap.empty(); }

/**
 * @brief Checks whether a collision with agents or obstacles is occurring and sets the member
 * `m_collision` accordingly.
 *
 * @param collisionChecker The collision checker used for the collision check.
 */
void Node::checkCollision(CollisionChecker& collisionChecker) {
  for (auto& agent_i : m_agents) {
    // Check for collision with other agents
    for (auto& agent_j : m_agents) {
      // compare two agents, use "<" to avoid duplicate comparison
      if (agent_i.m_id < agent_j.m_id) {
        // avoid collision check for two predefined agents
        if (agent_i.m_isPredefined && agent_j.m_isPredefined) {
          continue;
        }

        if (collisionChecker.collision(agent_i.m_vehicle, agent_i.m_trajectory, agent_j.m_vehicle,
                                       agent_j.m_trajectory)) {
          agent_i.m_collision = true;
          agent_j.m_collision = true;
          // if any agent is in collision the state/node is in collision
          m_collision = true;
        }
      }
    }

    // Check for collision with obstacles
    if (collisionChecker.collision(agent_i.m_vehicle, agent_i.m_trajectory, sOpt().obstacles)) {
      agent_i.m_collision = true;
      m_collision         = true;
    }
  }
}

std::tuple<bool, bool> Node::validateInitialization() {
  return {checkValidInit(), checkInitForCollisions()};
}

/**
 * @brief Checks if initial agent positions produce collisions.
 *
 * @return true if initial agent positions are collision free.
 * @return false if initial agent positions are not collision free.
 */
bool Node::checkInitForCollisions() {
  auto collisionChecker = CollisionChecker::createCollisionChecker(cOpt().collision_checker, 0.0f);

  for (const auto& agent_i : m_agents) {
    // Check for collision with other agents
    for (const auto& agent_j : m_agents) {
      // compare two agents, use "<" to avoid duplicate comparison
      if (agent_i.m_id < agent_j.m_id) {
        // avoid collision check for two predefined agents
        if (agent_i.m_isPredefined && agent_j.m_isPredefined) {
          continue;
        }

        if (collisionChecker->collision(agent_i.m_vehicle, agent_j.m_vehicle)) return false;
      }
    }

    // Check for collision with static obstacles
    if (collisionChecker->collision(agent_i.m_vehicle, agent_i.m_trajectory, sOpt().obstacles)) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Creates and adds a new child node.
 * @todo change for working with node action space
 * agents have to check for their valid action space
 * aggregated action space has to be updated
 *
 * @param actionSet actionSet that leads from this node to the new child node
 * @return Pointer to the new child node.
 */
Node* Node::addChild(const ActionSet& actionSet) {
  auto child = std::make_unique<Node>(actionSet, this);

  // update the executable actions of all agents
  for (auto& agent : child->m_agents) {
    agent.setAvailableActions(m_depth);

    // reset the action value
    agent.m_actionValue = 0;
  }

  // update childmap
  auto childPtr = child.get();
  m_childMap.insert(std::make_pair(actionSet, std::move(child)));

  return childPtr;
}

/**
 * @brief Returns the child node that is reached by the given action set.
 *
 * @param actionSet The action set that leads to the child node.
 * @return Node* The pointer to the child node.
 */
Node* Node::getChild(const ActionSet& actionSet) const { return m_childMap.at(actionSet).get(); }

/**
 * @brief execute actions for all agents. This node is adjusted so that it represents the state
 * after the execution.
 *
 * @param actionSet actionSet that should be executed.
 * @param collisionChecker The collision checker.
 * @param trajectoryGenerator The trajectory generator.
 * @param executeFraction Flag for executing the complete action or only a fraction.
 */
void Node::executeActions(const ActionSet& actionSet, CollisionChecker& collisionChecker,
                          const TrajectoryGenerator& trajectoryGenerator,
                          const bool executeFraction) {
  // Set the trajectory flag for executing the complete action or only a fraction
  Trajectory::useActionFraction = executeFraction;

  // generate the action set of all agents and set the state variables but don't yet simulate
  for (size_t i = 0; i < std::min(m_agents.size(), actionSet.size()); ++i) {
    m_agents[i].setAction(actionSet[i], trajectoryGenerator);
  }

  // check the state for collisions and update the state as well as the agents
  // state
  checkCollision(collisionChecker);
  // check the state for validity and update the state as well as the agents
  // state
  checkValidity();

  // calculate the new ego cost
  for (auto& agent : m_agents) {
    // simulate the agents actions and update the costs
    agent.simulate();
  }

  // check the state for terminality and update the state as well as the agents
  // state
  /// @todo rename
  checkTerminality();

  // calculate the cooperative cost - should be integrated into the cost model
  for (auto& agent_i : m_agents) {
    agent_i.m_coopReward = 0;
    agent_i.m_coopReward += agent_i.m_egoReward;
    for (auto& agent_j : m_agents) {
      if (agent_i.m_id != agent_j.m_id) {
        // calculate the cooperative cost
        agent_i.m_coopReward += agent_i.m_costModel->calculateCooperativeCost(
            agent_j.m_desire, agent_j.m_vehicle, agent_j.m_trajectory, agent_j.m_collision,
            agent_j.m_invalid, m_agents.size(), agent_j.m_egoReward, agent_i.m_cooperationFactor);
      }
    }
  }
}

/**
 * @brief Checks for validity of the state and action and sets the member `m_invalid` accordingly.
 *
 */
// checks whether a state is valid
void Node::checkValidity() {
  for (auto& agent : m_agents) {
    agent.m_invalid = !agent.m_trajectory.isValidAction(agent.m_vehicle) ||
                      !agent.m_trajectory.isValidState(agent.m_vehicle);
    m_invalid = agent.m_invalid ? true : m_invalid;
  }
}

/**
 * @brief Checks whether two vehicles get too close and update the costs accordingly.
 * @todo this can possibly be removed
 */
void Node::checkSafeRangeCost() {
  float safeRangeCost{0.0f};
  for (auto& agent_i : m_agents) {
    for (auto& agent_j : m_agents) {
      if (agent_i.m_id < agent_j.m_id) {
        if (agent_i.m_vehicle.m_lane == agent_j.m_vehicle.m_lane) {
          agent_i.m_costModel->costSafeRange(agent_i.m_vehicle, agent_j.m_vehicle);
          agent_i.m_safeRangeCost += safeRangeCost;
          agent_j.m_safeRangeCost += safeRangeCost;
        }
      }
    }
  }
}

/**
 * @brief Checks for terminality of the state and sets the member `m_terminal` accordingly. E.g.,
 * checks whether the goal has been reached.
 */
void Node::checkTerminality() {
  if (std::all_of(m_agents.begin(), m_agents.end(),
                  [](const Agent& agent) { return agent.desiresFulfilled(); })) {
    m_terminal = !m_invalid && !m_collision;
  } else {
    m_terminal = false;
  }
}

/**
 * @brief Calculates statistics for collisions, invalid actions and action count.
 *
 * @param childMap The child map for the current action.
 * @param action
 * @param agentIdx
 * @return std::tuple<float, float, float>
 */
std::tuple<float, float, float> Node::calculateActionStatistics(
    const std::map<ActionSet, std::unique_ptr<Node>>& childMap, const ActionPtr& action,
    const int agentIdx) {
  float actionCount{0.0f};
  float invalidCount{0.0f};
  float collisionCount{0.0f};

  for (const auto& [actionSet, childPtr] : childMap) {
    // if the action at the agentIdx is equal to the action
    if (actionSet.at(agentIdx) == action) {
      // count number of times action has been executed
      ++actionCount;
      if (childPtr->m_invalid) {
        // count number of collided states
        ++invalidCount;
      }
      if (childPtr->m_collision) {
        // count number of invalid states
        ++collisionCount;
      }
    }
  }
  return std::make_tuple(collisionCount / actionCount, invalidCount / actionCount, actionCount);
}

//#######################################################################
// Exports
//#######################################################################

// --------------------------------------------------------------------------------------------------
// Members for JSON export
json Node::childMapToJSON(const ActionSet& bestActionSet) const {
  json jChildMap;
  jChildMap["num_agents"] = m_agents.size();
  jChildMap["node_depth"] = m_depth;
  jChildMap["agents"]     = json::array();

  for (size_t agentIdx{}; agentIdx < m_agents.size(); ++agentIdx) {
    const auto& agent = m_agents[agentIdx];
    if (!agent.is_ego) continue;
    json jAgentInfo;
    jAgentInfo["id"]      = agent.m_id;
    jAgentInfo["actions"] = json::array();
    for (const auto& [action, value] : agent.m_actionValues) {
      // calculate the probability of collision, invalid and the number of combinations for this
      // action
      const auto [collisionProbability, invalidProbability, actionCount] =
          calculateActionStatistics(m_childMap, action, agentIdx);

      json jActionInfo;
      // determine if this action is the finally chosen one
      jActionInfo["action_chosen"] = (bestActionSet[agentIdx] == action);
      jActionInfo["action_class"]  = ActionSpace::ACTION_CLASS_NAME_MAP.at(action->m_actionClass);
      jActionInfo["d_velocity"]    = action->m_velocityChange;
      jActionInfo["d_lateral"]     = action->m_lateralChange;

      // extract action information
      jActionInfo["action_value"]  = value;
      jActionInfo["action_uct"]    = agent.m_actionUCT.at(action);
      jActionInfo["action_visits"] = agent.m_actionVisits.at(action);

      // extract action class information
      jActionInfo["class_count"]  = agent.m_actionClassCount.at(action->m_actionClass);
      jActionInfo["class_value"]  = agent.m_actionClassValues.at(action->m_actionClass);
      jActionInfo["class_uct"]    = agent.m_actionClassUCT.at(action->m_actionClass);
      jActionInfo["class_visits"] = agent.m_actionClassVisits.at(action->m_actionClass);

      // extract information about action validity
      jActionInfo["collision_prob"] = collisionProbability;
      jActionInfo["invalid_prob"]   = invalidProbability;
      jActionInfo["is_invalid"]     = action->m_invalidAction;

      jActionInfo["actions_combined"] = actionCount;
      jAgentInfo["actions"].push_back(jActionInfo);
    }
    jChildMap["agents"].push_back(jAgentInfo);
  }
  return jChildMap;
}

/**
 * @brief Returns the permutation map as a .json file
 *
 * @param bestActionSet The best action set.
 * @return json The permutation as .json file.
 */
json Node::permutationMapToJSON(const ActionSet& bestActionSet) const {
  json jPermutationMap;
  jPermutationMap["num_agents"] = m_agents.size();
  jPermutationMap["node_depth"] = m_depth;
  jPermutationMap["agents"]     = json::array();
  // generate placeholders for all agents
  for (size_t agentID{}; agentID < m_agents.size(); ++agentID) {
    if (!m_agents[agentID].is_ego) continue;
    jPermutationMap["agents"].push_back(
        {{"id", m_agents[agentID].m_id}, {"actions", json::array()}});
  }

  for (const auto& [actionPtrVec, nodePtr] : m_childMap) {
    for (size_t agent_j{}; agent_j < m_agents.size(); ++agent_j) {
      // get all nodes which contain the chosen action of agent_j
      if (bestActionSet[agent_j] == actionPtrVec[agent_j]) {
        for (size_t agent_i{}; agent_i < m_agents.size(); ++agent_i) {
          if (!m_agents[agent_i].is_ego) continue;
          json jActionInfo;
          jActionInfo["action_chosen"] = (agent_i == agent_j);
          jActionInfo["action_class"] =
              ActionSpace::ACTION_CLASS_NAME_MAP.at(nodePtr->m_actionSet[agent_i]->m_actionClass);
          jActionInfo["d_velocity"]   = nodePtr->m_actionSet[agent_i]->m_velocityChange;
          jActionInfo["d_lateral"]    = nodePtr->m_actionSet[agent_i]->m_lateralChange;
          jActionInfo["state_visits"] = nodePtr->m_visits;
          jActionInfo["node_ptr"]     = (long long)nodePtr.get();
          // add action data to the action array of agent i
          jPermutationMap["agents"][agent_i]["actions"].push_back(jActionInfo);
        }
      }
    }
  }
  return jPermutationMap;
}

/**
 * @brief Returns the move groups as a .json file.
 *
 * @return json The moveGroups as .json file.
 */
json Node::moveGroupsToJSON() const {
  json jMoveGroups;
  jMoveGroups["num_agents"] = m_agents.size();
  jMoveGroups["node_depth"] = m_depth;
  jMoveGroups["agents"]     = json::array();

  for (const auto& agent : m_agents) {
    if (!agent.is_ego) continue;
    json jAgentInfo;
    jAgentInfo["id"]             = agent.m_id;
    jAgentInfo["action_classes"] = json::array();
    for (const auto& [actionClass, value] : agent.m_actionClassUCT) {
      json jActionClassInfo;
      jActionClassInfo["id"] = ActionSpace::ACTION_CLASS_NAME_MAP.at(actionClass);

      // action boundaries for this action class are exported if the agent's action space is an
      // instance of `ActionSpaceRectangle`
      auto p_actionSpaceRectangle = dynamic_cast<ActionSpaceRectangle*>(agent.m_actionSpace.get());
      if (p_actionSpaceRectangle != nullptr) {
        // the action space is an instance of `ActionSpaceRectangle`, so export
        auto boundary =
            p_actionSpaceRectangle->getActionClassBoundary(actionClass, agent.m_vehicle);
        jActionClassInfo["velocity_change_min"] = boundary.velocityChange.min;
        jActionClassInfo["velocity_change_max"] = boundary.velocityChange.max;
        jActionClassInfo["lateral_change_min"]  = boundary.lateralChange.min;
        jActionClassInfo["lateral_change_max"]  = boundary.lateralChange.max;
      }

      jActionClassInfo["value"]       = agent.m_actionClassValues.at(actionClass);
      jActionClassInfo["uct"]         = agent.m_actionClassUCT.at(actionClass);
      jActionClassInfo["visit_count"] = agent.m_actionClassVisits.at(actionClass);
      jActionClassInfo["class_count"] = agent.m_actionClassCount.at(actionClass);

      jAgentInfo["action_classes"].push_back(jActionClassInfo);
    }
    jMoveGroups["agents"].push_back(jAgentInfo);
  }
  return jMoveGroups;
}

/**
 * @brief Saves the child map at the specified step to the disk.
 *
 * @param step The specified step.
 * @param bestActionSet The best action set.
 */
// --------------------------------------------------------------------------------------------------
// Members for MsgPack/JSON serialization
void Node::exportChildMap(const int step, const ActionSet& bestActionSet) const {
  std::string fileName{oOpt().output_path + "/" + "root_node_" + std::to_string(step)};

  switch (oOpt().export_format) {
    case config::exportFormat::MSGPACK: {
      json jChildMap = childMapToJSON(bestActionSet);
      util::saveAsMsgPack(fileName, jChildMap);
      break;
    }
    case config::exportFormat::JSON: {
      json jChildMap = childMapToJSON(bestActionSet);
      util::saveAsJSON(fileName, jChildMap);
      break;
    }
    case config::exportFormat::NONE: {
      break;
    }
  }
}

/**
 * @brief Saves the permutation map at the specified step to the disk.
 *
 * @param step The specified step.
 * @param bestActionSet The best action set.
 */
void Node::exportPermutationMap(const int step, const ActionSet& bestActionSet) const {
  // write binary file to disk
  std::string fileName{oOpt().output_path + "/" + "bestActionPermutation" + std::to_string(step)};

  switch (oOpt().export_format) {
    case config::exportFormat::MSGPACK: {
      json jPermutationMap = permutationMapToJSON(bestActionSet);
      util::saveAsMsgPack(fileName, jPermutationMap);
      break;
    }
    case config::exportFormat::JSON: {
      json jPermutationMap = permutationMapToJSON(bestActionSet);
      util::saveAsJSON(fileName, jPermutationMap);
      break;
    }
    case config::exportFormat::NONE: {
      break;
    }
  }
}

/**
 * @brief Saves the move groups (moveGroups) at the specified step to the disk.
 *
 * @param step The specified step.
 */
void Node::exportMoveGroups(const int step) const {
  std::string fileName{oOpt().output_path + "/" + "move_groups_" + std::to_string(step)};

  switch (oOpt().export_format) {
    case config::exportFormat::MSGPACK: {
      json jMoveGroups = moveGroupsToJSON();
      util::saveAsMsgPack(fileName, jMoveGroups);
      break;
    }
    case config::exportFormat::JSON: {
      json jMoveGroups = moveGroupsToJSON();
      util::saveAsJSON(fileName, jMoveGroups);
      break;
    }
    case config::exportFormat::NONE: {
      break;
    }
  }
}

/**
 * @brief Collects all relevant information for a node of the tree.
 *
 * @return json The node as JSON.
 */
json Node::treeNodeToJSON() const {
  std::string nameString = util::actionSetToString(m_actionSet) + "v" +
                           util::toStringPrecision(m_agents[0].m_actionValue, 1) + ",n" +
                           std::to_string(m_visits);
  json jNode;
  if (m_terminal) {
    jNode["name"] = nameString + "T";
  } else if (m_collision) {
    jNode["name"] = nameString + "C";
  } else if (m_invalid) {
    jNode["name"] = nameString + "I";
  } else {
    jNode["name"] = nameString;
  }
  jNode["visits"]         = m_visits;
  jNode["numberChildren"] = m_childMap.size();
  return jNode;
};

/**
 * @brief Generates a tree starting with the specified node.
 *
 * @param node The node to start with the tree building.
 * @param jTtree The resulting JSON tree.
 */
void Node::treeToJSON(const Node* const node, json& jTtree) {
  json jNode = node->treeNodeToJSON();
  if (node->hasChildren()) {
    jNode["children"] = json::array();
    for (auto& [actionSet, child] : node->m_childMap) {
      treeToJSON(child.get(), jNode["children"]);
    }
  }
  if (jTtree.is_array()) {
    jTtree.push_back(jNode);
  } else {
    jTtree = jNode;
  }
}

/**
 * @brief Exports a tree starting from the current node.
 *
 * @param step The current step of the scenario.
 */
void Node::exportTree(const int step) const {
  std::string fileName{oOpt().output_path + "/" + "search_tree_" + std::to_string(step)};
  json jTree;

  switch (oOpt().export_format) {
    case config::exportFormat::MSGPACK: {
      treeToJSON(this, jTree);
      // currently the tree visualization only reads .json
      // util::saveMsgPack(fileName, jTree);
      util::saveJSON(fileName, jTree);
      break;
    }
    case config::exportFormat::JSON: {
      treeToJSON(this, jTree);
      util::saveJSON(fileName, jTree);
      break;
    }
    case config::exportFormat::NONE: {
      break;
    }
  }
}
/**
 * @brief Function to allow conversion of an Node to a JSON object.
 * @details Gets called by the json constructor of the nlohmann json library.
 *
 * @param j The JSON object to be filled.
 * @param node The Node to be converted.
 */
void to_json(json& j, const Node& node) {
  j["address"]    = util::pointerToString(&node);
  j["collision"]  = node.m_collision;
  j["invalid"]    = node.m_invalid;
  j["terminal"]   = node.m_terminal;
  j["parent"]     = util::pointerToString(node.m_parent);
  j["action_set"] = node.m_actionSet;
  j["visits"]     = node.m_visits;
  j["depth"]      = node.m_depth;
  j["childMap"]   = node.m_childMap;
  j["agents"]     = node.m_agents;
}
}  // namespace proseco_planning
