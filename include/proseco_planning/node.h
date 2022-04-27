/**
 * @file node.h
 * @brief This file defines the Node class.
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class CollisionChecker;
class TrajectoryGenerator;

namespace config {
struct Agent;
}  // namespace config
/**
 * @brief The Node class defines a node in the search tree.
 *
 */
class Node {
 public:
  // default constructor
  Node();

  explicit Node(std::vector<Agent> agents);

  explicit Node(const std::vector<config::Agent>& agents);

  Node(const ActionSet& actionSet, Node* const parent);

  explicit Node(const Node* node);

  bool hasChildren() const;

  Node* addChild(const ActionSet& actionSet);

  Node* getChild(const ActionSet& actionSet) const;

  void checkCollision(CollisionChecker& collisionChecker);

  std::tuple<bool, bool> validateInitialization();

  bool checkInitForCollisions();

  void checkValidity();

  /**
   * @brief Checks if all agents have been initialized with valid positions.
   *
   * @return true if all agent starting pisitions are valid.
   */
  inline bool checkValidInit() const {
    return std::all_of(m_agents.begin(), m_agents.end(),
                       [](const Agent& agent) { return agent.m_vehicle.isValid(); });
  }

  void checkTerminality();

  void checkSafeRangeCost();

  void executeActions(const ActionSet& actionSet, CollisionChecker& collisionChecker,
                      const TrajectoryGenerator& trajectoryGenerator, const bool executeFraction);

  // json exports
  json childMapToJSON(const ActionSet& bestActionSet) const;
  json permutationMapToJSON(const ActionSet& bestActionSet) const;
  json moveGroupsToJSON() const;
  static void treeToJSON(const Node* const node, json& jTree);

  json treeNodeToJSON() const;

  // save data to disk
  void exportChildMap(const int step, const ActionSet& bestActionSet) const;
  void exportPermutationMap(const int step, const ActionSet& bestActionSet) const;
  void exportMoveGroups(const int step) const;
  void exportTree(const int step) const;

  // The action set that led to the node.
  ActionSet m_actionSet;
  // parent
  Node* m_parent;
  // agents
  std::vector<Agent> m_agents;
  // number of visits
  unsigned int m_visits;
  // depth of the node
  unsigned int m_depth;
  // children map, action set as the index
  std::map<ActionSet, std::unique_ptr<Node>> m_childMap;

  // represents a collision state resulting from agents executing actions that cause a collision
  // between at least two agents
  bool m_collision{false};
  // represents an invalid state resulting from an agent executing an invalid action
  bool m_invalid{false};
  // represents a terminal state resulting from all agents reaching their desired states
  bool m_terminal{false};

  // Calculate probability for invalid/collided node
  static std::tuple<float, float, float> calculateActionStatistics(
      const std::map<ActionSet, std::unique_ptr<Node>>& childMap, const ActionPtr& action,
      const int agentIdx);
};

void to_json(json& j, const Node& node);
}  // namespace proseco_planning