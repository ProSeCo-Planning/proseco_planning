/**
 * @file simulationPolicy.h
 * @brief This file defines the SimulationPolicy class, the base class for all simulation policies.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "policy.h"
#include "proseco_planning/collision_checker/collisionChecker.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"

namespace proseco_planning {
class Node;

/*!
 * \brief The SimulationPolicy class is a base class for all simulation policies, i.e. single
 * threaded and multi threaded, as well as random and moderate.
 */
class SimulationPolicy : public Policy {
 public:
  /**
   * @brief Construct a new Simulation Policy object.
   *
   * @param name The name of the policy.
   */
  explicit SimulationPolicy(const std::string& name) : Policy(name) {}

  /// The virtual destructor.
  virtual ~SimulationPolicy() = default;

  static std::unique_ptr<SimulationPolicy> createPolicy(const std::string& name, int agentsSize);

  /**
   * @brief run a simulation from a selected node
   *
   * @param node pointer to the node that the simulation is started from
   * @param agentsRewards vector that contains for every step along the tree path another vector
   * that stores the reward for each agent; update this parameter by reference
   * @param maxDepth maximum depth of the simulation
   * @return depth of the simulation node
   */
  virtual unsigned int runSimulation(Node* const node,
                                     std::vector<std::vector<float> >& agentsRewards,
                                     unsigned int maxDepth) = 0;

  void setSimulationActionSet(Node* const simulationNode) const;

  unsigned int simulate(Node* const simulationNode, unsigned int maxDepth,
                        std::vector<std::vector<float> >& agentsRewards);

 protected:
  /// The collision checker.
  std::unique_ptr<CollisionChecker> m_collisionChecker;

  /// The trajectory generator.
  std::unique_ptr<TrajectoryGenerator> m_trajectoryGenerator;
};
}  // namespace proseco_planning