/**
 * @file simulationSingleThread.h
 * @brief This file defines the SimulationSingleThread class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <string>
#include <vector>

#include "proseco_planning/policies/simulationPolicy.h"

namespace proseco_planning {
class Node;

/**
 * @brief The SimulationSingleThread class is a singlethreaded simulation policy.
 */
class SimulationSingleThread : public SimulationPolicy {
 public:
  explicit SimulationSingleThread(const std::string& name);

  unsigned int runSimulation(Node* const node, std::vector<std::vector<float> >& agentsRewards,
                             unsigned int maxDepth) override;
};
}  // namespace proseco_planning
