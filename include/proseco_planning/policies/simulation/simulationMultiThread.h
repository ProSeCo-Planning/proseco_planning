/**
 * @file simulationMultiThread.h
 * @brief This file defines the SimulationMultiThread class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "proseco_planning/node.h"
#include "proseco_planning/policies/simulationPolicy.h"

namespace proseco_planning {
/**
 * @brief The SimulationMultiThread class is a multithreaded simulation policy.
 */
class SimulationMultiThread : public SimulationPolicy {
 public:
  SimulationMultiThread(const std::string& name, const int agentsSize);

  unsigned int runSimulation(Node* const node, std::vector<std::vector<float>>& agentsRewards,
                             unsigned int maxDepth) override;

  void resetRewardsVector();

  void getMaxRewards(const unsigned int nodeDepth, const unsigned int maxSimDepth,
                     std::vector<std::vector<float>>& agentsRewards);

  void getMeanRewards(const unsigned int nodeDepth, const unsigned int maxSimDepth,
                      std::vector<std::vector<float>>& agentsRewards);

  void calculateVariance(const unsigned int nodeDepth,
                         const std::vector<std::vector<float>>& agentsRewards);

 private:
  void simulationRewardsToCSV(const std::vector<std::vector<float>>& agentsRewards);

  /// The vector contains an agentsRewards vector for each thread.
  std::vector<std::vector<std::vector<float>>> m_multiThreadAgentsRewards;

  /// The vector contains a node to simulate from for each thread.
  std::vector<std::unique_ptr<Node>> m_simulationNodes;
};
}  // namespace proseco_planning
