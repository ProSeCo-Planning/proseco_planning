#include "proseco_planning/policies/simulation/simulationMultiThread.h"

#include <omp.h>
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <limits>

#include "proseco_planning/collision_checker/collisionChecker.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/outputOptions.h"
#include "proseco_planning/policies/policy.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"

namespace proseco_planning {

/**
 * @brief Constructs a new multi-threaded simulation object according to the specified policy.
 *
 * @param name The name that specifies the policy for the simulation.
 * @param agentsSize Integer that specifies the number of agents.
 */
SimulationMultiThread::SimulationMultiThread(const std::string& name, const int agentsSize)
    : SimulationPolicy(name) {
  m_collisionChecker    = CollisionChecker::createCollisionChecker(cOpt().collision_checker);
  m_trajectoryGenerator = TrajectoryGenerator::createTrajectoryGenerator(cOpt().trajectory_type);

  m_simulationNodes =
      std::vector<std::unique_ptr<Node>>(cOpt().parallelization_options.n_simulationThreads);

  // create and initialize m_multiThreadAgentsRewards
  m_multiThreadAgentsRewards = std::vector<std::vector<std::vector<float>>>(
      cOpt().parallelization_options.n_simulationThreads,
      std::vector<std::vector<float>>(cOpt().max_search_depth, std::vector<float>(agentsSize, 0)));
}

/**
 * @brief Resets the reward vector to 0.
 *
 */
void SimulationMultiThread::resetRewardsVector() {
  // iterate over threads
  for (size_t i{0}; i < m_multiThreadAgentsRewards.size(); ++i) {
    // iterate over depth
    for (size_t j{0}; j < m_multiThreadAgentsRewards[i].size(); ++j) {
      // set all the rewards to 0
      std::fill(m_multiThreadAgentsRewards[i][j].begin(), m_multiThreadAgentsRewards[i][j].end(),
                0);
    }
  }
}
/**
 * @brief Starts a multi-thread simulation and returns the maximum simulation depth.
 *
 * @param node The node from which the simulation should start.
 * @param agentsRewards The agent's rewards.
 * @param maxDepth The maximum simulation depth.
 * @return unsigned int The maximum simulation depth.
 */
unsigned int SimulationMultiThread::runSimulation(Node* const node,
                                                  std::vector<std::vector<float>>& agentsRewards,
                                                  unsigned int maxDepth) {
  std::vector<unsigned int> simDepths(cOpt().parallelization_options.n_simulationThreads, 0);
  auto maxSimDepth = node->m_depth;
  if (!Policy::isNodeTerminal(node, maxDepth)) {
    // Initialize multiThreadAgentsRewards with 0
    resetRewardsVector();

    omp_set_num_threads(cOpt().parallelization_options.n_simulationThreads);
#pragma omp parallel for
    for (size_t idx = 0; idx < cOpt().parallelization_options.n_simulationThreads; ++idx) {
      // Simulation's node state
      m_simulationNodes[idx] = std::make_unique<Node>(node);
      simDepths[idx] =
          simulate(m_simulationNodes[idx].get(), maxDepth, m_multiThreadAgentsRewards[idx]);
    }

    // Maximum depth reached during simulation
    maxSimDepth = *std::max_element(simDepths.begin(), simDepths.end());

    if (cOpt().parallelization_options.simulation_aggregation == "mean") {
      getMeanRewards(node->m_depth, maxSimDepth, agentsRewards);
    } else if (cOpt().parallelization_options.simulation_aggregation == "max") {
      getMaxRewards(node->m_depth, maxSimDepth, agentsRewards);
    }
  }
  return maxSimDepth;
}

/**
 * @brief Finds the highest reward per depth step over all threads
 * and saves it in agentsRewards accordingly.
 *
 * @param nodeDepth The depth of the current node.
 * @param maxSimDepth The maximum depth of the simulation.
 * @param agentsRewards The result vector that contains the maximum afterwards.
 */
void SimulationMultiThread::getMaxRewards(const unsigned int nodeDepth,
                                          const unsigned int maxSimDepth,
                                          std::vector<std::vector<float>>& agentsRewards) {
  // initialize a vector to store the sum of Rewards of all agents per depth per thread
  std::vector<float> sumRewards(cOpt().parallelization_options.n_simulationThreads,
                                std::numeric_limits<float>::lowest());
  // iterate over all threads
  for (size_t threadIdx = 0; threadIdx < m_multiThreadAgentsRewards.size(); ++threadIdx) {
    // only process threads that have reached the deepest level
    if (m_simulationNodes[threadIdx]->m_depth == maxSimDepth) {
      // iterate over all agents and save the sum of the rewards in sumRewards
      float agentSum{0};
      for (size_t agentIdx = 0; agentIdx < m_simulationNodes[threadIdx]->m_agents.size();
           ++agentIdx) {
        // += to sum over all threads
        agentSum += m_multiThreadAgentsRewards[threadIdx][maxSimDepth - 1][agentIdx];
      }
      sumRewards[threadIdx] = agentSum;
    }
  }

  // get the thread_index with the maximum reward at the deepest simulation level
  auto threadIdx = static_cast<unsigned int>(
      std::distance(sumRewards.begin(), std::max_element(sumRewards.begin(), sumRewards.end())));

  // iterate over the simulation of the chosen thread and save the rewards in agentsRewards
  for (size_t depthIdx{nodeDepth + 1}; depthIdx <= m_simulationNodes[threadIdx]->m_depth;
       ++depthIdx) {
    for (size_t agentIdx = 0; agentIdx < m_simulationNodes[threadIdx]->m_agents.size();
         ++agentIdx) {
      agentsRewards[depthIdx - 1][agentIdx] =
          m_multiThreadAgentsRewards[threadIdx][depthIdx - 1][agentIdx];
    }
  }
}

/**
 * @brief Calculates the mean reward per agent over all simulations.
 * @param nodeDepth The depth of the current node.
 * @param maxSimDepth The maximum depth of the simulation.
 * @param agentsRewards The result vector that contains the maximum afterwards.
 */
void SimulationMultiThread::getMeanRewards(const unsigned int nodeDepth,
                                           const unsigned int maxSimDepth,
                                           std::vector<std::vector<float>>& agentsRewards) {
  // a vector to store how deep each simulation went
  std::vector<unsigned int> simulationDepthPerThread(
      cOpt().parallelization_options.n_simulationThreads, nodeDepth);

  // build the sum over all simulation threads and save it in agentsRewards
  for (size_t threadIdx = 0; threadIdx < m_multiThreadAgentsRewards.size(); ++threadIdx) {
    // start at the depth of the node where the simulation started and go up to the simulated depth
    // which is stored in simulationNode->m_depth
    for (size_t depthIdx = nodeDepth + 1; depthIdx <= m_simulationNodes[threadIdx]->m_depth;
         ++depthIdx) {
      // increase depth counter in vector
      ++simulationDepthPerThread[threadIdx];

      // simply iterating over all agents
      for (size_t agentIdx = 0; agentIdx < m_simulationNodes[threadIdx]->m_agents.size();
           ++agentIdx) {
        // += to sum over all threads
        agentsRewards[depthIdx - 1][agentIdx] +=
            m_multiThreadAgentsRewards[threadIdx][depthIdx - 1][agentIdx];
      }
    }
  }

  // After this loop simulationsAtDepth will contain the number of simulations
  // per depth step At depths lower than nodedepth + 1 the vector will contain
  // the value threadCount
  std::vector<unsigned int> simulationsAtDepth(maxSimDepth,
                                               cOpt().parallelization_options.n_simulationThreads);
  for (size_t depthIdx = nodeDepth + 1; depthIdx <= maxSimDepth; ++depthIdx) {
    for (size_t threadIdx = 0; threadIdx < cOpt().parallelization_options.n_simulationThreads;
         ++threadIdx) {
      if (simulationDepthPerThread[threadIdx] < depthIdx) {
        simulationsAtDepth[depthIdx - 1]--;
      }
    }
  }

  // divide by amount of simulations per depth
  // This will cause a problem if the program supports nodes with different amounts of Agents
  for (size_t depthIdx = nodeDepth + 1; depthIdx <= maxSimDepth; ++depthIdx) {
    for (size_t agentIdx = 0; agentIdx < m_simulationNodes[0]->m_agents.size(); ++agentIdx) {
      agentsRewards[depthIdx - 1][agentIdx] /= simulationsAtDepth[depthIdx - 1];
    }
  }
}

/**
 * @brief Calculates the variance for each simulation and writes it to a csv
 * file. Used for debug purposes only.
 *
 * @param nodeDepth The depth of the current node.
 * @param agentsRewards The result vector of the agents' rewards.
 */
void SimulationMultiThread::calculateVariance(
    const unsigned int nodeDepth, const std::vector<std::vector<float>>& agentsRewards) {
  unsigned int depthIdx{nodeDepth + 1};
  if (agentsRewards[depthIdx - 1][0] != 0) {
    float meanDifSum{0.0f};
    for (const auto& threadReward : m_multiThreadAgentsRewards) {
      // += to sum over all threads
      meanDifSum += (threadReward[depthIdx - 1][0] - agentsRewards[depthIdx - 1][0]) *
                    (threadReward[depthIdx - 1][0] - agentsRewards[depthIdx - 1][0]);
    }

    float variance =
        meanDifSum / static_cast<float>(cOpt().parallelization_options.n_simulationThreads);

    std::fstream fileStream;
    std::string fileName = "variance_" +
                           std::to_string(cOpt().parallelization_options.n_simulationThreads) +
                           "_threads";
    std::string filePath = oOpt().output_path + "/" + fileName + ".csv";

    fileStream.open(filePath, std::fstream::out | std::fstream::app);

    fileStream << std::to_string(variance);
    fileStream << std::endl;

    fileStream.close();
  }
}
/**
 * @brief Prints agentsRewards and m_multiThreadAgentsRewards into a csv file
 *  at m_path only used for debug purposes.
 *
 * @param agentsRewards Read only vector of the agents' rewards
 */
void SimulationMultiThread::simulationRewardsToCSV(
    const std::vector<std::vector<float>>& agentsRewards) {
  std::fstream fileStream;
  std::string fileName{"simulationRewards"};
  std::string filePath{oOpt().output_path + "/" + fileName + ".csv"};

  fileStream.open(filePath, std::fstream::out | std::fstream::app);

  for (const auto& threadReward : m_multiThreadAgentsRewards) {
    for (const auto& stepReward : threadReward) {
      fileStream << std::to_string(stepReward[0]) + ",";
    }
    fileStream << std::endl;
  }
  for (const auto& agentReward : agentsRewards) {
    fileStream << std::to_string(agentReward[0]) + ",";
  }
  fileStream << std::endl;
  fileStream << std::endl;

  fileStream.close();
}
}  // namespace proseco_planning
