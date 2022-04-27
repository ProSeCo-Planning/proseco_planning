#include "proseco_planning/policies/simulationPolicy.h"

#include <iostream>

#include "proseco_planning/action/actionSpace.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/node.h"
#include "proseco_planning/policies/policy.h"
#include "proseco_planning/policies/simulation/simulationMultiThread.h"
#include "proseco_planning/policies/simulation/simulationSingleThread.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {

/**
 * @brief Creates and returns a simulation policy either single-threaded or multi-threaded.
 *
 * @param name The name of the policy.
 * @param agentsSize The number of agents to be simulated.
 * @return std::unique_ptr<SimulationPolicy> The pointer to the simulation policy.
 */
std::unique_ptr<SimulationPolicy> SimulationPolicy::createPolicy(const std::string& name,
                                                                 int agentsSize) {
  const auto& cOptions = cOpt();
  if (cOptions.parallelization_options.n_simulationThreads > 1) {
    return std::make_unique<SimulationMultiThread>(name, agentsSize);
  } else if (cOptions.parallelization_options.n_simulationThreads == 1) {
    return std::make_unique<SimulationSingleThread>(name);
  } else {
    throw std::invalid_argument("Unknown final simulation policy type: " + name);
  }
}

/**
 * @brief Runs a simulation from the passed simulation node and collects the agentsRewards,
 * returning the maxSimDepth.
 *
 * @param simulationNode The start node of the simulation.
 * @param maxDepth Integer value for desired search depth.
 * @param agentsRewards The agent's rewards.
 * @return unsigned int Maximum simulation depth.
 */
unsigned int SimulationPolicy::simulate(Node* const simulationNode, unsigned int maxDepth,
                                        std::vector<std::vector<float> >& agentsRewards) {
  while (!Policy::isNodeTerminal(simulationNode, maxDepth)) {
    // Determine actionSet for simulation
    setSimulationActionSet(simulationNode);
    // Execute the action
    simulationNode->executeActions(simulationNode->m_actionSet, *m_collisionChecker,
                                   *m_trajectoryGenerator, false);
    // Increase the depth counter
    ++simulationNode->m_depth;
    // Extract the reward
    extractReward(simulationNode, agentsRewards);
  }
  return simulationNode->m_depth;
}

/**
 * @brief Determines the action set that is executed for the current step of the simulation.
 *
 * @param simulationNode Pointer to the simulation node.
 */
void SimulationPolicy::setSimulationActionSet(Node* const simulationNode) const {
  simulationNode->m_actionSet.clear();
  for (const auto& agent : simulationNode->m_agents) {
    if (agent.m_isPredefined) {
      // Predefined agent: Only drives straight if the ActionSpaceRectangle is used
      simulationNode->m_actionSet.push_back(agent.m_actionSpace->getPredefinedActions()[0]);
    } else if (m_name == "moderate") {
      /*
       * Uses the agent's current position to determine actions according to semantic move groups.
       * One of these actions is then sampled and added.
       */
      simulationNode->m_actionSet.push_back(
          agent.m_actionSpace->sampleModerateAction(agent.m_vehicle));
    } else {
      // Nothing else specified: Sample an action uniformly at random and add it
      simulationNode->m_actionSet.push_back(
          agent.m_actionSpace->sampleRandomAction(agent.m_vehicle));
    }
  }
}

}  // namespace proseco_planning