#include "proseco_planning/policies/simulation/simulationSingleThread.h"

#include <memory>

#include "proseco_planning/collision_checker/collisionChecker.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/node.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"

namespace proseco_planning {

/**
 * @brief Constructs a new single-threaded simulation object according to the specified policy.
 *
 * @param name The name that specifies the policy of the simulation.
 */
SimulationSingleThread::SimulationSingleThread(const std::string& name) : SimulationPolicy(name) {
  m_collisionChecker    = CollisionChecker::createCollisionChecker(cOpt().collision_checker);
  m_trajectoryGenerator = TrajectoryGenerator::createTrajectoryGenerator(cOpt().trajectory_type);
}
/**
 * @brief Creates a new simulation node pointer and starts a simulation.
 *
 * @param node The node from which the simulation should start.
 * @param agentsRewards The agent's rewards.
 * @param maxDepth The maximum simulation depth.
 * @return unsigned int The maximum simulation depth.
 */
unsigned int SimulationSingleThread::runSimulation(Node* const node,
                                                   std::vector<std::vector<float> >& agentsRewards,
                                                   unsigned int maxDepth) {
  auto simulationNode = std::make_unique<Node>(node);
  return simulate(simulationNode.get(), maxDepth, agentsRewards);
}
}  // namespace proseco_planning