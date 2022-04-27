#include "proseco_planning/policies/expansionPolicy.h"

#include <iostream>

#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/policies/expansion/expansionUCT.h"
#include "proseco_planning/policies/policy.h"

namespace proseco_planning {
/**
 * @brief Constructs a new Expansion Policy object.
 *
 * @param name Constructs Expansion Policy specified by received String.
 */
ExpansionPolicy::ExpansionPolicy(const std::string& name) : Policy(name) {
  m_collisionChecker    = CollisionChecker::createCollisionChecker(cOpt().collision_checker);
  m_trajectoryGenerator = TrajectoryGenerator::createTrajectoryGenerator(cOpt().trajectory_type);
}

/**
 * @brief Creates a pointer to a unique ExpansionPolicy object.
 *
 * @param name Desired Policy as a String with default UCT.
 * @return std::unique_ptr<ExpansionPolicy> Pointer to the ExpansionPolicy object.
 */
std::unique_ptr<ExpansionPolicy> ExpansionPolicy::createPolicy(const std::string& name) {
  if (name == "UCT") {
    return std::make_unique<ExpansionUCT>(name);
  } else {
    throw std::invalid_argument("Unknown expansion policy type: " + name);
  }
}
}  // namespace proseco_planning
