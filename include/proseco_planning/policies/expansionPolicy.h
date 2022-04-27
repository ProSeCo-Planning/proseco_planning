/**
 * @file expansionPolicy.h
 * @brief This file defines the ExpansionPolicy class, the base class for all expansion policies. It
 * is used to expand the tree.
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
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Node;

/**
 * @brief The ExpansionPolicy class defines the base class for all expansion policies. It is used to
 * expand the tree.
 *
 */
class ExpansionPolicy : public Policy {
 public:
  explicit ExpansionPolicy(const std::string& name);

  /// The virtual destructor.
  virtual ~ExpansionPolicy() = default;

  static std::unique_ptr<ExpansionPolicy> createPolicy(const std::string& name);

  virtual Node* expandTree(Node* node, ActionSet& actionSet,
                           std::vector<std::vector<float> >& agentsRewards,
                           const unsigned int maxDepth) = 0;

 protected:
  /// The collision checker.
  std::unique_ptr<CollisionChecker> m_collisionChecker;

  /// The trajectory generator.
  std::unique_ptr<TrajectoryGenerator> m_trajectoryGenerator;
};
}  // namespace proseco_planning