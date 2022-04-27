#include "proseco_planning/policies/expansion/expansionUCT.h"

#include <memory>

#include "proseco_planning/node.h"
#include "proseco_planning/policies/policy.h"

namespace proseco_planning {

/**
 * @brief Expands the search tree by adding a child node.
 *
 * @param node Pointer to the parent node.
 * @param actionSet Action set that leads from the parent node to the child node.
 * @param agentsRewards Vector that contains for every step along the tree path another vector
 * that stores the reward for each agent; update this parameter by reference.
 * @param maxDepth Maximum depth of the search tree.
 * @return Pointer to the child node.
 */
Node* ExpansionUCT::expandTree(Node* node, ActionSet& actionSet,
                               std::vector<std::vector<float> >& agentsRewards,
                               const unsigned int maxDepth) {
  // Check if node is expandable
  if (!Policy::isNodeTerminal(node, maxDepth)) {
    // Create a child node
    node = node->addChild(actionSet);
    // execute the action used for reaching the child
    node->executeActions(actionSet, *m_collisionChecker, *m_trajectoryGenerator, false);
    // extract the reward
    extractReward(node, agentsRewards);
  }
  return node;
}
}  // namespace proseco_planning
