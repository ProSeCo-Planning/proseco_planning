#include "proseco_planning/policies/selectionPolicy.h"

#include <iostream>

#include "proseco_planning/policies/selection/selectionUCTProgressiveWidening.h"

namespace proseco_planning {

/**
 * @brief Creates and returns a selection policy.
 * @note Currently only UCT Progressive Widening is supported.
 * @param name The name of the policy.
 * @return std::unique_ptr<SelectionPolicy> The pointer to the selection policy.
 */
std::unique_ptr<SelectionPolicy> SelectionPolicy::createPolicy(const std::string& name) {
  if (name == "UCTProgressiveWidening") {
    return std::make_unique<SelectionUCTProgressiveWidening>(name);
  } else {
    throw std::invalid_argument("Unknown selection policy type: " + name);
  }
}

}  // namespace proseco_planning
