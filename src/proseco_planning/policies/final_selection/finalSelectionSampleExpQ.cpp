#include "proseco_planning/policies/final_selection/finalSelectionSampleExpQ.h"

#include <map>
#include <memory>
#include <random>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/node.h"

namespace proseco_planning {

/**
 * @brief Calculates the exponential action value of an action factored by a constant q_scale.
 * @note The action weight can be infinity due to "floating point overflow". Since the resulting
 * distribution would return NaN as a probability, the action weight is set to the maximum limit of
 * float. Furthermore, the action weight can be zero due to "floating point underflow". Since a
 * distribution cannot be zero at any given point, the action weight is set to the minimum limit of
 * float. If the weight is NaN, this functions throws an exception.
 *
 * @param actionValue The action value.
 * @return float The exponential action value.
 */
float FinalSelectionSampleExpQ::calculateActionWeight(const float actionValue) {
  float weight = std::exp(actionValue * cOpt().policy_options.policy_enhancements.q_scale);
  weight       = std::isinf(weight) ? std::numeric_limits<float>::max() : weight;
  weight       = weight == 0 ? std::numeric_limits<float>::min() : weight;
  if (std::isnan(weight)) {
    throw std::runtime_error("The action weight is NaN.");
  } else {
    return weight;
  }
}

/**
 * @brief Instantiate the softmax Q distribution over the weights of the actions, and sample an
 * action from it.
 * @note std::discrete_distribution automatically normalizes the actionWeights to obtain a valid
 * distribution.
 *
 * @param weights The weights of the actions.
 * @return std::tuple<unsigned int, float> The index of the sampled action and its probability.
 */
std::tuple<unsigned int, float> FinalSelectionSampleExpQ::sampleActionFromWeights(
    std::vector<float> weights) {
  std::discrete_distribution<unsigned int> distribution(weights.begin(), weights.end());
  unsigned int index = distribution(math::Random::engine());
  float probability  = static_cast<float>(distribution.probabilities()[index]);
  return {index, probability};
}

/**
 * @brief Gets the best action set based on the resulting policy of the exponential action values.
 *
 * @param node The node to get the best action set from.
 * @return ActionSet The best action set.
 */
ActionSet FinalSelectionSampleExpQ::getBestActionSet(const Node* const node) {
  m_bestActionSet.clear();

  for (const auto& agent : node->m_agents) {
    // Initialize placeholders
    std::vector<float> actionWeights;
    ActionSet actions;
    /*
     * Calculate the weights for defining a categorical distribution over an
     * agent's actions. The weights for selecting each action are defined by applying the
     * softmax function over the agent's Q-values.
     */
    for (const auto& [action, value] : agent.m_actionValues) {
      actionWeights.push_back(calculateActionWeight(value));
      actions.push_back(action);
    }

    auto [index, probability] = sampleActionFromWeights(actionWeights);
    auto bestAction           = actions[index];
    // Add the sampling probability to the chosen action
    bestAction->m_selectionLikelihood = probability;
    // Add the unnormalized action selection weights to the chosen action
    bestAction->m_selectionWeights = actionWeights;
    // Append the action for agent i to the final action set
    m_bestActionSet.push_back(bestAction);
  }

  return m_bestActionSet;
}
}  // namespace proseco_planning
