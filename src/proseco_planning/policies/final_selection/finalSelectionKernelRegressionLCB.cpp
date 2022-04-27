#include "proseco_planning/policies/final_selection/finalSelectionKernelRegressionLCB.h"

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <utility>

#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/node.h"

namespace proseco_planning {

/**
 * @brief Constructs a new FinalSelectionKernelRegressionLCB object.
 *
 * @param name The name of the policy.
 */
// initialize the policy
FinalSelectionKernelRegressionLCB::FinalSelectionKernelRegressionLCB(const std::string& name)
    : FinalSelectionPolicy::FinalSelectionPolicy(name) {
  /// @todo Find a way to remove these and still have working tests (instantiate config new in each
  /// test with values?)
  m_moveGrouping     = cOpt().policy_options.kernel_regression_lcb.move_grouping;
  m_gammaAction      = cOpt().policy_options.kernel_regression_lcb.action.gamma;
  m_gammaActionClass = cOpt().policy_options.kernel_regression_lcb.action_class.gamma;
  m_cpAction         = cOpt().policy_options.kernel_regression_lcb.action.cp;
  m_cpActionClass    = cOpt().policy_options.kernel_regression_lcb.action_class.cp;
}

/**
 * @brief Returns the best action set for the current node based on kernel regression.
 *
 * @param node Pointer to the final selection node (e.g., the root node).
 * @return ActionSet The best action set.
 */
ActionSet FinalSelectionKernelRegressionLCB::getBestActionSet(const Node* const node) {
  if (m_moveGrouping) {
    setBestActionClass(node);
    // since m_moveGrouping is true, getBestActionSet considers only actions which belong to the
    // best action class
    setBestActionSet(node);
  }
  // no moveGrouping
  else {
    setBestActionSet(node);
  }
  return m_bestActionSet;
}

/**
 * @brief Determines the best action class and updates m_bestActionClassSet accordingly.
 * @param node node The node to determine the best action class for.
 */
void FinalSelectionKernelRegressionLCB::setBestActionClass(const Node* const node) {
  // reset the former bestActionClassSet
  m_bestActionClassSet.clear();

  for (const auto& agent : node->m_agents) {
    // store the density and the weighted value for each action class
    std::vector<float> densities;
    densities.reserve(agent.m_actionClassValues.size());
    std::vector<float> weightedValues;
    weightedValues.reserve(agent.m_actionClassValues.size());

    float sumDensities{0.0f};

    // calculate for each centerActionClass the density and weighted value
    for (const auto& [centerClass, _] : agent.m_actionClassValues) {
      float density{0.0f};
      float weightedValue{0.0f};

      for (const auto& [compareClass, compareValue] : agent.m_actionClassValues) {
        float similarity   = getSimilarity(centerClass, compareClass);
        float densityDelta = similarity * agent.m_actionClassVisits.at(compareClass);
        density += densityDelta;
        weightedValue += densityDelta * compareValue;
      }
      // save results for centerActionClass
      densities.push_back(density);
      sumDensities += density;
      weightedValues.push_back(weightedValue);
    }

    ActionClass bestActionClass;

    // catch case of unexplored action classes -> sumDensity (near) zero
    // select group randomly
    if (sumDensities < 0.1) {
      auto randomAction = getRandomAction(agent);
      bestActionClass   = randomAction->m_actionClass;
    }
    // use kernel regression lower confidence bound value for selection
    else {
      // stores the kernel regression value for each action class
      std::vector<float> kernelRegValues;
      kernelRegValues.reserve(agent.m_actionClassValues.size());

      for (size_t i = 0; i < agent.m_actionClassValues.size(); ++i) {
        kernelRegValues.push_back(weightedValues.at(i) / densities.at(i));
      }

      // get max and min for normalization of kernel regression values
      float maxKR{*std::max_element(kernelRegValues.cbegin(), kernelRegValues.cend())};
      float minKR{*std::min_element(kernelRegValues.cbegin(), kernelRegValues.cend())};
      bool is_maxKR_equal_minKR{(maxKR - minKR < cOpt().error_tolerance)};

      float currentMax{-std::numeric_limits<float>::max()};
      float logSumDensities{std::log(sumDensities)};

      auto classValueIter = agent.m_actionClassValues.cbegin();
      for (size_t iClass = 0; classValueIter != agent.m_actionClassValues.cend();
           ++iClass, ++classValueIter) {
        // normalized kernel regression value
        // use a default value if maxKR == minKR
        float normKernelRegValue =
            is_maxKR_equal_minKR ? 0.5f : (kernelRegValues.at(iClass) - minKR) / (maxKR - minKR);
        // calculate the kernel regression lower confidence bound value
        float currentValue = normKernelRegValue -
                             m_cpActionClass * std::sqrt(logSumDensities / densities.at(iClass));
        if (currentValue > currentMax) {
          currentMax      = currentValue;
          bestActionClass = classValueIter->first;
        }
      }
    }
    // update m_bestActionClassSet
    m_bestActionClassSet.push_back(bestActionClass);
  }
}

/**
 * @brief Determines the best action set and updates m_bestActionSet accordingly. If m_moveGrouping
 * is true, only actions which belong to m_bestActionClassSet are considered.
 * @param node node The node to determine the best action set for.
 */
void FinalSelectionKernelRegressionLCB::setBestActionSet(const Node* const node) {
  // reset the former bestActionSet
  m_bestActionSet.clear();

  size_t iAgent = 0;  // loop index for agent
  for (const auto& agent : node->m_agents) {
    // store the density and the weighted value for each action
    std::vector<float> densities;
    densities.reserve(agent.m_actionValues.size());
    std::vector<float> weightedValues;
    weightedValues.reserve(agent.m_actionValues.size());

    float sumDensities{0.0f};

    // calculate for each centerAction the density and weighted value
    for (const auto& [centerAction, _] : agent.m_actionValues) {
      float density{0.0f};
      float weightedValue{0.0f};

      for (const auto& [compareAction, compareValue] : agent.m_actionValues) {
        float similarity   = Action::getSimilarity(centerAction, compareAction, m_gammaAction);
        float densityDelta = similarity * agent.m_actionVisits.at(compareAction);
        density += densityDelta;
        weightedValue += densityDelta * compareValue;
      }
      // save results for centerAction
      densities.push_back(density);
      sumDensities += density;
      weightedValues.push_back(weightedValue);
    }

    ActionPtr bestAction;

    // catch case of unexplored actions -> sumDensity (near) zero
    // select action randomly
    if (sumDensities < 0.1) {
      bestAction = getRandomAction(agent);
    }
    // use kernel regression lower confidence bound value for selection
    else {
      // stores the kernel regression value for each action
      std::vector<float> kernelRegValues;
      kernelRegValues.reserve(agent.m_actionValues.size());

      for (size_t i = 0; i < agent.m_actionValues.size(); ++i) {
        kernelRegValues.push_back(weightedValues.at(i) / densities.at(i));
      }

      // get max and min for normalization of kernel regression values
      float maxKR{*std::max_element(kernelRegValues.cbegin(), kernelRegValues.cend())};
      float minKR{*std::min_element(kernelRegValues.cbegin(), kernelRegValues.cend())};
      bool is_maxKR_equal_minKR{(maxKR - minKR < cOpt().error_tolerance)};

      float currentValue;
      float currentMax{-std::numeric_limits<float>::max()};
      float logSumDensities{std::log(sumDensities)};

      auto actionValueIter = agent.m_actionValues.cbegin();
      for (size_t iAction = 0; actionValueIter != agent.m_actionValues.cend();
           ++iAction, ++actionValueIter) {
        // if move grouping is active, only actions which belong to the bestActionClass are
        // considered
        if (m_moveGrouping &&
            actionValueIter->first->m_actionClass != m_bestActionClassSet.at(iAgent)) {
          continue;
        }
        // normalized kernel regression value
        // use a default value if maxKR == minKR
        float normKernelRegValue =
            is_maxKR_equal_minKR ? 0.5f : (kernelRegValues.at(iAction) - minKR) / (maxKR - minKR);
        // calculate the kernel regression lower confidence bound value
        currentValue =
            normKernelRegValue - m_cpAction * std::sqrt(logSumDensities / densities.at(iAction));
        if (currentValue > currentMax) {
          currentMax = currentValue;
          bestAction = actionValueIter->first;
        }
      }
    }
    // update m_bestActionSet
    m_bestActionSet.push_back(bestAction);
    ++iAgent;  // increment loop index for agent
  }
}

/**
 * @brief Calculates the distance between action classes using a kernel specified by
 * m_kernelActionClass.
 * @param centerActionClass centerActionClass
 * @param compareActionClass compareActionClass
 * @return float Distance between centerActionClass and compareActionClass.
 */
float FinalSelectionKernelRegressionLCB::getSimilarity(
    const ActionClass& centerActionClass, const ActionClass& compareActionClass) const {
  // use kernel dependant on m_kernelActionClass
  // no other kernels are supported yet
  return useManhattanKernel(centerActionClass, compareActionClass);
}

/**
 * @brief Uses the Manhattan metric to compute the distance between action classes.
 * @param centerActionClass centerActionClass
 * @param compareActionClass compareActionClass
 * @return Distance between centerActionClass and compareActionClass.
 */
float FinalSelectionKernelRegressionLCB::useManhattanKernel(
    const ActionClass& centerActionClass, const ActionClass& compareActionClass) const {
  using ac = ActionClass;

  // maps every actionClass to an index pair (deltaVelocity, deltaY)
  // (-1,+1) | (+0,+1) | (+1,+1)
  // (-1,+0) | (+0,+0) | (+1,+0)
  // (-1,-1) | (+0,-1) | (+1,-1)
  // static guarantees that map is only initialized in the first call of the method and is reused
  // afterwards
  static const std::map<ActionClass, std::pair<int, int>> actionClassMap = {
      {ac::CHANGE_LEFT_SLOW, {-1, 1}},   {ac::CHANGE_LEFT, {0, 1}},
      {ac::CHANGE_LEFT_FAST, {1, 1}},    {ac::DECELERATE, {-1, 0}},
      {ac::DO_NOTHING, {0, 0}},          {ac::ACCELERATE, {1, 0}},
      {ac::CHANGE_RIGHT_SLOW, {-1, -1}}, {ac::CHANGE_RIGHT, {0, -1}},
      {ac::CHANGE_RIGHT_FAST, {1, -1}}};

  // get (deltaVelocity, deltaY) for centerActionClass and compareActionClass
  auto&& [centerVel, centerY]   = actionClassMap.at(centerActionClass);
  auto&& [compareVel, compareY] = actionClassMap.at(compareActionClass);

  return std::exp(-m_gammaActionClass *
                  (std::abs(compareVel - centerVel) + std::abs(compareY - centerY)));
}
}  // namespace proseco_planning