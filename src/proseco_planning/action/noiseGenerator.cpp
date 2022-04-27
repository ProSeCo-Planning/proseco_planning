
#include "proseco_planning/action/noiseGenerator.h"

#include <boost/math/policies/policy.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include <memory>

#include "proseco_planning/action/action.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/math/mathlib.h"

namespace proseco_planning {

/**
 * @brief Constructs a new Noise Generator object
 *
 */
NoiseGenerator::NoiseGenerator() {
  m_distributionY =
      std::normal_distribution<float>(cOpt().action_noise.meanY, cOpt().action_noise.sigmaY);
  m_distributionVx =
      std::normal_distribution<float>(cOpt().action_noise.meanVx, cOpt().action_noise.sigmaVx);
  m_normalDensityY  = boost::math::normal_distribution<float>(cOpt().action_noise.meanY,
                                                             cOpt().action_noise.sigmaY);
  m_normalDensityVx = boost::math::normal_distribution<float>(cOpt().action_noise.meanVx,
                                                              cOpt().action_noise.sigmaVx);
}

/**
 * @brief Creates a noisy action using the noise parameters for the Gaussian distribution from the
 * configuration.
 *
 * @param action The action which is to become noisy.
 * @return ActionPtr The action with noise.
 */
ActionPtr NoiseGenerator::createNoisyAction(const ActionPtr& action) {
  float epsilonY{m_distributionY(math::Random::engine())};
  float epsilonVx{m_distributionVx(math::Random::engine())};
  float likelihoodY{boost::math::pdf(m_normalDensityY, epsilonY)};
  float likelihoodVx{boost::math::pdf(m_normalDensityVx, epsilonVx)};

  auto noisyAction = std::make_shared<Action>(action->m_velocityChange + epsilonVx,
                                              action->m_lateralChange + epsilonY);

  noisyAction->noise.m_likelihoodY   = likelihoodY;
  noisyAction->noise.m_likelihoodVx  = likelihoodVx;
  noisyAction->noise.m_muY           = action->m_lateralChange;
  noisyAction->noise.m_muVx          = action->m_velocityChange;
  noisyAction->noise.m_sigmaY        = cOpt().action_noise.sigmaY;
  noisyAction->noise.m_sigmaVx       = cOpt().action_noise.sigmaVx;
  noisyAction->m_selectionLikelihood = action->m_selectionLikelihood;
  return noisyAction;
}

/**
 * @brief Create noisy versions of all actions of the action set using the noise generator.
 *
 * @param actionSet The action set which is to become noisy.
 * @return ActionSet The action set with noise.
 */
ActionSet NoiseGenerator::createNoisyActions(const ActionSet& actionSet) {
  ActionSet noisyActionSet;
  for (const auto& action : actionSet) {
    noisyActionSet.push_back(createNoisyAction(action));
  }
  return noisyActionSet;
}
}  // namespace proseco_planning