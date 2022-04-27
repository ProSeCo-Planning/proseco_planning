/**
 * @file noiseGenerator.h
 * @brief This file defines the NoiseGenerator class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <boost/math/distributions/normal.hpp>
#include <random>

#include "proseco_planning/util/alias.h"

namespace proseco_planning {

/**
 * @brief The NoiseGenerator is used to add noise to actions.
 *
 */
class NoiseGenerator {
 public:
  NoiseGenerator();

  ActionPtr createNoisyAction(const ActionPtr& action);

  ActionSet createNoisyActions(const ActionSet& actionSet);

 private:
  /// The normal distribution for the lateral position change.
  std::normal_distribution<float> m_distributionY;

  /// The normal distribution for the longitudinal velocity change.
  std::normal_distribution<float> m_distributionVx;

  /// The density of the normal distribution for the lateral position change. Used to calculate the
  /// probability for the lateral position change.
  boost::math::normal_distribution<float> m_normalDensityY;

  /// The density of the normal distribution for the longitudinal velocity change. Used to calculate
  /// the probability for the longitudinal velocity change.
  boost::math::normal_distribution<float> m_normalDensityVx;
};

}  // namespace proseco_planning