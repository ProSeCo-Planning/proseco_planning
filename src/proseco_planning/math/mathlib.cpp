#include <cmath>
#include <random>

#include "proseco_planning/math/mathlib.h"

/**
 * @brief The namespace where all pure math functions are defined.
 *
 */
namespace proseco_planning::math {

std::atomic_ulong Random::g_salt;
std::mt19937_64::result_type Random::g_seed = 0;

/**
 * @brief Generates a data sample from the distribution specified by its mean and its standard
 * deviation.
 *
 * @param mean The mean of the Gaussian distribution.
 * @param sigma The standard deviation of the Gaussian distribution.
 * @param limit If true, the noise will be limited to values below two sigma.
 * @return float A sample from the Gaussian distribution.
 */
float getNoise(float mean, float sigma, bool limit) {
  std::normal_distribution<float> dist(mean, sigma);

  float noise = dist(math::Random::engine());

  if (limit) {
    if (std::abs(noise - mean) > 2.0f * sigma) {
      noise = getNoise(mean, sigma, limit);
    }
  }

  return noise;
}

/**
 * @brief Calculates the cumulative moving average
 *
 * @param n The number of elements the current average is comprised of
 * @param average The current average
 * @param value The new value that should be included
 * @return float
 */
float cumulativeMovingAverage(const unsigned int n, const float average, const float value) {
  assert(n >= 0 && "negative n");
  return average + (value - average) / (n + 1);
}
}  // namespace proseco_planning::math
