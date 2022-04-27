/**
 * @file mathlib.h
 * @brief This file defines basic math tools.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <sys/types.h>
#include <algorithm>
#include <atomic>
#include <cassert>
#include <map>
#include <numeric>
#include <random>

namespace proseco_planning {
namespace math {

/**
 * ====================================================================
 * Thread-local global random engine.
 *
 * Why `global` (static)?
 *   Having a single random engine globally is easier to maintain
 *   and there is only a single point where we can initialize the seed.
 *
 * Why `thread_local`?
 *   The random engines are not threadsafe!
 *   When each thread has a local engine this issue is resolved.
 *
 * So basically having a `global` `thread_local` random engine solves both
 * threadsafety, usability and maintainability!
 * ====================================================================
 */

class Random {
 private:
  /// The thread safe counter
  static std::atomic_ulong g_salt;

 public:
  /// The seed for the thread-local random engines (volatile = force reload when used)
  static std::mt19937_64::result_type g_seed;

  /// Zero implies that the engine is randomly seeded
  static inline void setRandomSeed(const unsigned int seed) {
    g_seed = seed != 0 ? seed : std::random_device{}();
  }

  // Salt (increment) per thread and does not change!
  static inline std::mt19937_64::result_type& _salt() noexcept {
    thread_local std::mt19937_64::result_type salt = ++g_salt;
    return salt;
  }

  // Seed
  static inline std::mt19937_64::result_type& _seed() noexcept {
    thread_local std::mt19937_64::result_type seed = g_seed;
    return seed;
  }

  static void setSalt(std::mt19937_64::result_type salt) noexcept {
    _salt()  = salt;
    engine() = std::mt19937_64(_seed() + salt);
  }

  /// Gets the thread-local random engine (threadsafe)
  static std::mt19937_64& engine() noexcept {
    auto& seed = _seed();
    auto salt  = _salt();
    // Engine
    thread_local std::mt19937_64 randomEngine(seed + salt);

    // Check if global seed has changed?
    if (seed != g_seed) {  // Global random seed changed!
      seed         = g_seed;
      randomEngine = std::mt19937_64(seed + salt);
    }
    return randomEngine;
  }
};

float getNoise(float mean, float sigma, bool limit = false);

float cumulativeMovingAverage(const unsigned int n, const float average, const float value);

/**
 * @brief Calculates the Upper Confidence Bounds for Trees (UCT).
 *
 * @param actionValue The average value of that action.
 * @param childVisits The number of times that action/child has been visited.
 * @param parentVisits The number of times that parent has been visited.
 * @param c The constant that balances exploration and exploitation.
 * @return float The calculated UCT.
 */
inline float UCT(const float actionValue, const float childVisits, const float parentVisits,
                 const float c) {
  assert(actionValue >= 0 && actionValue <= 1 && "actionValue should be within [0,1]");
  assert(childVisits > 0 && "UCT undefined for unvisited nodes");
  return actionValue + c * std::sqrt(std::log(parentVisits) / childVisits);
}

/**
 * @brief Normalizes a value between 0 and 1.
 *
 * @param value The value to be normalized.
 * @param max The maximum value of the variable.
 * @param min The minimum value of the variable.
 * @return float The normalized value.
 */
inline float normalize(const float value, const float max, const float min) {
  assert(max > min && "max must be larger than min");
  return (value - min) / (max - min);
}

/**
 * @brief Calculates the magnitude of a 2D vector.
 *
 * @param x The first component of the vector.
 * @param y The second component of the vector.
 * @return float The magnitude of the vector.
 */
inline float magnitude(const float x, const float y) { return std::sqrt(x * x + y * y); }

/**
 * @brief Creates a linearly spaced vector of the size n using the start and end values.
 *
 * @tparam T The type of the vector.
 * @param start The start value.
 * @param end The end value.
 * @param n The number of elements in the vector.
 * @return std::vector<T>
 */

template <typename T>
std::vector<T> linspace(const T start, const T end, const unsigned int n) {
  assert(n > 0 && "n must be greater than 0");
  std::vector<T> result(n);
  const T step = (end - start) / T(n - 1);
  for (unsigned int i = 0; i < n; ++i) {
    result[i] = start + i * step;
  }
  return result;
}

/**
 * @brief Extracts a subvector from a vector.
 *
 * @tparam T The type of the vector.
 * @param vector The vector to extract from.
 * @param start The start index.
 * @param end The end index.
 * @return std::vector<T> The extracted subvector.
 */
template <typename T>
std::vector<T> getSubvectorFromVector(const std::vector<T>& vector, const size_t start,
                                      const size_t end) {
  return {vector.begin() + start, vector.begin() + end + 1};
}

/**
 * @brief Evaluates whether two values are equal given an epsilon error.
 * @tparam a The first value.
 * @tparam b The second value.
 * @tparam epsilon The error tolerance.
 */
template <typename T>
inline bool isEqual(const T a, const T b, const T epsilon) {
  return std::abs(a - b) <= epsilon;
}

/**
 * @brief Calculates the absolute sum of a vector.
 *
 * @tparam T The type of the vector.
 * @param vector The vector.
 * @return T The absolute sum of the vector.
 */
template <typename T>
inline T absSum(const std::vector<T>& vector) {
  return std::transform_reduce(vector.begin(), vector.end(), 0, std::plus<>{},
                               static_cast<T (*)(T)>(std::abs));
}

/**
 * @brief Calculates the sum of a vector.
 *
 * @tparam T The type of the vector.
 * @param vector The vector.
 * @return float The sum of the vector.
 */
template <typename T>
float sumFromVector(const std::vector<T>& vector) {
  return std::reduce(vector.begin(), vector.end());
}

/**
 * @brief Calculates the mean of a vector.
 *
 * @tparam T The type of the vector.
 * @param vector The vector.
 * @return float The mean of the vector.
 */
template <typename T>
float meanFromVector(const std::vector<T>& vector) {
  return sumFromVector(vector) / static_cast<float>(vector.size());
}

/**
 * @brief Calculates the variance of a vector.
 *
 * @tparam T The type of the vector.
 * @param vector The vector.
 * @return float The variance of the vector.
 */
template <typename T>
float varFromVector(const std::vector<T>& vector) {
  auto mean{meanFromVector(vector)};
  return std::transform_reduce(vector.begin(), vector.end(), 0.0f, std::plus<>{},
                               [mean](T x) { return (x - mean) * (x - mean); }) /
         static_cast<float>(vector.size());
}

/**
 * @brief Calculates the standard deviation of a vector.
 *
 * @tparam T The type of the vector.
 * @param vector The vector.
 * @return float The standard deviation of the vector.
 */
template <typename T>
float stdFromVector(const std::vector<T>& vector) {
  return std::sqrt(varFromVector(vector));
}

/**
 * @brief Gets a random index from an std::vector.
 *
 * @tparam T The type of the vector.
 * @param container The vector.
 * @return size_t The random index.
 */
template <typename T>
inline size_t getRandomIndexFromVector(const std::vector<T>& vector) {
  assert(vector.size() > 0 && "vector must be larger than 0 ");
  std::uniform_int_distribution<std::size_t> indexes(0, vector.size() - 1);
  return indexes(math::Random::engine());
}

/**
 * @brief Gets a random element from an std::vector.
 *
 * @tparam T The type of the vector.
 * @param vector The vector.
 * @return T The random element.
 */
template <typename T>
inline T getRandomElementFromVector(const std::vector<T>& vector) {
  return vector[getRandomIndexFromVector(vector)];
}

/**
 * @brief Gets a random number in the interval of [a,b).
 *
 * @tparam T The type of `a` and `b`.
 * @param a The start of the interval (included).
 * @param b The end of the interval (not included).
 * @return T The random number.
 */
template <typename T>
inline T getRandomNumberInInterval(const T a, const T b) {
  std::uniform_real_distribution<T> distribution(a, b);
  return distribution(math::Random::engine());
}

/**
 * @brief Copies values from a map to a vector.
 *
 * @tparam T The key type of the map.
 * @tparam U The value type of the map.
 * @param map The map.
 * @return std::vector<T> The vector with the values of the map.
 */
template <typename T, typename U>
std::vector<U> mapValuesToVector(const std::map<T, U>& map) {
  std::vector<U> vector;
  vector.reserve(map.size());
  for (const auto& [_, value] : map) {
    vector.push_back(value);
  }
  return vector;
}

/**
 * @brief Map iterator to the max element of the map.
 *
 * @tparam The value type.
 * @tparam The key type.
 * @param map
 * @return auto The maximum element iterator.
 */
template <class T, class U>
static const auto max_map_iterator(const std::map<U, T>& map) {
  return std::max_element(
      map.begin(), map.end(),
      [](const std::pair<U, T>& p1, const std::pair<U, T>& p2) { return p1.second < p2.second; });
}

/**
 * @brief Map iterator to the min element of the map.
 *
 * @tparam The value type.
 * @tparam The key type.
 * @param map
 * @return auto The minimum element iterator.
 */
template <class T, class U>
static const auto min_map_iterator(const std::map<U, T>& map) {
  return std::min_element(
      map.begin(), map.end(),
      [](const std::pair<U, T>& p1, const std::pair<U, T>& p2) { return p1.second < p2.second; });
}

/**
 * @brief Returns the max element of all elements of a map.
 *
 * @tparam The value type.
 * @tparam The key type.
 * @param map
 * @return The maximum element.
 */
template <class T, class U>
static U max_map_element(const std::map<U, T>& map) {
  return max_map_iterator(map)->first;
}

/**
 * @brief Returns the max value of all elements of a map.
 *
 * @tparam The value type.
 * @tparam The key type.
 * @param map
 * @return The maximum value.
 */
template <class T, class U>
static T max_map_value(const std::map<U, T>& map) {
  return max_map_iterator(map)->second;
}

/**
 * @brief Returns the min element of all elements of a map.
 *
 * @tparam The value type.
 * @tparam The key type.
 * @param map
 * @return The minium element.
 */
template <class T, class U>
static U min_map_element(const std::map<U, T>& map) {
  return min_map_iterator(map)->first;
}

/**
 * @brief Returns the min value of all elements of a map.
 *
 * @tparam The value type.
 * @tparam The key type.
 * @param map
 * @return The minium value.
 */
template <class T, class U>
static T min_map_value(const std::map<U, T>& map) {
  return min_map_iterator(map)->second;
}

/**
 * @brief This data structure holds a min and max value.
 *
 * @tparam T The type of the members.
 */
template <typename T>
struct MinMaxPair {
  T min;
  T max;
};
}  // namespace math
}  // namespace proseco_planning