/**
 * @file test_mathlib.cpp
 * @brief This file defines the test cases for the math library.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <array>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <random>
#include <thread>

#include "proseco_planning/math/mathlib.h"

using namespace proseco_planning;

BOOST_AUTO_TEST_SUITE(mathlibTest)

BOOST_AUTO_TEST_CASE(local_random_engine_basic) {
  std::mt19937_64::result_type a, b, c;
  std::thread ta([&a]() { a = math::Random::engine()(); });
  std::thread tb([&b]() { b = math::Random::engine()(); });
  std::thread tc([&c]() { c = math::Random::engine()(); });

  ta.join();
  tb.join();
  tc.join();

  BOOST_CHECK_NE(a, b);
  BOOST_CHECK_NE(a, c);
}

BOOST_AUTO_TEST_CASE(local_random_engine_seed) {
  std::mt19937_64::result_type seed = 0;
  math::Random::g_seed              = seed;

  std::array<std::mt19937_64::result_type, 2> a, b, c;
  std::thread ta([&a]() { a = {math::Random::engine()(), math::Random::engine()()}; });
  std::thread tb([&b]() { b = {math::Random::engine()(), math::Random::engine()()}; });
  std::thread tc([&c]() { c = {math::Random::engine()(), math::Random::engine()()}; });

  ta.join();
  tb.join();
  tc.join();

  BOOST_CHECK_NE(a[0], b[0]);
  BOOST_CHECK_NE(a[0], c[0]);
  BOOST_CHECK_NE(a[1], b[1]);
  BOOST_CHECK_NE(a[1], c[1]);

  BOOST_CHECK_NE(a[0], a[1]);
  BOOST_CHECK_NE(b[0], b[1]);
  BOOST_CHECK_NE(c[0], c[1]);
}

BOOST_AUTO_TEST_CASE(random_engine_update_global_seed) {
  math::Random::g_seed = 0;

  std::mt19937_64::result_type a1, a2, a3, a4;
  a1 = math::Random::engine()();
  a2 = math::Random::engine()();

  // update the global seed value
  math::Random::g_seed = 7;

  a3 = math::Random::engine()();
  a4 = math::Random::engine()();

  BOOST_CHECK_NE(a1, a2);
  BOOST_CHECK_NE(a3, a4);
}

BOOST_AUTO_TEST_CASE(local_random_engine_fixed) {
  std::mt19937_64::result_type seed = 0;
  std::mt19937_64 ref(seed);
  std::array<std::mt19937_64::result_type, 2> r = {ref(), ref()};

  std::array<std::mt19937_64::result_type, 2> a, b, c;

  std::thread ta([&a, seed]() {
    math::Random::engine() = std::mt19937_64(seed);
    a                      = {math::Random::engine()(), math::Random::engine()()};
  });
  std::thread tb([&b, seed]() {
    math::Random::engine() = std::mt19937_64(seed);
    b                      = {math::Random::engine()(), math::Random::engine()()};
  });
  std::thread tc([&c, seed]() {
    math::Random::engine() = std::mt19937_64(seed);
    c                      = {math::Random::engine()(), math::Random::engine()()};
  });

  ta.join();
  tb.join();
  tc.join();

  BOOST_CHECK_EQUAL(r[0], a[0]);
  BOOST_CHECK_EQUAL(r[0], b[0]);
  BOOST_CHECK_EQUAL(r[0], c[0]);

  BOOST_CHECK_EQUAL(r[1], a[1]);
  BOOST_CHECK_EQUAL(r[1], b[1]);
  BOOST_CHECK_EQUAL(r[1], c[1]);
}

BOOST_AUTO_TEST_CASE(cumulative_moving_average) {
  BOOST_CHECK_CLOSE(math::cumulativeMovingAverage(1, 10, 5), 7.5, 0.0001);
  BOOST_CHECK_CLOSE(math::cumulativeMovingAverage(100, 10, 5), 9.950495, 0.0001);
}

BOOST_AUTO_TEST_CASE(linearly_spaced_vector) {
  auto v = math::linspace(0.0f, 2.0f, 5);
  // expected vector
  std::vector<float> exp_v{0.0f, 0.5f, 1.0f, 1.5f, 2.0f};

  BOOST_CHECK_EQUAL_COLLECTIONS(v.begin(), v.end(), exp_v.begin(), exp_v.end());
}

BOOST_AUTO_TEST_CASE(subvector_from_vector) {
  std::vector<int> v{0, 1, 2, 3, 4};
  auto sub_v = math::getSubvectorFromVector(v, 0, 2);
  // expected subvector
  std::vector<int> exp_sub_v{0, 1, 2};

  BOOST_CHECK_EQUAL_COLLECTIONS(sub_v.begin(), sub_v.end(), exp_sub_v.begin(), exp_sub_v.end());
}

BOOST_AUTO_TEST_CASE(absolute_sum_of_vector) {
  std::vector<float> v{0.5f, 1.5f, 3.0f, -4.0f};
  BOOST_CHECK_CLOSE(math::absSum(v), 9.0f, 0.0001);
}

BOOST_AUTO_TEST_CASE(sum_of_vector) {
  std::vector<float> v{0.5f, 1.5f, 3.0f, -4.0f};
  BOOST_CHECK_CLOSE(math::sumFromVector(v), 1.0f, 0.0001);
}

BOOST_AUTO_TEST_CASE(mean_of_vector) {
  std::vector<float> v{0.5f, 1.5f, 3.0f, -4.0f};
  BOOST_CHECK_CLOSE(math::meanFromVector(v), 0.25f, 0.0001);
}

BOOST_AUTO_TEST_CASE(variance_of_vector) {
  std::vector<float> v{0.5f, 1.5f, 3.0f, -4.0f};
  BOOST_CHECK_CLOSE(math::varFromVector(v), 6.8125f, 0.0001);
}

BOOST_AUTO_TEST_CASE(std_of_vector) {
  std::vector<float> v{0.5f, 1.5f, 3.0f, -4.0f};
  BOOST_CHECK_CLOSE(math::stdFromVector(v), 2.61007667f, 0.0001);
}

BOOST_AUTO_TEST_CASE(map_values_to_vector) {
  std::map<int, float> map{{0, 0.5f}, {1, 1.5f}, {2, 3.0f}, {3, -4.0f}};
  std::vector<float> vector = math::mapValuesToVector(map);
  for (size_t i = 0; i < vector.size(); ++i) {
    BOOST_CHECK_CLOSE(vector[i], map[i], 0.0001);
  }
}

BOOST_AUTO_TEST_SUITE_END()
