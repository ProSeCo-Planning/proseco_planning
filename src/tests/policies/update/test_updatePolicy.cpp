/**
 * @file test_updatePolicy.cpp
 * @brief This file defines the test cases for the update policy.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <boost/test/tools/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <vector>

#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/policies/updatePolicy.h"

using namespace proseco_planning;

BOOST_AUTO_TEST_CASE(discountedReward) {
  auto discounted_reward = UpdatePolicy::discountedReward(0.99, 5, 10);
  BOOST_CHECK_CLOSE(discounted_reward, 9.509900499, 0.0001);
}

BOOST_AUTO_TEST_CASE(cumulativeDiscountedReward) {
  std::vector<float> rewards0{0.5f, 0.5f, 0.5f};
  std::vector<float> rewards1{2.f, 2.f, 2.f};
  std::vector<float> rewards2{5.f, 5.f, 5.f};
  std::vector<float> rewards3{-10.f, -10.f, -10.f};
  std::vector<std::vector<float>> agents_rewards{rewards0, rewards1, rewards2, rewards3};

  auto cumulative_discounted_reward0 =
      UpdatePolicy::cumulativeDiscountedReward(0.99, 1, 1, agents_rewards, 1);
  BOOST_CHECK_CLOSE(cumulative_discounted_reward0, 0.5, 0.0001);

  auto cumulative_discounted_reward1 =
      UpdatePolicy::cumulativeDiscountedReward(0.99, 1, 2, agents_rewards, 1);
  BOOST_CHECK_CLOSE(cumulative_discounted_reward1, 2.48, 0.0001);

  auto cumulative_discounted_reward2 =
      UpdatePolicy::cumulativeDiscountedReward(0.99, 2, 4, agents_rewards, 0);
  BOOST_CHECK_CLOSE(cumulative_discounted_reward2, -2.851, 0.0001);

  auto cumulative_discounted_reward3 =
      UpdatePolicy::cumulativeDiscountedReward(0.99, 1, 4, agents_rewards, 2);
  BOOST_CHECK_CLOSE(cumulative_discounted_reward3, -2.32249, 0.0001);
}