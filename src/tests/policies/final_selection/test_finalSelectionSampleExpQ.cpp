/**
 * @file test_finalSelectionSampleExpQ.cpp
 * @brief This file defines the test cases for the final selection policy based on the sample
 * exponential Q-function.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <algorithm>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/policies/final_selection/finalSelectionSampleExpQ.h"
#include "proseco_planning/util/alias.h"

namespace utf = boost::unit_test;
using namespace proseco_planning;

struct ConfigFixture {
  ConfigFixture() { Config::create(config::scenarioSimple, config::optionsSimple); }
  ~ConfigFixture() { Config::get()->reset(); }
};

struct FinalSelectionSampleExpQFixture : ConfigFixture, FinalSelectionSampleExpQ {
  FinalSelectionSampleExpQFixture() : FinalSelectionSampleExpQ("") {}
};

BOOST_FIXTURE_TEST_SUITE(FinalSelectionSampleExpQ, FinalSelectionSampleExpQFixture)

BOOST_AUTO_TEST_CASE(exponentiated_action_value, *utf::tolerance(0.00001f)) {
  BOOST_TEST(FinalSelectionSampleExpQ::calculateActionWeight(1000.0f) ==
             std::numeric_limits<float>::max());
  BOOST_TEST(FinalSelectionSampleExpQ::calculateActionWeight(10.0f) ==
             std::numeric_limits<float>::max());
  BOOST_TEST(FinalSelectionSampleExpQ::calculateActionWeight(0.1f) == 22026.46579480f);
  BOOST_TEST(FinalSelectionSampleExpQ::calculateActionWeight(-0.1f) == 4.5399931e-05f);
  BOOST_TEST(FinalSelectionSampleExpQ::calculateActionWeight(-1.0f) == 0.0f);
}

BOOST_AUTO_TEST_CASE(sampleActionFromWeights, *utf::tolerance(0.00001f)) {
  math::Random::setRandomSeed(99);
  std::vector<float> weights{0.1f, 0.1f, 0.999f, 0.1f, 0.1f};
  auto [index, probability] = FinalSelectionSampleExpQ::sampleActionFromWeights(weights);
  BOOST_TEST(index == 2);
  BOOST_TEST(probability == 0.714081466f);
  weights                      = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f};
  std::tie(index, probability) = FinalSelectionSampleExpQ::sampleActionFromWeights(weights);
  BOOST_TEST(index == 4);
  BOOST_TEST(probability == 0.333333343f);
}

BOOST_AUTO_TEST_SUITE_END()