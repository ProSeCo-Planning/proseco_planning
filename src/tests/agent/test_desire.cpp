/**
 * @file test_desire.cpp
 * @brief This file defines the test cases for the desire.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include "proseco_planning/agent/desire.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"

using namespace proseco_planning;

struct DesireFixture {
  DesireFixture() { Config::create(config::scenarioSimple, config::optionsSimple); }

  ~DesireFixture() { Config::get()->reset(); }
};

BOOST_FIXTURE_TEST_SUITE(desire, DesireFixture)

BOOST_AUTO_TEST_SUITE_END()
