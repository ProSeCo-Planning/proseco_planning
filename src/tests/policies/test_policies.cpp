/**
 * @file test_policies.cpp
 * @brief This file defines the test cases for the policies.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <memory>
#include <string>

#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/policies/selectionPolicy.h"

using namespace proseco_planning;

struct ConfigFixture {
  ConfigFixture() { Config::create(config::scenarioSimple, config::optionsSimple); }
  ~ConfigFixture() { Config::get()->reset(); }
};

BOOST_FIXTURE_TEST_SUITE(UCT, ConfigFixture)

BOOST_AUTO_TEST_CASE(instantiation) {
  std::string selectionPolicyName = "UCTProgressiveWidening";

  auto selectionPolicy = SelectionPolicy::createPolicy(selectionPolicyName);

  BOOST_REQUIRE_EQUAL(selectionPolicy->m_name, selectionPolicyName);
}

BOOST_AUTO_TEST_SUITE_END()
