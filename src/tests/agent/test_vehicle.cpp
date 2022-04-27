/**
 * @file test_vehicle.cpp
 * @brief This file defines the test cases for the vehicle.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"

// detailed testing of "simulation" with the trajectory generator can be found
// within test_trajectoryGenerator

using namespace proseco_planning;

struct VehicleFixture {
  VehicleFixture() { Config::create(config::scenarioSimple, config::optionsSimple); }

  ~VehicleFixture() { Config::get()->reset(); }
};

BOOST_FIXTURE_TEST_SUITE(vehicle, VehicleFixture)

BOOST_AUTO_TEST_CASE(accelerate) {
  Vehicle vehicle(config::vehicle);

  vehicle.m_accelerationX = 8;
  vehicle.m_velocityX     = 10;
  BOOST_CHECK_EQUAL(vehicle.m_accelerationX, 8);
}

BOOST_AUTO_TEST_SUITE_END()
