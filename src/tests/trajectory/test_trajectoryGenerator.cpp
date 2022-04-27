/**
 * @file test_trajectoryGenerator.cpp
 * @brief This file defines the test cases for the trajectory generator.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <memory>

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/trajectory/trajectory.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"

using namespace proseco_planning;

struct TrajectoryPlannerFixture {
  TrajectoryPlannerFixture() { Config::create(config::scenarioSimple, config::optionsSimple); }

  ~TrajectoryPlannerFixture() { Config::get()->reset(); }
};

BOOST_FIXTURE_TEST_SUITE(trajectoryPlanner, TrajectoryPlannerFixture)

BOOST_AUTO_TEST_CASE(laneDetection) {
  // Create road

  float lateralChange{-1.0};
  float velocityChange{5.0};
  Vehicle vehicleConstantAcceleration(config::vehicle);
  Vehicle vehiclePolynomialTrajectory(config::vehicle);
  auto action = std::make_shared<Action>(Action(velocityChange, lateralChange));

  // constant Acceleration
  const auto trajectoryGeneratorC =
      TrajectoryGenerator::createTrajectoryGenerator("constantAcceleration");

  auto trajectoryConstantAcceleration =
      trajectoryGeneratorC->createTrajectory(0.0, action, vehicleConstantAcceleration);

  vehicleConstantAcceleration.updateState(trajectoryConstantAcceleration.m_finalState);

  auto finalStateConstantAcceleration = trajectoryConstantAcceleration.m_finalState;

  // polynomial
  const auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  auto trajectoryPolynomial =
      trajectoryGeneratorP->createTrajectory(0.0, action, vehiclePolynomialTrajectory);

  vehiclePolynomialTrajectory.updateState(trajectoryPolynomial.m_finalState);

  auto finalStatePolynomial = trajectoryPolynomial.m_finalState;

  // correct lane detection
  BOOST_REQUIRE(vehicleConstantAcceleration.m_lane < 0);
  BOOST_REQUIRE(vehiclePolynomialTrajectory.m_lane < 0);

  // equality of trajectory variants
  BOOST_REQUIRE(vehicleConstantAcceleration.m_positionY == lateralChange);
  BOOST_REQUIRE(vehicleConstantAcceleration.m_velocityX == velocityChange);
  BOOST_REQUIRE(vehicleConstantAcceleration.m_velocityY == 0);
  BOOST_REQUIRE(vehicleConstantAcceleration.m_accelerationX == 0);
  BOOST_REQUIRE(vehicleConstantAcceleration.m_accelerationY == 0);

  BOOST_REQUIRE(vehiclePolynomialTrajectory.m_positionY == lateralChange);
  BOOST_REQUIRE(vehiclePolynomialTrajectory.m_velocityX == velocityChange);
  BOOST_REQUIRE_SMALL(vehiclePolynomialTrajectory.m_velocityY,
                      float(0.001));  // velocity deviation is less than 1mm/s
  BOOST_REQUIRE(vehiclePolynomialTrajectory.m_accelerationX == 0);
  BOOST_REQUIRE(vehiclePolynomialTrajectory.m_accelerationY == 0);
}

BOOST_AUTO_TEST_CASE(trajectoryGeneration) {
  // create the environment
  float lateralChange{10.0};
  float velocityChange{10.0};
  Vehicle vehicleConstantAcceleration(config::vehicle);
  Vehicle vehiclePolynomialTrajectory(config::vehicle);
  auto action = std::make_shared<Action>(Action(velocityChange, lateralChange));

  // constant Acceleration
  const auto trajectoryGeneratorC =
      TrajectoryGenerator::createTrajectoryGenerator("constantAcceleration");

  auto trajectoryConstantAcceleration =
      trajectoryGeneratorC->createTrajectory(0.0, action, vehicleConstantAcceleration);

  vehicleConstantAcceleration.updateState(trajectoryConstantAcceleration.m_finalState);

  // polynomial
  const auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  auto trajectoryPolynomial =
      trajectoryGeneratorP->createTrajectory(0.0, action, vehiclePolynomialTrajectory);

  vehiclePolynomialTrajectory.updateState(trajectoryPolynomial.m_finalState);

  BOOST_REQUIRE(vehicleConstantAcceleration.m_positionY == lateralChange);
  BOOST_REQUIRE(vehicleConstantAcceleration.m_velocityX == velocityChange);
  BOOST_REQUIRE(vehicleConstantAcceleration.m_velocityY == 0);
  BOOST_REQUIRE(vehicleConstantAcceleration.m_accelerationX == 0);
  BOOST_REQUIRE(vehicleConstantAcceleration.m_accelerationY == 0);

  BOOST_REQUIRE_SMALL(vehiclePolynomialTrajectory.m_positionY - lateralChange,
                      (float)(std::abs(lateralChange) * 10e-06));
  BOOST_REQUIRE(vehiclePolynomialTrajectory.m_velocityX == velocityChange);
  BOOST_REQUIRE_SMALL(vehiclePolynomialTrajectory.m_velocityY,
                      float(0.001));  // velocity deviation is less than 1mm/s
  BOOST_REQUIRE(vehiclePolynomialTrajectory.m_accelerationX == 0);
  BOOST_REQUIRE_SMALL(vehiclePolynomialTrajectory.m_accelerationY,
                      float(0.001));  // acceleration deviation is less than 1mm/s^2
}

BOOST_AUTO_TEST_SUITE_END()
