/**
 * @file test_actionClass.cpp
 * @brief This file defines the test cases for action classes.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/action/actionSpaceRectangle.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"

using namespace proseco_planning;

struct ConfigFixture {
  ConfigFixture() { Config::create(config::scenarioSimple, config::optionsSimple); }
  ~ConfigFixture() { Config::get()->reset(); }
};

struct ActionClassFixture : ConfigFixture, ActionSpaceRectangle {
  ActionClassFixture() : ActionSpaceRectangle(config::actionSpaceRectangle) {}
};

BOOST_FIXTURE_TEST_SUITE(actionClassDetection, ActionClassFixture)

BOOST_AUTO_TEST_CASE(StateDoNothing) {
  Vehicle vehicle(config::vehicle);
  vehicle.setLane(1);
  vehicle.m_velocityX = 10;

  float lateralChange{sOpt().road.lane_width / 3.0f};
  float velocityChange{0.4f};
  Action action(velocityChange, lateralChange);

  action.updateActionClass(*this, vehicle);

  BOOST_REQUIRE(action.m_actionClass == ActionClass::DO_NOTHING);
}

BOOST_AUTO_TEST_CASE(StateChangeLeft) {
  Vehicle vehicle(config::vehicle);
  vehicle.setLane(1);
  vehicle.m_velocityX = 10;
  vehicle.m_positionY = sOpt().road.lane_width * (vehicle.m_lane + 0.7);

  float lateralChange{sOpt().road.lane_width / 3.0f};
  float velocityChange{0.5f};
  Action action(velocityChange, lateralChange);

  action.updateActionClass(*this, vehicle);

  BOOST_REQUIRE(action.m_actionClass == ActionClass::CHANGE_LEFT);
}

BOOST_AUTO_TEST_CASE(StateChangeRight) {
  Vehicle vehicle(config::vehicle);
  vehicle.setLane(1);
  vehicle.m_velocityX = 10;
  vehicle.m_positionY = sOpt().road.lane_width * ((vehicle.m_lane + 0.5) - 0.7);

  float lateralChange{-sOpt().road.lane_width / 3.0f};
  float velocityChange{0.5f};
  Action action(velocityChange, lateralChange);

  action.updateActionClass(*this, vehicle);

  BOOST_REQUIRE(action.m_actionClass == ActionClass::CHANGE_RIGHT);
}

BOOST_AUTO_TEST_CASE(StateAccelerate) {
  Vehicle vehicle(config::vehicle);
  vehicle.setLane(1);
  vehicle.m_velocityX = 10;

  float lateralChange{sOpt().road.lane_width / 3.0f};
  float velocityChange{m_config.delta_velocity * 1.2f};
  Action action(velocityChange, lateralChange);

  action.updateActionClass(*this, vehicle);

  BOOST_REQUIRE(action.m_actionClass == ActionClass::ACCELERATE);
}

BOOST_AUTO_TEST_CASE(StateDecelerate) {
  Vehicle vehicle(config::vehicle);
  vehicle.setLane(1);
  vehicle.m_velocityX = 10;

  float lateralChange{sOpt().road.lane_width / 3.0f};
  float velocityChange{-m_config.delta_velocity * 1.2f};
  Action action(velocityChange, lateralChange);

  action.updateActionClass(*this, vehicle);

  BOOST_REQUIRE(action.m_actionClass == ActionClass::DECELERATE);
}

BOOST_AUTO_TEST_CASE(StateDifferent) {
  Vehicle vehicle(config::vehicle);
  vehicle.setLane(1);
  vehicle.m_velocityX = 10;

  float lateralChange{-sOpt().road.lane_width / 3.0f};
  float velocityChange{-m_config.delta_velocity * 1.2f};
  Action action(velocityChange, lateralChange);

  action.updateActionClass(*this, vehicle);

  BOOST_REQUIRE(action.m_actionClass == ActionClass::DECELERATE);
}

BOOST_AUTO_TEST_CASE(changeLeftDetailed) {
  // ax = [-1,0,1] => dV = -2, 0, 2
  // ay = [-1.75, 0, 1.75] => dLateral = 3.5
  float laneWidth{3.5f};
  float acceleration{2.0f};
  Action doNothing(0.0, 0.0);
  Action decelerate(-acceleration, 0.0);
  Action accelerate(acceleration, 0.0);
  Action changeRightSlow(-acceleration, -laneWidth);
  Action changeRight(0.0, -laneWidth);
  Action changeRightFast(acceleration, -laneWidth);
  Action changeLeftSlow(-acceleration, laneWidth);
  Action changeLeft(0.0, laneWidth);
  Action changeLeftFast(acceleration, laneWidth);

  // create environment
  // specify vehicle state
  Vehicle vehicle(config::vehicle);
  vehicle.setLane(1);
  vehicle.m_positionX     = 15.375;
  vehicle.m_positionY     = 3.5;
  vehicle.m_velocityX     = 10;
  vehicle.m_velocityY     = 3.28125;
  vehicle.m_accelerationX = 1.5;
  vehicle.m_accelerationY = 0.0;

  doNothing.updateActionClass(*this, vehicle);
  decelerate.updateActionClass(*this, vehicle);
  accelerate.updateActionClass(*this, vehicle);
  changeRightSlow.updateActionClass(*this, vehicle);
  changeRight.updateActionClass(*this, vehicle);
  changeRightFast.updateActionClass(*this, vehicle);
  changeLeftSlow.updateActionClass(*this, vehicle);
  changeLeft.updateActionClass(*this, vehicle);
  changeLeftFast.updateActionClass(*this, vehicle);

  // Check state dependent action class assignment
  BOOST_CHECK(doNothing.m_actionClass == ActionClass::DO_NOTHING);
  BOOST_CHECK(accelerate.m_actionClass == ActionClass::ACCELERATE);
  BOOST_CHECK(decelerate.m_actionClass == ActionClass::DECELERATE);
  BOOST_CHECK(changeRightSlow.m_actionClass == ActionClass::CHANGE_RIGHT_SLOW);
  BOOST_CHECK(changeRight.m_actionClass == ActionClass::CHANGE_RIGHT);
  BOOST_CHECK(changeRightFast.m_actionClass == ActionClass::CHANGE_RIGHT_FAST);
  BOOST_CHECK(changeLeftSlow.m_actionClass == ActionClass::CHANGE_LEFT_SLOW);
  BOOST_CHECK(changeLeft.m_actionClass == ActionClass::CHANGE_LEFT);
  BOOST_CHECK(changeLeftFast.m_actionClass == ActionClass::CHANGE_LEFT_FAST);
}
BOOST_AUTO_TEST_SUITE_END()