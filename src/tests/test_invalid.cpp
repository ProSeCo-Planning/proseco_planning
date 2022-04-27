/**
 * @file test_invalid.cpp
 * @brief This file defines the test cases for the invalid checks.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <memory>
#include <string>
#include <vector>

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/collision_checker/collisionChecker.h"
#include "proseco_planning/collision_checker/collisionCheckerCircleApproximation.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/node.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"

using namespace proseco_planning;

struct ThreeLaneFixture {
  ThreeLaneFixture() {
    auto road     = config::Road(false, 3, 3.5, 0);
    auto scenario = config::Scenario("simple", road, config::agents, config::obstacles);
    Config::create(scenario, config::optionsSimple);
  }

  ~ThreeLaneFixture() { Config::get()->reset(); }
};

struct DefaultFixture {
  DefaultFixture() { Config::create(config::scenarioSimple, config::optionsSimple); }

  ~DefaultFixture() { Config::get()->reset(); }
};

BOOST_AUTO_TEST_SUITE(invalidState)

BOOST_FIXTURE_TEST_CASE(rightBoundary, ThreeLaneFixture) {
  auto doNothing = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }

  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  // create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  auto rootNode = std::make_unique<Node>(agents);

  // default vehicle dimension: LxW: 4x2
  // Without orientation change
  rootNode->m_agents[0].m_vehicle.m_positionY = 0.99;

  rootNode->m_agents[1].m_vehicle.m_positionY = 1.5;
  rootNode->m_agents[1].m_vehicle.m_heading   = -3.14 / 2.0;

  rootNode->m_agents[2].m_vehicle.m_positionY = -5.0;

  // Set action
  rootNode->m_agents[0].setAction(doNothing, *trajectoryGeneratorP);
  rootNode->m_agents[1].setAction(doNothing, *trajectoryGeneratorP);
  rootNode->m_agents[2].setAction(doNothing, *trajectoryGeneratorP);

  rootNode->checkValidity();

  // for visualizing the actions
  {
    // Create temporary derived class pointer to set members for debugging
    auto temp        = static_cast<CollisionCheckerCircleApproximation*>(collisionChecker.get());
    temp->m_exporter = true;
    temp->m_fileName = "debugCircleApproximation";
  }
  collisionChecker->collision(rootNode->m_agents[0].m_vehicle, rootNode->m_agents[0].m_trajectory,
                              rootNode->m_agents[1].m_vehicle, rootNode->m_agents[1].m_trajectory);

  BOOST_CHECK(rootNode->m_agents[0].m_invalid);
  BOOST_CHECK(rootNode->m_agents[1].m_invalid);
  BOOST_CHECK(rootNode->m_agents[2].m_invalid);
}

BOOST_FIXTURE_TEST_CASE(leftBoundary, ThreeLaneFixture) {
  auto changeLeft  = std::make_shared<Action>(Action(2.0, sOpt().road.lane_width));
  auto changeRight = std::make_shared<Action>(Action(2.0, -sOpt().road.lane_width));

  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  // create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  auto rootNode = std::make_unique<Node>(agents);

  // default vehicle dimension: LxW: 4x2
  // Without orientation change
  rootNode->m_agents[0].m_vehicle.setLane(1);
  rootNode->m_agents[0].m_vehicle.m_velocityX = 10;

  rootNode->m_agents[1].m_vehicle.setLane(1);
  rootNode->m_agents[1].m_vehicle.m_positionX = 20;
  rootNode->m_agents[1].m_vehicle.m_heading   = 0;
  rootNode->m_agents[1].m_vehicle.m_velocityX = 10;

  rootNode->m_agents[2].m_vehicle.setLane(1);
  rootNode->m_agents[2].m_vehicle.m_velocityX = 10;

  // Set action
  rootNode->m_agents[0].setAction(changeLeft, *trajectoryGeneratorP);
  rootNode->m_agents[1].setAction(changeLeft, *trajectoryGeneratorP);
  rootNode->m_agents[2].setAction(changeRight, *trajectoryGeneratorP);

  rootNode->checkValidity();
  BOOST_CHECK(!rootNode->m_agents[0].m_invalid);
  BOOST_CHECK(!rootNode->m_agents[1].m_invalid);
  BOOST_CHECK(!rootNode->m_agents[2].m_invalid);
}

BOOST_FIXTURE_TEST_CASE(laneChange, DefaultFixture) {
  auto doNothing = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  // create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  auto rootNode = std::make_unique<Node>(agents);

  // default vehicle dimension: LxW: 4x2
  // Without orientation change
  rootNode->m_agents[0].m_vehicle.m_positionY = 7.01;

  rootNode->m_agents[1].m_vehicle.m_positionY = 5.5;
  rootNode->m_agents[1].m_vehicle.m_heading   = 3.14 / 2.0;

  rootNode->m_agents[2].m_vehicle.m_positionY = 12.0;

  // Set action
  rootNode->m_agents[0].setAction(doNothing, *trajectoryGeneratorP);
  rootNode->m_agents[1].setAction(doNothing, *trajectoryGeneratorP);
  rootNode->m_agents[2].setAction(doNothing, *trajectoryGeneratorP);

  rootNode->checkValidity();

  BOOST_CHECK(rootNode->m_agents[0].m_invalid);
  BOOST_CHECK(rootNode->m_agents[1].m_invalid);
  BOOST_CHECK(rootNode->m_agents[2].m_invalid);
}

BOOST_FIXTURE_TEST_CASE(debugging_3veh, DefaultFixture) {
  auto doNothing = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  // create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  auto rootNode = std::make_unique<Node>(agents);

  // default vehicle dimension: LxW: 4x2
  // Without orientation change
  rootNode->m_agents[0].m_vehicle.m_positionY = 7.01;

  rootNode->m_agents[1].m_vehicle.m_positionY = 5.5;
  rootNode->m_agents[1].m_vehicle.m_heading   = 3.14 / 2.0;

  rootNode->m_agents[2].m_vehicle.m_positionY = 12.0;

  // Set action
  rootNode->m_agents[0].setAction(doNothing, *trajectoryGeneratorP);
  rootNode->m_agents[1].setAction(doNothing, *trajectoryGeneratorP);
  rootNode->m_agents[2].setAction(doNothing, *trajectoryGeneratorP);

  rootNode->checkValidity();

  BOOST_CHECK(rootNode->m_agents[0].m_invalid);
  BOOST_CHECK(rootNode->m_agents[1].m_invalid);
  BOOST_CHECK(rootNode->m_agents[2].m_invalid);
}

BOOST_FIXTURE_TEST_CASE(debugging, DefaultFixture) {
  auto doNothing = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  // create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  auto rootNode = std::make_unique<Node>(agents);

  // default vehicle dimension: LxW: 4x2
  // Without orientation change
  rootNode->m_agents[0].m_vehicle.m_positionX     = 42.828947;
  rootNode->m_agents[0].m_vehicle.m_positionY     = 1.972868;
  rootNode->m_agents[0].m_vehicle.m_velocityX     = -0.0575;
  rootNode->m_agents[0].m_vehicle.m_velocityY     = -0.001995;
  rootNode->m_agents[0].m_vehicle.m_accelerationX = -1.44011;
  rootNode->m_agents[0].m_vehicle.m_accelerationY = -0.015022;
  rootNode->m_agents[0].m_vehicle.m_heading       = 0.0;

  auto action1 = std::make_shared<Action>(-0.071, 0.89);

  // Set action
  rootNode->m_agents[0].setAction(action1, *trajectoryGeneratorP);
  rootNode->m_agents[1].setAction(doNothing, *trajectoryGeneratorP);
  rootNode->m_agents[2].setAction(doNothing, *trajectoryGeneratorP);

  rootNode->checkValidity();

  BOOST_CHECK(rootNode->m_agents[0].m_invalid);
}

BOOST_AUTO_TEST_SUITE_END()
