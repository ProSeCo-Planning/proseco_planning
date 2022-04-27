/**
 * @file test_node.cpp
 * @brief This file defines the test cases for the node.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <memory>
#include <vector>

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/desire.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/collision_checker/collisionChecker.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/node.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"
#include "proseco_planning/util/alias.h"

using namespace proseco_planning;

struct NodeFixture {
  std::vector<Agent> agents;
  NodeFixture() {
    Config::create(config::scenarioSimple, config::optionsSimple);
    config::Agent defaultAgent{0,
                               false,
                               0.5,
                               config::Desire{25.0, 0, 0, 0},
                               config::vehicle,
                               config::terminalCondition,
                               config::actionSpace,
                               config::costModel};
    agents.emplace_back(defaultAgent);
  }

  ~NodeFixture() { Config::get()->reset(); }
};

BOOST_FIXTURE_TEST_SUITE(node, NodeFixture)

BOOST_AUTO_TEST_CASE(terminality) {
  // add second agent for this test case
  agents.push_back(agents[0]);

  // set values
  agents[0].m_vehicle.m_velocityX      = 25;
  agents[0].m_vehicle.m_positionY      = 1.75;
  agents[1].m_id                       = 1;
  agents[1].m_vehicle.m_positionX      = 10;
  agents[1].m_vehicle.m_positionY      = 1.75;
  agents[1].m_vehicle.m_velocityX      = 20;
  agents[1].m_desire.m_desiredLane     = 0;
  agents[1].m_desire.m_desiredVelocity = 20;

  auto node = std::make_unique<Node>(agents);
  node->checkTerminality();
  BOOST_REQUIRE(node->m_terminal);
}

BOOST_AUTO_TEST_CASE(validity) {
  // Create Collision Checker
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  // Create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  float lateralChange = sOpt().road.lane_width;

  Action changeLeft = Action(ActionClass::CHANGE_LEFT, 0.0, lateralChange);

  agents[0].m_vehicle.m_positionX = 0;
  agents[0].m_vehicle.m_velocityX = 15;
  agents[0].m_vehicle.setLane(0);

  auto node = std::make_unique<Node>(agents);

  ActionSet currentAction;
  currentAction.push_back(std::make_shared<Action>(changeLeft));
  node->executeActions(currentAction, *collisionChecker, *trajectoryGeneratorP, false);
  node->executeActions(currentAction, *collisionChecker, *trajectoryGeneratorP, false);

  BOOST_REQUIRE(node->m_invalid);
}

BOOST_AUTO_TEST_SUITE_END()
