/**
 * @file test_collisionChecker.cpp
 * @brief Ths file defines the test cases for the collision checker.
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
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"

using namespace proseco_planning;

struct CollisionCheckerCircleApproximationFixture {
  CollisionCheckerCircleApproximationFixture() {
    Config::create(config::scenarioSimple, config::optionsSimple);
  }

  ~CollisionCheckerCircleApproximationFixture() { Config::get()->reset(); }
};

BOOST_FIXTURE_TEST_SUITE(collisionCheckerCircleApproximation,
                         CollisionCheckerCircleApproximationFixture)

BOOST_AUTO_TEST_CASE(overlappingCircleApproximation) {
  auto action = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");

  // create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  agents[1].m_vehicle.m_positionX = 3;
  agents[1].m_vehicle.m_positionY = 1;

  agents[0].setAction(action, *trajectoryGeneratorP);
  agents[1].setAction(action, *trajectoryGeneratorP);

  BOOST_REQUIRE(collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                            agents[1].m_vehicle, agents[1].m_trajectory) &&
                collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                            agents[0].m_vehicle, agents[0].m_trajectory));
}

BOOST_AUTO_TEST_CASE(no_velocity_differenceCircleApproximation) {
  auto action = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");

  // create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  agents[0].m_vehicle.m_velocityX = 10;
  agents[1].m_vehicle.m_positionX = 10;
  agents[1].m_vehicle.m_velocityX = 10;

  agents[0].setAction(action, *trajectoryGeneratorP);
  agents[1].setAction(action, *trajectoryGeneratorP);

  BOOST_REQUIRE(!collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                             agents[1].m_vehicle, agents[1].m_trajectory) &&
                !collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                             agents[0].m_vehicle, agents[0].m_trajectory));
}

BOOST_AUTO_TEST_CASE(constant_velocity_differenceCircleApproximation) {
  auto action = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  // Create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  agents[0].m_vehicle.m_velocityX = 15;
  agents[1].m_vehicle.m_positionX = 10;

  agents[0].setAction(action, *trajectoryGeneratorP);
  agents[1].setAction(action, *trajectoryGeneratorP);

  BOOST_REQUIRE(collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                            agents[1].m_vehicle, agents[1].m_trajectory) &&
                collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                            agents[0].m_vehicle, agents[0].m_trajectory));
}

BOOST_AUTO_TEST_CASE(constant_acceleration_differenceCircleApproximation) {
  auto doNothing  = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  auto accelerate = std::make_shared<Action>(Action(ActionClass::ACCELERATE, 5, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  // Create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  agents[0].m_vehicle.m_velocityX = 10;
  agents[1].m_vehicle.m_positionX = 5;
  agents[1].m_vehicle.m_velocityX = 10;

  agents[0].setAction(accelerate, *trajectoryGeneratorP);
  agents[1].setAction(doNothing, *trajectoryGeneratorP);

  BOOST_REQUIRE(collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                            agents[1].m_vehicle, agents[1].m_trajectory) &&
                collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                            agents[0].m_vehicle, agents[0].m_trajectory));
}

BOOST_AUTO_TEST_CASE(constant_acceleration_difference_and_lane_changeCircleApproximation) {
  auto changeLeft = std::make_shared<Action>(Action(ActionClass::CHANGE_LEFT, 0, 2));
  auto accelerate = std::make_shared<Action>(Action(ActionClass::ACCELERATE, 2, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  // Create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  agents[0].m_vehicle.m_velocityX = 20;

  agents[1].m_vehicle.m_positionX = 10;
  agents[1].m_vehicle.m_velocityX = 15;

  agents[0].setAction(changeLeft, *trajectoryGeneratorP);
  agents[1].setAction(accelerate, *trajectoryGeneratorP);

  BOOST_REQUIRE(!collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                             agents[1].m_vehicle, agents[1].m_trajectory) &&
                !collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                             agents[0].m_vehicle, agents[0].m_trajectory));
}

BOOST_AUTO_TEST_CASE(collision_avoidanceCircleApproximation) {
  auto changeLeft = std::make_shared<Action>(Action(ActionClass::CHANGE_LEFT, 0, 1));
  auto accelerate = std::make_shared<Action>(Action(ActionClass::ACCELERATE, 2, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  // Create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  agents[0].m_vehicle.m_velocityX = 21.95;
  agents[0].m_vehicle.m_positionX = 130.9;

  agents[1].m_vehicle.m_positionX = 147.8;
  agents[1].m_vehicle.m_velocityX = 15;

  agents[0].setAction(changeLeft, *trajectoryGeneratorP);
  agents[1].setAction(accelerate, *trajectoryGeneratorP);

  BOOST_REQUIRE(!collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                             agents[1].m_vehicle, agents[1].m_trajectory) &&
                !collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                             agents[0].m_vehicle, agents[0].m_trajectory));
}

BOOST_AUTO_TEST_CASE(run_towards_each_otherCircleApproximation) {
  auto doNothing   = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  auto accelerate  = std::make_shared<Action>(Action(ActionClass::ACCELERATE, 1, 0));
  auto deccelerate = std::make_shared<Action>(Action(ActionClass::DECELERATE, -1, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  // Create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  agents[0].m_vehicle.m_velocityX = 19;
  agents[0].m_vehicle.m_positionX = 84;

  agents[1].m_vehicle.m_velocityX = 16;
  agents[1].m_vehicle.m_positionX = 61;

  agents[2].m_vehicle.m_velocityX = -15;
  agents[2].m_vehicle.m_positionX = 110;

  agents[0].setAction(accelerate, *trajectoryGeneratorP);
  agents[1].setAction(doNothing, *trajectoryGeneratorP);
  agents[2].setAction(deccelerate, *trajectoryGeneratorP);

  BOOST_REQUIRE(collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                            agents[2].m_vehicle, agents[2].m_trajectory) &&
                collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                            agents[2].m_vehicle, agents[2].m_trajectory) &&
                collisionChecker->collision(agents[2].m_vehicle, agents[2].m_trajectory,
                                            agents[0].m_vehicle, agents[0].m_trajectory) &&
                collisionChecker->collision(agents[2].m_vehicle, agents[2].m_trajectory,
                                            agents[1].m_vehicle, agents[1].m_trajectory));
}

BOOST_AUTO_TEST_CASE(stationaryAgentCircleApproximation) {
  auto doNothing = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  // Create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  // define a stationary agent
  agents[0].m_vehicle.m_velocityX = 0;
  agents[0].m_vehicle.m_positionX = 50;
  agents[0].m_vehicle.m_positionY = 1;
  agents[0].m_vehicle.m_width     = 2;
  agents[0].m_vehicle.m_length    = 50;

  agents[1].m_vehicle.m_velocityX = 15;
  agents[1].m_vehicle.m_positionX = 40;
  agents[1].m_vehicle.m_positionY = 1.5;

  agents[0].setAction(doNothing, *trajectoryGeneratorP);
  agents[1].setAction(doNothing, *trajectoryGeneratorP);

  BOOST_REQUIRE(collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                            agents[1].m_vehicle, agents[1].m_trajectory) &&
                collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                            agents[0].m_vehicle, agents[0].m_trajectory));
}

BOOST_AUTO_TEST_CASE(collision_obstacle_CircleApproximation) {
  auto action = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  // Create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  agents[1].m_vehicle.m_positionX = 3;
  agents[1].m_vehicle.m_positionY = 1;

  agents[0].setAction(action, *trajectoryGeneratorP);
  agents[1].setAction(action, *trajectoryGeneratorP);

  BOOST_REQUIRE(collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                            agents[1].m_vehicle, agents[1].m_trajectory) &&
                collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                            agents[0].m_vehicle, agents[0].m_trajectory));
}

BOOST_AUTO_TEST_CASE(differentHeadingConstantVelocity_CircleApproximation) {
  auto changeLeft  = std::make_shared<Action>(Action(0, 3.5));
  auto changeRight = std::make_shared<Action>(Action(0, -3.5));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  {
    // Create temporary derived class pointer to set members for debugging
    auto temp        = static_cast<CollisionCheckerCircleApproximation*>(collisionChecker.get());
    temp->m_exporter = true;
    temp->m_fileName = "debugCircleApproximation";
  }
  // create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  agents[0].m_vehicle.m_positionX = 0;
  agents[0].m_vehicle.m_velocityX = 15;
  agents[0].m_vehicle.m_heading   = 0.0;

  agents[1].m_vehicle.m_positionX = 25;
  agents[1].m_vehicle.m_velocityX = -15;

  agents[0].setAction(changeLeft, *trajectoryGeneratorP);
  agents[1].setAction(changeRight, *trajectoryGeneratorP);

  //   due to coarse approximation: not a true collision
  BOOST_REQUIRE(!collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                             agents[1].m_vehicle, agents[1].m_trajectory) &&
                !collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                             agents[0].m_vehicle, agents[0].m_trajectory));
}

BOOST_AUTO_TEST_CASE(differentHeadingObstacles_CircleApproximation) {
  auto changeLeft = std::make_shared<Action>(Action(0, 3.5));
  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  // Create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  agents[0].m_vehicle.m_positionX = 10;
  agents[0].m_vehicle.m_velocityX = 15;
  agents[0].m_vehicle.m_heading   = 0.0;

  Vehicle obstacle(config::vehicle);
  obstacle.m_positionX = 30;
  obstacle.m_positionY = 1.75;
  obstacle.m_heading   = 1.5;

  agents[0].setAction(changeLeft, *trajectoryGeneratorP);

  BOOST_REQUIRE(collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory, obstacle));
}

BOOST_AUTO_TEST_CASE(debugCircleApproximation) {
  auto action0 = std::make_shared<Action>(Action(0.0, 0.0));
  auto action1 = std::make_shared<Action>(Action(0.0, 0.0));

  std::vector<Agent> agents;
  for (const auto& agent : sOpt().agents) {
    agents.emplace_back(agent);
  }
  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");
  // Create TrajectoryGenerator
  auto trajectoryGeneratorP = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  // define a stationary agent
  agents[0].m_vehicle.m_positionX     = 76;
  agents[0].m_vehicle.m_velocityX     = -15;
  agents[0].m_vehicle.m_accelerationX = 0;

  agents[0].m_vehicle.m_positionY     = 5.25;
  agents[0].m_vehicle.m_velocityY     = 0;
  agents[0].m_vehicle.m_accelerationY = 0;

  agents[0].m_vehicle.m_heading = -3.14159;
  agents[0].m_vehicle.m_width   = 2;
  agents[0].m_vehicle.m_length  = 4;

  agents[1].m_vehicle.m_positionX     = 50;
  agents[1].m_vehicle.m_velocityX     = 0;
  agents[1].m_vehicle.m_accelerationX = 0;

  agents[1].m_vehicle.m_positionY     = 0;
  agents[1].m_vehicle.m_velocityY     = 0;
  agents[1].m_vehicle.m_accelerationY = 0;

  agents[1].m_vehicle.m_heading = 1.57;
  agents[1].m_vehicle.m_width   = 2;
  agents[1].m_vehicle.m_length  = 3.5;

  agents[0].setAction(action0, *trajectoryGeneratorP);
  agents[1].setAction(action1, *trajectoryGeneratorP);

  // Create obstacle instance (Vehicle instance)
  Vehicle obstacle(config::vehicle);

  // Modify the obstacle as specified
  obstacle.m_positionX = 50;
  obstacle.m_positionY = 0.0;
  obstacle.m_heading   = 1.57;
  obstacle.m_length    = 3.5;
  obstacle.m_width     = 2.0;

  BOOST_REQUIRE(collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory, obstacle));
  BOOST_REQUIRE(collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                            agents[1].m_vehicle, agents[1].m_trajectory) &&
                collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                            agents[0].m_vehicle, agents[0].m_trajectory));
  BOOST_REQUIRE(collisionChecker->collision(agents[0].m_vehicle, agents[0].m_trajectory,
                                            agents[1].m_vehicle, agents[1].m_trajectory) &&
                collisionChecker->collision(agents[1].m_vehicle, agents[1].m_trajectory,
                                            agents[0].m_vehicle, agents[0].m_trajectory));
}

BOOST_AUTO_TEST_CASE(bug_fix_sc06) {
  /**
   * Backport of a case where the approximation circle collision checker missed a collision.
   **/
  namespace pp = proseco_planning;
  pp::config::Road road(false, 3, 3.5, 0);

  auto cfg = Config::create(config::scenarioSimple, config::optionsSimple);

  std::vector<pp::config::Obstacle> obstacles;
  obstacles.emplace_back(0, false, 120.0f, 1.75f, 0.0f, 10.0f, 3.0f, 0, 0, 0, 0, 0);
  obstacles.emplace_back(1, false, 135.0f, 1.75f, 0.0f, 10.0f, 3.0f, 0, 0, 0, 0, 0);
  obstacles.emplace_back(2, false, 150.0f, 1.75f, 0.0f, 10.0f, 3.0f, 0, 0, 0, 0, 0);

  std::vector<Agent> agents;
  for (const auto& agent : cfg->scenario.agents) {
    agents.emplace_back(agent);
  }
  auto agent = agents[0];
  agent.m_vehicle.setLane(1);
  agent.m_vehicle.m_positionX     = 154.22972106933594f;
  agent.m_vehicle.m_positionY     = 4.5488176345825195f;
  agent.m_vehicle.m_velocityX     = 10.390761375427246f;
  agent.m_vehicle.m_velocityY     = -3.039308547973633f;
  agent.m_vehicle.m_accelerationX = -0.6920642852783203f;
  agent.m_vehicle.m_accelerationY = -5.150600433349609f;
  agent.m_vehicle.m_heading       = -0.284562885761261f;
  agent.m_vehicle.m_wheelBase     = (2.4f);

  auto trajectoryGenerator = pp::TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  auto action = std::make_shared<pp::Action>(-1.3754549026489258f, -3.3350813388824463f);
  agent.setAction(action, *trajectoryGenerator);

  auto collisionChecker = CollisionChecker::createCollisionChecker("circleApproximation");

  BOOST_CHECK(!collisionChecker->collision(agent.m_vehicle, agent.m_trajectory, obstacles[0]));
  BOOST_CHECK(!collisionChecker->collision(agent.m_vehicle, agent.m_trajectory, obstacles[1]));
  BOOST_CHECK(collisionChecker->collision(agent.m_vehicle, agent.m_trajectory, obstacles[2]));
}

BOOST_AUTO_TEST_SUITE_END()
