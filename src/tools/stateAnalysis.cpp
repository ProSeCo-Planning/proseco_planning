/**
 * @file stateAnalysis.cpp
 * @brief This tool generates a .json file with the states resulting from the respective actions of
 * the agent.
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <cassert>
#include <cstddef>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/action/actionSpaceRectangle.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/collision_checker/collisionChecker.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/node.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"
#include "proseco_planning/util/alias.h"
#include "proseco_planning/util/json.h"
#include "proseco_planning/util/utilities.h"

using namespace proseco_planning;

int main(int argc, char* argv[]) {
  util::createConfig(std::string(argv[1]), std::string(argv[2]));

  Agent agent{sOpt().agents[0]};

  // create collision checker
  auto collisionChecker = CollisionChecker::createCollisionChecker(cOpt().collision_checker);
  // create trajectory generator
  auto trajectoryGeneratorP =
      TrajectoryGenerator::createTrajectoryGenerator(cOpt().trajectory_type);

  // create max values of the action space
  auto actionSpace = dynamic_cast<ActionSpaceRectangle*>(agent.m_actionSpace.get());
  float maxChangeLateral{actionSpace->m_config.max_lateral_change};
  float maxChangeVelocity{actionSpace->m_config.max_velocity_change};

  size_t numberDataPoints{21};  // 100;
  auto changeLateral  = math::linspace(-maxChangeLateral, maxChangeLateral, numberDataPoints);
  auto changeVelocity = math::linspace(-maxChangeVelocity, maxChangeVelocity, numberDataPoints);

  // create agent vector to initialize the root node
  std::vector<Agent> agents{agent};
  // create the root node
  auto rootNode = std::make_unique<Node>(agents);

  // Calculate action cost
  ActionSet actionSet{};
  for (const auto& d_lat_y : changeLateral) {
    for (const auto& d_lon_v : changeVelocity) {
      auto action = std::make_shared<Action>(d_lon_v, d_lat_y);
      action->updateActionClass(*actionSpace, rootNode->m_agents[0].m_vehicle);
      rootNode->m_agents[0].addActionToMaps(action);

      actionSet.clear();
      actionSet.push_back(action);
      // create child node using the action set
      auto child = rootNode->addChild(actionSet);
      // execute the action used for reaching the child
      child->executeActions(actionSet, *collisionChecker, *trajectoryGeneratorP, false);
    }
  }

  util::saveJSON(std::string(argv[3]) + "/state_analysis", json(rootNode.get()));
}
