/**
 * @file trajectoryAnalysis.cpp
 * @brief This tool genereate a .json file with the trajectory of the vehicle given different
 * actions.
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <memory>
#include <string>

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionSpaceRectangle.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/trajectory/trajectory.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"
#include "proseco_planning/util/alias.h"
#include "proseco_planning/util/utilities.h"

using namespace proseco_planning;

int main(int argc, char* argv[]) {
  util::createConfig(std::string(argv[1]), std::string(argv[2]));

  auto trajectoryGenerator = TrajectoryGenerator::createTrajectoryGenerator(cOpt().trajectory_type);
  Agent agent{sOpt().agents[0]};
  ActionSet actionSet;

  bool lateralOffsetOnly = false;
  if (lateralOffsetOnly) {
    // create max values of the action space
    auto actionSpace = dynamic_cast<ActionSpaceRectangle*>(agent.m_actionSpace.get());
    float maxChangeLateral{actionSpace->m_config.max_lateral_change};
    // this number should not be larger than 10 due to the colormap being used in plotly
    size_t numberDataPoints{9};
    auto changeLateral = math::linspace(-maxChangeLateral, maxChangeLateral, numberDataPoints);

    for (const auto& d_lat_y : changeLateral) {
      actionSet.push_back(std::make_shared<Action>(0.0f, d_lat_y));
    }
  } else {
    actionSet = agent.m_actionSpace->getDetailedActions(agent.m_vehicle);
  }

  for (size_t i = 0; i < actionSet.size(); ++i) {
    agent.m_trajectory = trajectoryGenerator->createTrajectory(0.0f, actionSet[i], agent.m_vehicle);

    util::saveJSON(std::string(argv[3]) + "/trajectory_" + util::padNumber(i, 3),
                   json(agent.m_trajectory));
  }
}