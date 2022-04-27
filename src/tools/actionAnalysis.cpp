/**
 * @file actionAnalysis.cpp
 * @brief This tool generates a .json file of the actions of an agent over a rectangular action
 * space.
 * @details Each action is annotated with the corresponding action class as well as the cost for
 * executing the action.
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <algorithm>
#include <memory>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/action/actionSpace.h"
#include "proseco_planning/action/actionSpaceRectangle.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/cost_model/costModel.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/math/mathlib.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"
#include "proseco_planning/util/utilities.h"

using namespace proseco_planning;

int main(int argc, char* argv[]) {
  util::createConfig(std::string(argv[1]), std::string(argv[2]));

  Agent agent{sOpt().agents[0]};

  // create cost model
  auto costModel = CostModel::createCostModel(sOpt().agents[0].cost_model);
  // create trajectory generator
  auto trajectoryGenerator = TrajectoryGenerator::createTrajectoryGenerator(cOpt().trajectory_type);

  // Create max values of the action space
  auto actionSpace = dynamic_cast<ActionSpaceRectangle*>(agent.m_actionSpace.get());
  float maxChangeLateral{actionSpace->m_config.max_lateral_change};
  float maxChangeVelocity{actionSpace->m_config.max_lateral_change};

  // Set number of points to be evaluated
  size_t numberDataPoints{21};
  auto changeLateral  = math::linspace(-maxChangeLateral, maxChangeLateral, numberDataPoints);
  auto changeVelocity = math::linspace(-maxChangeVelocity, maxChangeVelocity, numberDataPoints);

  json jActionClasses;
  jActionClasses["lane_width"] = sOpt().road.lane_width;
  jActionClasses["position_y"] = agent.m_vehicle.m_positionY;
  jActionClasses["position_x"] = agent.m_vehicle.m_positionX;
  jActionClasses["cost_model"] = sOpt().agents[0].cost_model.name;
  jActionClasses["actions"]    = json::array();

  for (const auto& d_lat_y : changeLateral) {
    for (const auto& d_lon_v : changeVelocity) {
      auto action = std::make_shared<Action>(d_lon_v, d_lat_y);
      action->updateActionClass(*actionSpace, agent.m_vehicle);
      agent.setAction(action, *trajectoryGenerator);
      const auto [minAcceleration, maxAcceleration] =
          std::minmax_element(begin(agent.m_trajectory.m_totalAcceleration),
                              end(agent.m_trajectory.m_totalAcceleration));
      const auto [minVelocity, maxVelocity] = std::minmax_element(
          begin(agent.m_trajectory.m_totalVelocity), end(agent.m_trajectory.m_totalVelocity));
      const auto [minSteeringAngle, maxSteeringAngle] = std::minmax_element(
          begin(agent.m_trajectory.m_steeringAngle), end(agent.m_trajectory.m_steeringAngle));
      json jAction{{"d_lon_v", d_lon_v},
                   {"d_lat_y", d_lat_y},
                   {"class", ActionSpace::ACTION_CLASS_NAME_MAP.at(action->m_actionClass)},
                   {"cost_acc_x", costModel->costAccelerationX(agent.m_trajectory)},
                   {"cost_acc_y", costModel->costAccelerationY(agent.m_trajectory)},
                   {"cost_change_lane", costModel->costLaneChange(agent.m_trajectory)},
                   {"cost_total", agent.m_actionCost},
                   {"minTotalAcceleration", *minAcceleration},
                   {"maxTotalAcceleration", *maxAcceleration},
                   {"minTotalVelocity", *minVelocity},
                   {"maxTotalVelocity", *maxVelocity},
                   {"maxAbsSteeringAngle",
                    std::max(std::abs(*minSteeringAngle), std::abs(*maxSteeringAngle))},
                   {"invalid", agent.m_trajectory.m_invalidAction}};

      jActionClasses["actions"].push_back(jAction);
    }
  }

  // Sort the output by action class
  // std::sort(jActionClasses["actions"].begin(), jActionClasses["actions"].end(),
  //           [](const json& j1, const json& j2) { return j1.at("class") <= j2.at("class"); });

  util::saveJSON(std::string(argv[3]) + "/action_analysis", jActionClasses);
}
