#include "proseco_planning/exporters/jsonExporter.h"

#include <cstdlib>
#include <iterator>
#include <map>
#include <memory>

#include "nlohmann/json.hpp"
#include "proseco_planning/action/action.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/desire.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/collision_checker/collisionChecker.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/exporters/exporter.h"
#include "proseco_planning/node.h"
#include "proseco_planning/trajectory/trajectory.h"
#include "proseco_planning/trajectory/trajectorygenerator.h"
#include "proseco_planning/util/json.h"
#include "proseco_planning/util/utilities.h"

namespace proseco_planning {

/**
 * @brief Constructs a new JSONExporter object.
 *
 * @param outputPath The path to the output folder.
 */
JSONExporter::JSONExporter(const std::string& outputPath) : Exporter{outputPath}, m_ticks{0} {
  for (size_t i{}; i < std::size(m_data); ++i) {
    m_data[i] = sOpt().toJSON();
    // initialize trajectory arrays for all agents
    for (auto& agent : m_data[i]["agents"]) {
      agent["trajectory"] = json::array();
    }
  }
}

/**
 * @brief Writes the trajectory data for a single MCTS step.
 *
 * @param node The first node representing the beginning of the trajectory planning process.
 * @param actionSet The current action.
 * @param step Number of steps been taken.
 */
void JSONExporter::exportTrajectory(const Node* const node, const ActionSet& actionSet,
                                    const int step) {
  addStep(node, actionSet, step, ExportType::EXPORT_TRAJECTORY, 0);
}

/**
 * @brief Writes a single step to JSON.
 *
 * @param node The first node representing the beginning of the trajectory planning process.
 * @param actionSet The currently executed action.
 * @param step The number of steps been taken.
 * @param exportType The type of the export.
 * @param singleShotOffset The offset for time series: constant for export of single shot plan.
 */
void JSONExporter::addStep(const Node* const node, const ActionSet& actionSet, const int step,
                           const ExportType exportType, const float singleShotOffset) {
  // counter for single shot ticks, needs to always get declared
  unsigned int tickSingleShot{};

  // Set the actionFraction parameter to true as we only want to export the trajectory that has been
  // executed in the environment
  Trajectory::useActionFraction = true;
  for (size_t i{}; i <= node->m_agents[0].m_trajectory.getFractionIndex(); ++i) {
    // attention if time is updated within trajectory generator it has to be adopted here
    for (size_t agentIdx{}; agentIdx < node->m_agents.size(); ++agentIdx) {
      json trajectoryInfo = node->m_agents[agentIdx].trajectoryStepToJSON(i);

      // add information about the currently executed action for the agent
      trajectoryInfo["action"] = actionSet[agentIdx];

      trajectoryInfo["step"] = step;
      // add the correct tick  count for each export type
      trajectoryInfo["tick"] =
          (exportType == ExportType::EXPORT_SINGLESHOTPLAN) ? (tickSingleShot) : (m_ticks);
      trajectoryInfo["time"] = node->m_agents[0].m_trajectory.m_time[i] +
                               cOpt().policy_options.policy_enhancements.action_execution_fraction *
                                   (static_cast<float>(step)) * cOpt().action_duration +
                               singleShotOffset;

      // append the trajectory information to the correct member
      m_data[exportType]["agents"][agentIdx]["trajectory"].push_back(trajectoryInfo);
    }
    // increment the correct tick count
    switch (exportType) {
      case ExportType::EXPORT_SINGLESHOTPLAN: {
        ++tickSingleShot;
        break;
      }
      case ExportType::EXPORT_TRAJECTORY: {
        ++m_ticks;
        break;
      }
      default: {
        break;
      }
    }
  }
}

/**
 * @brief Writes trajectory and IRL-specific data for a single MCTS step to JSON.
 *
 * @param node The node for which the data is to be extracted.
 * @param actionSet The current action set.
 * @param step The current step.
 */
void JSONExporter::exportIrlTrajectory(const Node* const node, const ActionSet& actionSet,
                                       const int step) {
  for (size_t agentIdx{}; agentIdx < node->m_agents.size(); ++agentIdx) {
    auto agent  = node->m_agents[agentIdx];
    auto action = actionSet[agentIdx];

    json trajectoryDic;

    json stateDic;
    stateDic["posX"]       = agent.m_vehicle.m_positionX;
    stateDic["posY"]       = agent.m_vehicle.m_positionY;
    stateDic["velX"]       = agent.m_vehicle.m_velocityX;
    stateDic["velY"]       = agent.m_vehicle.m_velocityY;
    stateDic["accX"]       = agent.m_vehicle.m_accelerationX;
    stateDic["accY"]       = agent.m_vehicle.m_accelerationY;
    trajectoryDic["state"] = stateDic;

    json actionDic;
    actionDic["deltaY"]              = action->m_lateralChange;
    actionDic["deltaVx"]             = action->m_velocityChange;
    actionDic["likelihoodY"]         = action->noise.m_likelihoodY;
    actionDic["likelihoodVx"]        = action->noise.m_likelihoodVx;
    actionDic["muY"]                 = action->noise.m_muY;
    actionDic["muVx"]                = action->noise.m_muVx;
    actionDic["sigmaY"]              = action->noise.m_sigmaY;
    actionDic["sigmaVx"]             = action->noise.m_sigmaVx;
    actionDic["selectionLikelihood"] = action->m_selectionLikelihood;
    actionDic["selectionWeights"]    = action->m_selectionWeights;
    trajectoryDic["action"]          = actionDic;

    json featuresDic;
    featuresDic["diff_vel_vel_des"] =
        agent.m_desire.m_desiredVelocity - agent.m_vehicle.m_velocityX;
    featuresDic["desired_vel"]   = agent.m_desire.m_desiredVelocity;
    featuresDic["abs_lane_diff"] = std::abs(agent.m_vehicle.m_lane - agent.m_desire.m_desiredLane);
    featuresDic["desiredLane"]   = agent.m_desire.m_desiredLane;
    featuresDic["diff_des_lane_cent"]  = agent.m_vehicle.getDistanceToLaneCenter();
    featuresDic["laneChanged"]         = agent.m_trajectory.m_laneChange;
    featuresDic["invalidAction"]       = agent.m_trajectory.m_invalidAction;
    featuresDic["accX"]                = agent.m_trajectory.m_cumSquaredAccelerationLon;
    featuresDic["accY"]                = agent.m_trajectory.m_cumSquaredAccelerationLat;
    featuresDic["averageAbsoluteAccY"] = agent.m_trajectory.m_averageAbsoluteAcceleration;
    featuresDic["collided"]            = agent.m_collision;
    featuresDic["invalidState"]        = agent.m_invalid;
    trajectoryDic["features"]          = featuresDic;

    m_data[ExportType::EXPORT_IRL_TRAJECTORY]["agents"][agentIdx]["trajectory"].push_back(
        trajectoryDic);
  }
}

/**
 * @brief Writes the single shot plan to disk.
 *
 * @param step The number of steps been taken.
 * @param exportType The type of the export.
 */
void JSONExporter::writeData(const int step, const ExportType exportType) {
  std::string exportPath;
  json data = m_data[exportType];

  // write the correct json file to disk
  switch (exportType) {
    case ExportType::EXPORT_SINGLESHOTPLAN: {
      exportPath = m_path + "/" + m_fileNames[exportType] + std::to_string(step);
      break;
    }
    default: {
      exportPath = m_path + "/" + m_fileNames[exportType];
    }
  }

  util::saveJSON(exportPath, data);
}

/**
 * @brief Writes a single shot trajectory to JSON.
 * @note Single shot is planned without timeShift!
 *
 * @param node The node from which to generate a single shot trajectory.
 * @param actionPlan The plan sequence vector, where every entry represents the actions that all
 * agents took at this node.
 * @param step The current step.
 */
void JSONExporter::exportSingleShot(Node* const node, const ActionSetSequence& actionSetSequence,
                                    const int step) {
  auto collisionChecker    = CollisionChecker::createCollisionChecker("circleApproximation");
  auto trajectoryGenerator = TrajectoryGenerator::createTrajectoryGenerator("jerkOptimal");

  // offset for time series: constant for export of single shot plan
  float singleShotOffset = cOpt().policy_options.policy_enhancements.action_execution_fraction *
                           (float)step * cOpt().action_duration;

  for (size_t i = 0; i < actionSetSequence.size(); ++i) {
    // advance node by one action
    node->executeActions(actionSetSequence[i], *collisionChecker, *trajectoryGenerator, false);

    // within the tree search (=> within the single shot plan) no time shift is applied
    addStep(node, actionSetSequence[i], step + i, ExportType::EXPORT_SINGLESHOTPLAN,
            singleShotOffset);
  }

  // Write the single shot plan to disk
  writeData(step, ExportType::EXPORT_SINGLESHOTPLAN);

  // Clear the trajectory arrays for each agent
  for (auto& agent : m_data[ExportType::EXPORT_SINGLESHOTPLAN]["agents"]) {
    agent["trajectory"].clear();
  }
}
}  // namespace proseco_planning
