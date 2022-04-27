/**
 * @file defaultConfiguration.h
 * @brief This file defines the default configuration for the execution of the ProSeCo planner,
 * mainly used for testing.
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include <vector>

#include "configuration.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/outputOptions.h"
#include "proseco_planning/config/scenarioOptions.h"

namespace proseco_planning::config {

extern config::Road road;

extern config::Obstacle obstacle;
extern std::vector<config::Obstacle> obstacles;

extern config::TerminalCondition terminalCondition;
extern config::Desire desire;
extern config::Vehicle vehicle;
extern config::ActionSpaceRectangle actionSpaceRectangle;
extern config::ActionSpace::variant_t actionSpace;
extern config::CostModel costModel;
extern config::Agent agent;
extern std::vector<config::Agent> agents;

extern Scenario scenarioSimple;

extern OutputOptions oOptions;
extern SimilarityUpdate simUpdate;
extern SearchGuide searchGuide;
extern MoveGroupingCriteriaPW moveGroupingCriteriaPW;
extern MoveGrouping moveGrouping;
extern ProgressiveWidening progressiveWidening;
extern PolicyEnhancements policyEnhancements;
extern PolicyOptions policyOptions;
extern ParallelizationOptions parallelizationOptions;
extern ComputeOptions cOptions;
extern Options optionsSimple;

}  // namespace proseco_planning::config