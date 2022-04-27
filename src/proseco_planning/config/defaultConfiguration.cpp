#include "proseco_planning/config/defaultConfiguration.h"

#include <eigen3/Eigen/Core>
#include <string>

#include "proseco_planning/config/configuration.h"

namespace proseco_planning::config {

config::Road road = config::Road(false, 2, 3.5f, 0);

config::Obstacle obstacle               = config::Obstacle(0, false, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0);
std::vector<config::Obstacle> obstacles = {obstacle};

config::TerminalCondition terminalCondition = config::TerminalCondition(0, 0, "bigger", "equal");
config::Desire desire                       = config::Desire(0, 0, 0, 0);
config::Vehicle vehicle =
    config::Vehicle(false, 0, 0, 0, 0, 0, 4, 2, 0, 0, 0, 0, 0, 0, 0, 2.4f, 0.22f, 36.0f, 9.81f);
config::ActionSpaceRectangle actionSpaceRectangle{5.0f, 5.0f, 5.0f / 3};
config::ActionSpace::variant_t actionSpace = actionSpaceRectangle;

std::vector<float> v1(50.0f, 1.0f);
Eigen::MatrixXd w1 = config::CostModel::convertVectorToEigenMatrix(v1, 10, 5);
std::vector<float> v2(5.0f, 1.0f);
Eigen::MatrixXd w2          = config::CostModel::convertVectorToEigenMatrix(v2, 5, 1);
config::CostModel costModel = config::CostModel("costExponential", 1, 1, 1, 1, 1, 1, 1, 1, 1, -10,
                                                0, 1, 1, 1, 1, 1, 1, 1, w1, w2);
config::Agent agent0 =
    config::Agent(0, false, 0.5, desire, vehicle, terminalCondition, actionSpace, costModel);
config::Agent agent1 =
    config::Agent(1, false, 0.5, desire, vehicle, terminalCondition, actionSpace, costModel);
config::Agent agent2 =
    config::Agent(2, false, 0.5, desire, vehicle, terminalCondition, actionSpace, costModel);
std::vector<config::Agent> agents = {agent0, agent1, agent2};

Scenario scenarioSimple = Scenario("default", road, agents, obstacles);

config::OutputOptions oOptions =
    OutputOptions(config::exportFormat::NONE, std::vector<std::string>(), "");

config::SimilarityUpdate simUpdate                    = SimilarityUpdate(false, 1.0);
config::SearchGuide searchGuide                       = SearchGuide(0, "random");
config::MoveGroupingCriteriaPW moveGroupingCriteriaPW = MoveGroupingCriteriaPW(false, 1.0, 1.0);
config::MoveGrouping moveGrouping =
    MoveGrouping(false, 12.0f, moveGroupingCriteriaPW, false, false);
config::ProgressiveWidening progressiveWidening = ProgressiveWidening(2, 0.5, 25);
config::PolicyEnhancements policyEnhancements =
    PolicyEnhancements(simUpdate, searchGuide, moveGrouping, progressiveWidening, 1.0, 100.0);
config::PolicyOptions policyOptions = PolicyOptions("UCTProgressiveWidening", "UCT", "moderate",
                                                    "UCT", "maxActionValue", policyEnhancements);
config::ParallelizationOptions parallelizationOptions =
    ParallelizationOptions(1, 1, true, 1.0f, "max");
config::Noise noise = Noise(false, 0, 0.15);
config::ComputeOptions cOptions =
    ComputeOptions(0, 100, 15.0f, 13, 12.0f, 5, 10, 0.7f, 0.1f, 2.0f, "circleApproximation", 0,
                   "scenario", policyOptions, parallelizationOptions, "jerkOptimal", 4.0f, noise,
                   ActionNoise(false, 0, 0, 0, 0), false);

Options optionsSimple = Options(oOptions, cOptions);

}  // namespace proseco_planning::config
