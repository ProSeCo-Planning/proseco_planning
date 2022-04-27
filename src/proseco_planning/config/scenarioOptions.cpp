#include "proseco_planning/config/scenarioOptions.h"

#include <map>
#include <stdexcept>

#include "nlohmann/json.hpp"
#include "proseco_planning/math/mathlib.h"

namespace proseco_planning::config {
/**
 * @brief Exports the parameters of the Obstacle object to JSON.
 *
 * @return json The parameters.
 */
json Obstacle::toJSON() const {
  json jObstacle;
  jObstacle["id"]               = id;
  jObstacle["random"]           = random;
  jObstacle["position_x"]       = position_x;
  jObstacle["position_y"]       = position_y;
  jObstacle["heading"]          = heading;
  jObstacle["length"]           = length;
  jObstacle["width"]            = width;
  jObstacle["sigma_position_x"] = sigma_position_x;
  jObstacle["sigma_position_y"] = sigma_position_y;
  jObstacle["sigma_heading"]    = sigma_heading;
  jObstacle["sigma_length"]     = sigma_length;
  jObstacle["sigma_width"]      = sigma_width;
  return jObstacle;
}

/**
 * @brief Returns a new Obstacle object created from the parameters of the JSON file.
 *
 * @param jObstacle The JSON file.
 * @return Obstacle
 */
Obstacle Obstacle::fromJSON(const json& jObstacle) {
  auto position_x = jObstacle["position_x"].get<float>();
  auto position_y = jObstacle["position_y"].get<float>();
  auto heading    = jObstacle["heading"].get<float>();
  auto length     = jObstacle["length"].get<float>();
  auto width      = jObstacle["width"].get<float>();

  if (jObstacle["random"].get<bool>()) {
    position_x = math::getNoise(position_x, jObstacle["sigma_position_x"].get<float>(), true);
    position_y = math::getNoise(position_y, jObstacle["sigma_position_y"].get<float>(), true);
    heading    = math::getNoise(heading, jObstacle["sigma_heading"].get<float>(), true);
    length     = math::getNoise(length, jObstacle["sigma_length"].get<float>(), true);
    width      = math::getNoise(width, jObstacle["sigma_width"].get<float>(), true);
  }

  return {jObstacle["id"].get<unsigned int>(),
          jObstacle["random"].get<bool>(),
          position_x,
          position_y,
          heading,
          length,
          width,
          jObstacle["sigma_position_x"].get<float>(),
          jObstacle["sigma_position_y"].get<float>(),
          jObstacle["sigma_heading"].get<float>(),
          jObstacle["sigma_length"].get<float>(),
          jObstacle["sigma_width"].get<float>()};
}

/**
 * @brief Exports the parameters of the Road object to JSON.
 *
 * @return json The parameters.
 */
json Road::toJSON() const {
  json jRoad;
  jRoad["random"]           = random;
  jRoad["number_lanes"]     = number_lanes;
  jRoad["lane_width"]       = lane_width;
  jRoad["sigma_lane_width"] = sigma_lane_width;
  return jRoad;
}

/**
 * @brief Returns a new Road object created from the parameters of the JSON file.
 *
 * @param jsonRoad The JSON file.
 * @return Road
 */
Road Road::fromJSON(const json& jsonRoad) {
  auto lane_width = jsonRoad["lane_width"].get<float>();

  if (jsonRoad["random"].get<bool>()) {
    lane_width = math::getNoise(jsonRoad["lane_width"].get<float>(),
                                jsonRoad["sigma_lane_width"].get<float>(), true);
  }

  auto road = Road(jsonRoad["random"].get<bool>(), jsonRoad["number_lanes"].get<unsigned int>(),
                   lane_width, jsonRoad["sigma_lane_width"].get<float>());
  return road;
}

/**
 * @brief Exports the parameters of the Vehicle object to JSON.
 *
 * @return json The parameters.
 */
json Vehicle::toJSON() const {
  json jVehicle;
  jVehicle["position_x"]         = position_x;
  jVehicle["position_y"]         = position_y;
  jVehicle["velocity_x"]         = velocity_x;
  jVehicle["velocity_y"]         = velocity_y;
  jVehicle["heading"]            = heading;
  jVehicle["length"]             = length;
  jVehicle["width"]              = width;
  jVehicle["sigma_position_x"]   = sigma_position_x;
  jVehicle["sigma_position_y"]   = sigma_position_y;
  jVehicle["sigma_velocity_x"]   = sigma_velocity_x;
  jVehicle["sigma_velocity_y"]   = sigma_velocity_y;
  jVehicle["sigma_heading"]      = sigma_heading;
  jVehicle["sigma_length"]       = sigma_length;
  jVehicle["sigma_width"]        = sigma_width;
  jVehicle["wheel_base"]         = wheel_base;
  jVehicle["max_steering_angle"] = max_steering_angle;
  jVehicle["max_speed"]          = max_speed;
  jVehicle["max_acceleration"]   = max_acceleration;
  jVehicle["random"]             = random;
  return jVehicle;
}

/**
 * @brief Returns a new Vehicle object created from the parameters of the JSON file.
 *
 * @param jVehicle The JSON file.
 * @return Vehicle
 */
Vehicle Vehicle::fromJSON(const json& jVehicle) {
  auto position_x = jVehicle["position_x"].get<float>();
  auto position_y = jVehicle["position_y"].get<float>();
  auto velocity_x = jVehicle["velocity_x"].get<float>();
  auto velocity_y = jVehicle["velocity_y"].get<float>();
  auto heading    = jVehicle["heading"].get<float>();
  auto length     = jVehicle["length"].get<float>();
  auto width      = jVehicle["width"].get<float>();

  if (jVehicle["random"].get<bool>()) {
    position_x = math::getNoise(position_x, jVehicle["sigma_position_x"].get<float>(), true);
    position_y = math::getNoise(position_y, jVehicle["sigma_position_y"].get<float>(), true);
    velocity_x = math::getNoise(velocity_x, jVehicle["sigma_velocity_x"].get<float>(), true);
    velocity_y = math::getNoise(velocity_y, jVehicle["sigma_velocity_y"].get<float>(), true);
    heading    = math::getNoise(heading, jVehicle["sigma_heading"].get<float>(), true);
    length     = math::getNoise(length, jVehicle["sigma_length"].get<float>(), true);
    width      = math::getNoise(width, jVehicle["sigma_width"].get<float>(), true);
  }

  auto vehicle =
      Vehicle(jVehicle["random"].get<bool>(), position_x, position_y, velocity_x, velocity_y,
              heading, length, width, jVehicle["sigma_position_x"].get<float>(),
              jVehicle["sigma_position_y"].get<float>(), jVehicle["sigma_velocity_x"].get<float>(),
              jVehicle["sigma_velocity_y"].get<float>(), jVehicle["sigma_heading"].get<float>(),
              jVehicle["sigma_length"].get<float>(), jVehicle["sigma_width"].get<float>(),
              jVehicle["wheel_base"].get<float>(), jVehicle["max_steering_angle"].get<float>(),
              jVehicle["max_speed"].get<float>(), jVehicle["max_acceleration"].get<float>());

  return vehicle;
}

/**
 * @brief Exports the parameters of the Desire object to JSON.
 *
 * @return json The parameters.
 */
json Desire::toJSON() const {
  json jDesire;
  jDesire["velocity"]              = velocity;
  jDesire["velocity_tolerance"]    = velocity_tolerance;
  jDesire["lane"]                  = lane;
  jDesire["lane_center_tolerance"] = lane_center_tolerance;
  return jDesire;
}

/**
 * @brief Returns a new Desire object created from the parameters of the JSON file.
 *
 * @param jDesire The JSON file.
 * @return Desire
 */
Desire Desire::fromJSON(const json& jDesire) {
  return {jDesire["velocity"].get<float>(), jDesire["velocity_tolerance"].get<float>(),
          jDesire["lane"].get<unsigned int>(), jDesire["lane_center_tolerance"].get<float>()};
}

/**
 * @brief Exports the parameters of the TerminalCondition object to JSON.
 *
 * @return json The parameters.
 */
json TerminalCondition::toJSON() const {
  json jTerminalCondition;
  jTerminalCondition["position_x"]            = position_x;
  jTerminalCondition["position_y"]            = position_y;
  jTerminalCondition["comparator_position_x"] = comparator_position_x;
  jTerminalCondition["comparator_position_y"] = comparator_position_y;
  return jTerminalCondition;
}

/**
 * @brief Returns a new TerminalCondition object created from the parameters of the JSON file.
 *
 * @param jTerminalCondition The JSON file.
 * @return TerminalCondition
 */
TerminalCondition TerminalCondition::fromJSON(const json& jTerminalCondition) {
  return {jTerminalCondition["position_x"].get<float>(),
          jTerminalCondition["position_y"].get<float>(),
          jTerminalCondition["comparator_position_x"].get<std::string>(),
          jTerminalCondition["comparator_position_y"].get<std::string>()};
}

/**
 * @brief Exports the parameters of the ActionSpaceRectangle object to JSON.
 *
 * @return json The parameters.
 */
json ActionSpaceRectangle::toJSON() const {
  json jActionSpaceRectangle;
  jActionSpaceRectangle["max_velocity_change"] = max_velocity_change;
  jActionSpaceRectangle["max_lateral_change"]  = max_lateral_change;
  jActionSpaceRectangle["delta_velocity"]      = delta_velocity;
  return jActionSpaceRectangle;
}

/**
 * @brief Returns a new ActionSpaceRectangle object created from the parameters of the JSON file.
 *
 * @param jActionSpaceRectangle The JSON file.
 * @return ActionSpaceRectangle
 */
ActionSpaceRectangle ActionSpaceRectangle::fromJSON(const json& jActionSpaceRectangle) {
  return {jActionSpaceRectangle["max_velocity_change"].get<float>(),
          jActionSpaceRectangle["max_lateral_change"].get<float>(),
          jActionSpaceRectangle["delta_velocity"].get<float>()};
}

/**
 * @brief Exports the parameters of the ActionSpace object to JSON.
 *
 * @param variant
 * @return json The parameters.
 */
json ActionSpace::toJSON(const ActionSpace::variant_t& variant) {
  json jActionSpace;
  if (std::holds_alternative<ActionSpaceRectangle>(variant)) {
    jActionSpace         = std::get<ActionSpaceRectangle>(variant).toJSON();
    jActionSpace["type"] = ActionSpace::Type::Rectangle;
  } else {
    throw std::runtime_error("Action space variant is invalid.");
  }
  return jActionSpace;
}

/**
 * @brief Returns a new ActionSpace object created from the parameters of the JSON file.
 *
 * @param jActionSpace The JSON file.
 * @return ActionSpace::variant_t
 */
ActionSpace::variant_t ActionSpace::fromJSON(const json& jActionSpace) {
  auto type = jActionSpace["type"].get<ActionSpace::Type>();
  switch (type) {
    case ActionSpace::Type::Rectangle:
      return ActionSpaceRectangle::fromJSON(jActionSpace);
    default:
      throw std::invalid_argument("Invalid type for ActionSpace: " +
                                  jActionSpace["type"].get<std::string>());
  }
}

/**
 * @brief Exports the parameters of the CostModel object to JSON.
 *
 * @return json The parameters.
 */
json CostModel::toJSON() const {
  json jCostModel;
  jCostModel["name"]                    = name;
  jCostModel["w_lane_change"]           = w_lane_change;
  jCostModel["w_lane_deviation"]        = w_lane_deviation;
  jCostModel["w_lane_center_deviation"] = w_lane_center_deviation;
  jCostModel["w_velocity_deviation"]    = w_velocity_deviation;
  jCostModel["w_acceleration_x"]        = w_acceleration_x;
  jCostModel["w_acceleration_y"]        = w_acceleration_y;
  jCostModel["cost_collision"]          = cost_collision;
  jCostModel["cost_invalid_state"]      = cost_invalid_state;
  jCostModel["cost_invalid_action"]     = cost_invalid_action;
  jCostModel["cost_enter_safe_range"]   = cost_enter_safe_range;
  jCostModel["reward_terminal"]         = reward_terminal;
  // linear cooperative parameters
  jCostModel["w_acceleration_y_cooperative"]        = w_acceleration_y_cooperative;
  jCostModel["w_lane_deviation_cooperative"]        = w_lane_deviation_cooperative;
  jCostModel["w_lane_center_deviation_cooperative"] = w_lane_center_deviation_cooperative;
  jCostModel["w_velocity_deviation_cooperative"]    = w_velocity_deviation_cooperative;
  jCostModel["cost_collision_cooperative"]          = cost_collision_cooperative;
  jCostModel["cost_invalid_state_cooperative"]      = cost_invalid_state_cooperative;
  jCostModel["cost_invalid_action_cooperative"]     = cost_invalid_action_cooperative;
  if (name == "costNonLinear") {
    const auto [w1_vector, w1_rows, w1_columns] = convertEigenMatrixToVector(w1);
    const auto [w2_vector, w2_rows, w2_columns] = convertEigenMatrixToVector(w2);
    jCostModel["w1"]                            = w1_vector;
    jCostModel["w2"]                            = w2_vector;
  }

  return jCostModel;
}

/**
 * @brief Returns a new CostModel object created from the parameters of the JSON file.
 *
 * @param jCostModel The JSON file.
 * @return CostModel
 */
CostModel CostModel::fromJSON(const json& jCostModel) {
  auto name = jCostModel["name"].get<std::string>();
  if (name == "costNonLinear") {
    std::vector<float> v1;
    int rowsW1{10};
    int colsW1{5};
    v1                 = jCostModel["w1"].get<std::vector<float>>();
    Eigen::MatrixXd w1 = convertVectorToEigenMatrix(v1, rowsW1, colsW1);
    std::vector<float> v2;
    int rowsW2{5};
    int colsW2{1};
    v2                 = jCostModel["w2"].get<std::vector<float>>();
    Eigen::MatrixXd w2 = convertVectorToEigenMatrix(v2, rowsW2, colsW2);
    return {jCostModel["name"].get<std::string>(),
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            w1,
            w2};

  } else if (name == "costLinearCooperative") {
    Eigen::MatrixXd w1;
    Eigen::MatrixXd w2;
    auto w_acceleration_y_cooperative = jCostModel["w_acceleration_y_cooperative"].get<float>();
    auto w_lane_deviation_cooperative = jCostModel["w_lane_deviation_cooperative"].get<float>();
    auto w_lane_center_deviation_cooperative =
        jCostModel["w_lane_center_deviation_cooperative"].get<float>();
    auto w_velocity_deviation_cooperative =
        jCostModel["w_velocity_deviation_cooperative"].get<float>();
    auto cost_collision_cooperative     = jCostModel["cost_collision_cooperative"].get<float>();
    auto cost_invalid_state_cooperative = jCostModel["cost_invalid_state_cooperative"].get<float>();
    auto cost_invalid_action_cooperative =
        jCostModel["cost_invalid_action_cooperative"].get<float>();
    return {jCostModel["name"].get<std::string>(),
            jCostModel["w_lane_change"].get<float>(),
            jCostModel["w_lane_deviation"].get<float>(),
            jCostModel["w_lane_center_deviation"].get<float>(),
            jCostModel["w_velocity_deviation"].get<float>(),
            jCostModel["w_acceleration_x"].get<float>(),
            jCostModel["w_acceleration_y"].get<float>(),
            jCostModel["cost_collision"].get<float>(),
            jCostModel["cost_invalid_state"].get<float>(),
            jCostModel["cost_invalid_action"].get<float>(),
            jCostModel["cost_enter_safe_range"].get<float>(),
            jCostModel["reward_terminal"].get<float>(),
            w_acceleration_y_cooperative,
            w_lane_deviation_cooperative,
            w_lane_center_deviation_cooperative,
            w_velocity_deviation_cooperative,
            cost_collision_cooperative,
            cost_invalid_state_cooperative,
            cost_invalid_action_cooperative,
            w1,
            w2};
  } else {
    Eigen::MatrixXd w1;
    Eigen::MatrixXd w2;
    float w_acceleration_y_cooperative{0.0f};
    float w_lane_deviation_cooperative{0.0f};
    float w_lane_center_deviation_cooperative{0.0f};
    float w_velocity_deviation_cooperative{0.0f};
    float cost_collision_cooperative{0.0f};
    float cost_invalid_state_cooperative{0.0f};
    float cost_invalid_action_cooperative{0.0f};
    return {jCostModel["name"].get<std::string>(),
            jCostModel["w_lane_change"].get<float>(),
            jCostModel["w_lane_deviation"].get<float>(),
            jCostModel["w_lane_center_deviation"].get<float>(),
            jCostModel["w_velocity_deviation"].get<float>(),
            jCostModel["w_acceleration_x"].get<float>(),
            jCostModel["w_acceleration_y"].get<float>(),
            jCostModel["cost_collision"].get<float>(),
            jCostModel["cost_invalid_state"].get<float>(),
            jCostModel["cost_invalid_action"].get<float>(),
            jCostModel["cost_enter_safe_range"].get<float>(),
            jCostModel["reward_terminal"].get<float>(),
            w_acceleration_y_cooperative,
            w_lane_deviation_cooperative,
            w_lane_center_deviation_cooperative,
            w_velocity_deviation_cooperative,
            cost_collision_cooperative,
            cost_invalid_state_cooperative,
            cost_invalid_action_cooperative,
            w1,
            w2};
  }
}

/**
 * @brief Converts the vector to an eigen matrix.
 *
 * @param values The list of elements for the matrix.
 * @param nRows The number of rows.
 * @param nCols The number of columns.
 * @return Eigen::MatrixXd The matrix.
 */
Eigen::MatrixXd CostModel::convertVectorToEigenMatrix(const std::vector<float>& values, int nRows,
                                                      int nCols) {
  Eigen::MatrixXd matrix(nRows, nCols);
  for (int i = 0; i < nRows; ++i) {
    for (int j = 0; j < nCols; ++j) {
      // list has the form [row1,row2,...]
      matrix(i, j) = values[j + nCols * i];
      // std::cout << matrix(i, j) << " ";
    }
    // std::cout << std::endl;
  }
  return matrix;
}

/**
 * @brief Converts the eigen matrix to a vector.
 *
 * @param matrix The matrix
 * @return std::vector<float>
 */
std::tuple<std::vector<float>, unsigned int, unsigned int> CostModel::convertEigenMatrixToVector(
    const Eigen::MatrixXd& matrix) {
  std::vector<float> v;
  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
      v.push_back(matrix(i, j));
    }
  }
  return {v, matrix.rows(), matrix.rows()};
}

/**
 * @brief Exports the parameters of the Agent object to JSON.
 *
 * @return json The parameters.
 */
json Agent::toJSON() const {
  json jAgent;
  jAgent["id"]                 = id;
  jAgent["is_predefined"]      = is_predefined;
  jAgent["cooperation_factor"] = cooperation_factor;
  jAgent["desire"]             = desire.toJSON();
  jAgent["vehicle"]            = vehicle.toJSON();
  jAgent["terminal_condition"] = terminal_condition.toJSON();
  jAgent["action_space"]       = ActionSpace::toJSON(action_space);
  jAgent["cost_model"]         = cost_model.toJSON();
  return jAgent;
}

/**
 * @brief Returns a new Agent object created from the parameters of the JSON file.
 *
 * @param jAgent The JSON file.
 * @return Agent
 */
Agent Agent::fromJSON(const json& jAgent) {
  auto vehicle            = Vehicle::fromJSON(jAgent["vehicle"]);
  auto action_space       = ActionSpace::fromJSON(jAgent["action_space"]);
  auto cost_model         = CostModel::fromJSON(jAgent["cost_model"]);
  auto desire             = Desire::fromJSON(jAgent["desire"]);
  auto terminal_condition = TerminalCondition::fromJSON(jAgent["terminal_condition"]);

  return {jAgent["id"].get<unsigned int>(),
          jAgent["is_predefined"].get<bool>(),
          jAgent["cooperation_factor"].get<float>(),
          desire,
          vehicle,
          terminal_condition,
          action_space,
          cost_model};
}

/**
 * @brief Exports the parameters of the Scenario object to JSON.
 *
 * @return json The parameters.
 */
json Scenario::toJSON() const {
  json jScenario;
  jScenario["name"] = name;
  jScenario["road"] = road.toJSON();
  json jObstacles;
  for (const auto& obstacle : obstacles) {
    jObstacles.push_back(obstacle.toJSON());
  }
  jScenario["obstacles"] = jObstacles;
  json jAgents;
  for (const auto& agent : agents) {
    jAgents.push_back(agent.toJSON());
  }
  jScenario["agents"] = jAgents;
  return jScenario;
}

/**
 * @brief Return a new Scenario object created from the parameters of the JSON file.
 *
 * @param jScenario The JSON file.
 * @return Scenario
 */
Scenario Scenario::fromJSON(const json& jScenario) {
  std::vector<Agent> agents;
  json jAgents = jScenario["agents"];
  for (auto& jAgent : jAgents) {
    agents.push_back(Agent::fromJSON(jAgent));
  }
  std::vector<Obstacle> obstacles;
  json jObstacles = jScenario["obstacles"];
  for (auto& jObstacle : jObstacles) {
    obstacles.push_back(Obstacle::fromJSON(jObstacle));
  }
  Road road = Road::fromJSON(jScenario["road"]);

  return {jScenario["name"], road, agents, obstacles};
}
}  // namespace proseco_planning::config