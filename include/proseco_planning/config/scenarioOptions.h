/**
 * @file scenarioOptions.h
 * @brief This file defines the scenario configuration.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <sys/types.h>
#include <eigen3/Eigen/Core>
#include <string>
#include <variant>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace proseco_planning::config {
/**
 * @brief The struct that contains the specifications of the obstacles involved in a scenario.
 *
 */
struct Obstacle {
  /// The unique identifier.
  const unsigned int id;
  /// The flag that indicates randomness in the objects attributes.
  const bool random;
  /// The x position [m].
  const float position_x;
  /// The y position [m].
  const float position_y;
  /// The heading [rad].
  const float heading;
  /// The length [m].
  const float length;
  /// The width [m].
  const float width;
  /// The standard deviation in x position [m].
  const float sigma_position_x;
  /// The standard deviation in y position [m].
  const float sigma_position_y;
  /// The standard deviation in heading [rad].
  const float sigma_heading;
  /// The standard deviation in length [m].
  const float sigma_length;
  /// The standard deviation in width [m].
  const float sigma_width;

  /**
   * @brief Constructs a new Obstacle object.
   *
   * @param id
   * @param random
   * @param position_x
   * @param position_y
   * @param heading
   * @param length
   * @param width
   * @param sigma_position_x
   * @param sigma_position_y
   * @param sigma_heading
   * @param sigma_length
   * @param sigma_width
   */
  Obstacle(unsigned int id, bool random, float position_x, float position_y, float heading,
           float length, float width, float sigma_position_x, float sigma_position_y,
           float sigma_heading, float sigma_length, float sigma_width)
      : id(id),
        random(random),
        position_x(position_x),
        position_y(position_y),
        heading(heading),
        length(length),
        width(width),
        sigma_position_x(sigma_position_x),
        sigma_position_y(sigma_position_y),
        sigma_heading(sigma_heading),
        sigma_length(sigma_length),
        sigma_width(sigma_width) {}

  json toJSON() const;

  static Obstacle fromJSON(const json& jObstacle);
};

/**
 * @brief The specification of the road.
 *
 */
struct Road {
  /// The flag that indicates randomness in the objects attributes.
  const bool random{};
  /// The number of lanes the road consists of.
  const unsigned int number_lanes{};
  /// The lane width of each lane [m].
  const float lane_width{};
  /// The standard deviation of the lane width [m].
  const float sigma_lane_width{};
  /**
   * @brief Constructs a new Road object.
   *
   * @param random
   * @param number_lanes
   * @param lane_width
   * @param sigma_lane_width
   */
  Road(bool random, unsigned int number_lanes, float lane_width, float sigma_lane_width)
      : random(random),
        number_lanes(number_lanes),
        lane_width(lane_width),
        sigma_lane_width(sigma_lane_width) {}

  json toJSON() const;

  static Road fromJSON(const json& jRoad);
};

/**
 * @brief The specification of the vehicle.
 *
 */
struct Vehicle {
  /// The flag that indicates randomness in the objects attributes.
  const bool random;
  /// The x position [m].
  const float position_x;
  /// The y position [m].
  const float position_y;
  /// The x velocity [m/s].
  const float velocity_x;
  /// The y velocity [m/s].
  const float velocity_y;
  /// The heading [rad].
  const float heading;
  /// The width [m].
  const float width;
  /// The length [m].
  const float length;
  /// The standard deviation in x position [m].
  const float sigma_position_x;
  /// The standard deviation in y position [m].
  const float sigma_position_y;
  /// The standard deviation in x velocity [m/s].
  const float sigma_velocity_x;
  /// The standard deviation in y velocity [m/s].
  const float sigma_velocity_y;
  /// The standard deviation in heading [rad].
  const float sigma_heading;
  /// The standard deviation in width [m].
  const float sigma_width;
  /// The standard deviation in length [m].
  const float sigma_length;
  /// The wheel base [m].
  const float wheel_base;
  /// The maximum steering angle [rad].
  const float max_steering_angle;
  /// The maximum speed [m/s].
  const float max_speed;
  /// The maximum acceleration [m/s^2].
  const float max_acceleration;
  /**
   * @brief Constructs a new Vehicle object.
   *
   * @param random
   * @param position_x
   * @param position_y
   * @param velocity_x
   * @param velocity_y
   * @param heading
   * @param length
   * @param width
   * @param sigma_position_x
   * @param sigma_position_y
   * @param sigma_velocity_x
   * @param sigma_velocity_y
   * @param sigma_heading
   * @param sigma_length
   * @param sigma_width
   * @param wheel_base
   * @param max_steering_angle
   * @param max_speed
   * @param max_acceleration
   */
  Vehicle(bool random, float position_x, float position_y, float velocity_x, float velocity_y,
          float heading, float length, float width, float sigma_position_x, float sigma_position_y,
          float sigma_velocity_x, float sigma_velocity_y, float sigma_heading, float sigma_length,
          float sigma_width, float wheel_base, float max_steering_angle, float max_speed,
          float max_acceleration)
      : random(random),
        position_x(position_x),
        position_y(position_y),
        velocity_x(velocity_x),
        velocity_y(velocity_y),
        heading(heading),
        width(width),
        length(length),
        sigma_position_x(sigma_position_x),
        sigma_position_y(sigma_position_y),
        sigma_velocity_x(sigma_velocity_x),
        sigma_velocity_y(sigma_velocity_y),
        sigma_heading(sigma_heading),
        sigma_width(sigma_width),
        sigma_length(sigma_length),
        wheel_base(wheel_base),
        max_steering_angle(max_steering_angle),
        max_speed(max_speed),
        max_acceleration(max_acceleration) {}

  json toJSON() const;

  static Vehicle fromJSON(const json& jVehicle);
};

/**
 * @brief The struct that contains the desire of the agent.
 *
 */
struct Desire {
  /// The desired velocity [m/s].
  const float velocity;
  /// The tolerance for velocity deviation [m/s], so that the desire remains fulfilled.
  const float velocity_tolerance;
  /// The desired lane.
  const unsigned int lane;
  /// The tolerance for lane center deviation [m], so that the desire remains fulfilled.
  const float lane_center_tolerance;

  /**
   * @brief Constructs a new Desire object.
   *
   * @param velocity
   * @param velocity_tolerance
   * @param lane
   * @param lane_center_tolerance
   */
  Desire(float velocity, float velocity_tolerance, unsigned int lane, float lane_center_tolerance)
      : velocity(velocity),
        velocity_tolerance(velocity_tolerance),
        lane(lane),
        lane_center_tolerance(lane_center_tolerance) {}

  json toJSON() const;

  static Desire fromJSON(const json& jDesire);
};

/**
 * @brief The struct that contains the terminal condition.
 *
 */
struct TerminalCondition {
  /// The x position [m].
  const float position_x;
  /// The y position [m].
  const float position_y;
  /// The comparator defining the relation to be compared.
  const std::string comparator_position_x;
  /// The comparator defining the relation to be compared.
  const std::string comparator_position_y;

  /**
   * @brief Constructs a new Terminal Condition object.
   *
   * @param position_x
   * @param position_y
   * @param comparator_position_x
   * @param comparator_position_y
   */
  TerminalCondition(float position_x, float position_y, std::string comparator_position_x,
                    std::string comparator_position_y)
      : position_x(position_x),
        position_y(position_y),
        comparator_position_x(comparator_position_x),
        comparator_position_y(comparator_position_y) {}

  json toJSON() const;

  static TerminalCondition fromJSON(const json& jTerminalCondition);
};

/**
 * @brief The struct that contains the information about the action space in form of a rectangle.
 *
 */
struct ActionSpaceRectangle {
  /// The action space type.
  inline static const std::string TYPE{"rectangle"};
  /// The maximum longitudinal velocity change.
  const float max_velocity_change;
  /// The maximum lateral position change.
  const float max_lateral_change;
  /// The velocity change that specifies the bound between the action class "do_nothing" and
  /// "accelerate".
  const float delta_velocity;

  /**
   * @brief Constructs a new Action Space Rectangle object.
   *
   * @param max_velocity_change
   * @param max_lateral_change
   * @param delta_velocity
   */
  ActionSpaceRectangle(float max_velocity_change, float max_lateral_change, float delta_velocity)
      : max_velocity_change(max_velocity_change),
        max_lateral_change(max_lateral_change),
        delta_velocity(delta_velocity){};

  json toJSON() const;

  static ActionSpaceRectangle fromJSON(const json& jActionSpaceRectangle);
};

/**
 * @brief The struct that contains the parameters of the ActionSpace.
 *
 */
struct ActionSpace {
  /// The enum for setting the type of the action space
  enum class Type {
    /// The invalid action space type (default for variant type).
    INVALID,
    /// The rectangular action space type.
    Rectangle
  };

  using variant_t = std::variant<std::monostate, ActionSpaceRectangle>;

  static json toJSON(const ActionSpace::variant_t& variant);

  static ActionSpace::variant_t fromJSON(const json& jActionSpace);
};

/// Maps ActionSpace::Type values to JSON as strings.*/
NLOHMANN_JSON_SERIALIZE_ENUM(ActionSpace::Type,
                             {
                                 {ActionSpace::Type::INVALID, nullptr},  // Invalid is default
                                 {ActionSpace::Type::Rectangle, ActionSpaceRectangle::TYPE},
                             })

struct CostModel {
  /// The name of the cost model.
  const std::string name;
  /// The weight for lane changes.
  const float w_lane_change;
  /// The weight for lane deviations (compared to the desired lane).
  const float w_lane_deviation;
  /// The weight for deviations from the center of a lane.
  const float w_lane_center_deviation;
  /// The weight for velocity deviations (compared to the desired velocity).
  const float w_velocity_deviation;
  /// The weight for x acceleration.
  const float w_acceleration_x;
  /// The weight for y acceleration.
  const float w_acceleration_y;
  /// The weight for collisions.
  const float cost_collision;
  /// The weight for invalid states (i.e. driving off road).
  const float cost_invalid_state;
  /// The weight for invalid actions (i.e. violating maximum steering angle or acceleration).
  const float cost_invalid_action;
  /// The weight for reaching a safe range @todo remove.
  const float cost_enter_safe_range;
  /// The weight for terminal rewards (i.e. when a scenario is considered solved).
  const float reward_terminal;

  ///@{
  /// The linear cooperative parameters. For specific definitions see linear parameters.
  const float w_acceleration_y_cooperative;
  const float w_lane_deviation_cooperative;
  const float w_lane_center_deviation_cooperative;
  const float w_velocity_deviation_cooperative;
  const float cost_collision_cooperative;
  const float cost_invalid_state_cooperative;
  const float cost_invalid_action_cooperative;
  ///@}

  ///@{
  /// The nonlinear parameters.
  const Eigen::MatrixXd w1;
  const Eigen::MatrixXd w2;
  ///@}

  /**
   * @brief Constructs a new Cost Model object.
   *
   * @param name
   * @param w_lane_change
   * @param w_lane_deviation
   * @param w_lane_center_deviation
   * @param w_velocity_deviation
   * @param w_acceleration_x
   * @param w_acceleration_y
   * @param cost_collision
   * @param cost_invalid_state
   * @param cost_invalid_action
   * @param cost_enter_safe_range
   * @param reward_terminal
   * @param w_acceleration_y_cooperative
   * @param w_lane_deviation_cooperative
   * @param w_lane_center_deviation_cooperative
   * @param w_velocity_deviation_cooperative
   * @param cost_collision_cooperative
   * @param cost_invalid_state_cooperative
   * @param cost_invalid_action_cooperative
   * @param w1
   * @param w2
   */
  CostModel(const std::string& name, float w_lane_change, float w_lane_deviation,
            float w_lane_center_deviation, float w_velocity_deviation, float w_acceleration_x,
            float w_acceleration_y, float cost_collision, float cost_invalid_state,
            float cost_invalid_action, float cost_enter_safe_range, float reward_terminal,
            float w_acceleration_y_cooperative, float w_lane_deviation_cooperative,
            float w_lane_center_deviation_cooperative, float w_velocity_deviation_cooperative,
            float cost_collision_cooperative, float cost_invalid_state_cooperative,
            float cost_invalid_action_cooperative, Eigen::MatrixXd w1, Eigen::MatrixXd w2)
      : name(name),
        w_lane_change(w_lane_change),
        w_lane_deviation(w_lane_deviation),
        w_lane_center_deviation(w_lane_center_deviation),
        w_velocity_deviation(w_velocity_deviation),
        w_acceleration_x(w_acceleration_x),
        w_acceleration_y(w_acceleration_y),
        cost_collision(cost_collision),
        cost_invalid_state(cost_invalid_state),
        cost_invalid_action(cost_invalid_action),
        cost_enter_safe_range(cost_enter_safe_range),
        reward_terminal(reward_terminal),
        w_acceleration_y_cooperative(w_acceleration_y_cooperative),
        w_lane_deviation_cooperative(w_lane_deviation_cooperative),
        w_lane_center_deviation_cooperative(w_lane_center_deviation_cooperative),
        w_velocity_deviation_cooperative(w_velocity_deviation_cooperative),
        cost_collision_cooperative(cost_collision_cooperative),
        cost_invalid_state_cooperative(cost_invalid_state_cooperative),
        cost_invalid_action_cooperative(cost_invalid_action_cooperative),
        w1(w1),
        w2(w2) {}

  json toJSON() const;

  static CostModel fromJSON(const json& jCostModel);

  static Eigen::MatrixXd convertVectorToEigenMatrix(const std::vector<float>& values, int nRows,
                                                    int nCols);

  static std::tuple<std::vector<float>, unsigned int, unsigned int> convertEigenMatrixToVector(
      const Eigen::MatrixXd& matrix);
};

/**
 * @brief The agent.
 *
 */
struct Agent {
  /// The unique identifier.
  const unsigned int id;
  /// The flag that indicates constant behavior (i.e. constant velocity).
  const bool is_predefined;
  /// The coefficient that weights the incorporation of the rewards of other agents.
  const float cooperation_factor;
  /// The desire of the agent.
  const Desire desire;
  /// The vehicle of the agent.
  const Vehicle vehicle;
  /// The terminal condition (e.g. when the scenario can be considered solved).
  const TerminalCondition terminal_condition;
  /// The action space.
  const ActionSpace::variant_t action_space;
  /// The cost model used to calculate the costs with.
  const CostModel cost_model;

  /**
   * @brief Constructs a new Agent object.
   *
   * @param id
   * @param is_predefined
   * @param cooperation_factor
   * @param desire
   * @param vehicle
   * @param terminal_condition
   * @param action_space
   * @param cost_model
   */
  Agent(unsigned int id, bool is_predefined, float cooperation_factor, Desire desire,
        Vehicle vehicle, TerminalCondition terminal_condition, ActionSpace::variant_t action_space,
        CostModel cost_model)
      : id(id),
        is_predefined(is_predefined),
        cooperation_factor(cooperation_factor),
        desire(desire),
        vehicle(vehicle),
        terminal_condition(terminal_condition),
        action_space(action_space),
        cost_model(cost_model) {}

  json toJSON() const;

  static Agent fromJSON(const json& jAgent);
};

/**
 * @brief The simulated scenario.
 */
struct Scenario {
  /// The name of the scenario (e.g. SC01).
  const std::string name;
  /// The road of the scenario.
  config::Road road;
  /// The agents of the scenario.
  const std::vector<config::Agent> agents;
  /// The obstacles of the scenario.
  const std::vector<config::Obstacle> obstacles;
  // constructor
  Scenario(const std::string& name, config::Road road, std::vector<config::Agent> agents,
           std::vector<config::Obstacle> obstacles)
      : name(name), road(road), agents(agents), obstacles(obstacles) {}

  json toJSON() const;

  static Scenario fromJSON(const json& jScenario);
};
}  // namespace proseco_planning::config