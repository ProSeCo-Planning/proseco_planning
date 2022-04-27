/**
 * @file configuration.h
 * @brief This file defines the configuration for the execution of the ProSeCo planner.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>

#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include "computeOptions.h"
#include "outputOptions.h"
#include "scenarioOptions.h"

namespace proseco_planning {

namespace config {

/**
 * @brief The struct that contains the options for the computation as well as the result output.
 *
 */
struct Options {
  /**
   * @brief Constructs a new Options object.
   *
   * @param output_options
   * @param compute_options
   */
  Options(OutputOptions output_options, ComputeOptions compute_options)
      : output_options(output_options), compute_options(compute_options) {}

  /// The output parameters.
  const OutputOptions output_options;
  /// The computation parameters.
  const ComputeOptions compute_options;

  json toJSON() const;

  static Options fromJSON(const json& jOptions);
};

}  // namespace config

/**
 * @brief The config class: Defines the configuration of the simulation.
 * @note Implemented as singleton so that the entire program configuration is accessible from one
 * point within the program.
 */

class Config {
 public:
  static const Config* create(const config::Scenario& scenario, const config::Options& options);

  json toJSON() const;

  static const Config* get();

  static const Config* reset();

  /// The simulated scenario.
  config::Scenario scenario;
  /// The options for the computation as well as the ouput.
  config::Options options;

 private:
  static std::unique_ptr<Config> instance;

  Config(const config::Scenario& scenario, const config::Options& options);
};

/**
 * @brief Shorthand for calling the compute options directly.
 *
 * @return compute The compute options.
 */
inline const auto& cOpt() { return Config::get()->options.compute_options; }

/**
 * @brief Shorthand for calling the output options directly.
 *
 * @return Output The output options.
 */
inline const auto& oOpt() { return Config::get()->options.output_options; }

/**
 * @brief Shorthand for calling the scenario directly.
 *
 * @return scenario The scenario configuration.
 */
inline const auto& sOpt() { return Config::get()->scenario; };
}  // namespace proseco_planning