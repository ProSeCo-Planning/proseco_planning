#include "proseco_planning/config/configuration.h"

#include <map>

#include "nlohmann/json.hpp"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/outputOptions.h"
#include "proseco_planning/config/scenarioOptions.h"

/**
 * @brief The namespace of the ProSeCo Planning library.
 *
 */
namespace proseco_planning {
/**
 * @brief The namespace where all config classes are defined, that specify the configuration of the
 * planner.
 *
 */
namespace config {
json Options::toJSON() const {
  json jOptions;
  jOptions["output_options"]  = output_options.toJSON();
  jOptions["compute_options"] = compute_options.toJSON();
  return jOptions;
}
/**
 * @brief Returns a new Options object created from the parameters of the JSON file.
 *
 * @param jOptions The JSON file.
 * @return Options
 */
Options Options::fromJSON(const json& jOptions) {
  Options options = Options(OutputOptions::fromJSON(jOptions["output_options"]),
                            ComputeOptions::fromJSON(jOptions["compute_options"]));
  return options;
}
}  // namespace config

/**
 * @brief Constructs a singleton object for the configuration of the entire program.
 *
 * @param scenario The simulated scenario.
 * @param options The options for the computation as well as the ouput.
 * @return const Config*
 */
const Config* Config::create(const config::Scenario& scenario, const config::Options& options) {
  if (instance == nullptr) {
    instance = std::make_unique<Config>(Config(scenario, options));
  }

  return instance.get();
}

/**
 * @brief Gets the instance.
 *
 * @return const Config*
 */
const Config* Config::get() {
  if (instance == nullptr) {
    throw "Config instance is nullptr";
  }
  return instance.get();
}

/**
 * @brief Resets the instance.
 *
 * @return const Config*
 */
const Config* Config::reset() {
  instance.reset(nullptr);
  return instance.get();
}

/**
 * @brief Exports the information included in the configuration to JSON.
 *
 * @return json
 */
json Config::toJSON() const {
  json jConfig;
  jConfig["scenario"] = scenario.toJSON();
  jConfig["options"]  = options.toJSON();
  return jConfig;
}

/**
 * @brief Stores the instance.
 * @note nullptr, because instance will be initialized on demand.
 */
std::unique_ptr<Config> Config::instance = nullptr;

/**
 * @brief Constructs a new Config object. Private constructor avoiding being called directly.
 *
 * @param scenario The simulated scenario.
 * @param options The options for the computation as well as the ouput.
 */
Config::Config(const config::Scenario& scenario, const config::Options& options)
    : scenario(scenario), options(options) {
  // load the configuration files from some path
  // assemble the config object
}
}  // namespace proseco_planning