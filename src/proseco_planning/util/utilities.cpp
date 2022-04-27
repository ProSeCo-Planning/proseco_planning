#include "proseco_planning/util/utilities.h"

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <iomanip>
#include <ios>
#include <iterator>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/action/actionSpace.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/outputOptions.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/math/mathlib.h"

/**
 * @brief The namespace where all utility functions are defined.
 *
 */
namespace proseco_planning::util {

/**
 * @brief Checks whether a string has a specified ending.
 *
 * @param fullString The string to check.
 * @param ending The ending to check.
 * @return true If the string has the specified ending.
 * @return false Otherwise.
 */
bool hasEnding(const std::string& fullString, const std::string& ending) {
  if (fullString.length() >= ending.length()) {
    return (0 ==
            fullString.compare(fullString.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}

/**
 * @brief Converts a float to a string of the given precision.
 *
 * @param number The floating point number.
 * @param precision The desired precision of the floating point number as a string.
 * @return std::string The string with the given precision.
 */
std::string toStringPrecision(const float number, const unsigned int precision) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision) << number;
  return ss.str();
}

/**
 * @brief Pads a number with zeros to the left.
 *
 * @param number The number to pad.
 * @param width The desired width of the padded number.
 * @return std::string The padded number.
 */
std::string padNumber(const unsigned int number, const unsigned int width) {
  std::stringstream ss;
  ss << std::setw(width) << std::setfill('0') << number;
  return ss.str();
}

/**
 * @brief Converts an action set to a string representation, based on the action classes used.
 *
 * @param actionSet The action set to convert.
 * @return std::string The string indicating an action class for each action in the set.
 */
std::string actionSetToString(const ActionSet& actionSet) {
  std::string actionSetString;
  for (const auto& action : actionSet) {
    actionSetString.append(ActionSpace::ACTION_CLASS_NAME_MAP.at(action->m_actionClass));
    actionSetString.append(",");
  }
  return actionSetString;
}

/**
 * @brief Create a Config object using a scenario and and options file.
 *
 * @param optionsFilePath The path to the options .json file.
 * @param scenarioFilePath The path to the scenario .json file.
 */
void createConfig(const std::string& optionsFilePath, const std::string& scenarioFilePath) {
  const auto& options = config::Options::fromJSON(loadJSON(optionsFilePath));
  math::Random::setRandomSeed(options.compute_options.random_seed);
  const auto& scenario = config::Scenario::fromJSON(loadJSON(scenarioFilePath));
  Config::create(scenario, options);
};

/**
 * @brief Loads a .json file.
 *
 * @param filePath The path to the .json file, with the extension.
 * @return json The loaded .json file.
 */
json loadJSON(const std::string& filePath) {
  std::ifstream i{filePath};
  json j;
  i >> j;
  return j;
}

/**
 * @brief Loads a .msgpack file.
 *
 * @param filePath The path to the .msgpack file, with the extension.
 * @return json The loaded .msgpack file as a json object.
 */
json loadMsgPackToJSON(const std::string& filePath) {
  std::ifstream file(filePath, std::ios::in | std::ios::binary);
  std::vector<uint8_t> msgpackObject((std::istreambuf_iterator<char>(file)),
                                     std::istreambuf_iterator<char>());
  return json::from_msgpack(msgpackObject);
}

/**
 * @brief Saves a json object as .json file.
 *
 * @param filePath The path of the file to be saved, without the extension.
 * @param jObject The json object to be saved.
 */
void saveJSON(const std::string& filePath, const json& jObject) {
  std::ofstream file(filePath + ".json");
  file << jObject;
  file.close();
}

/**
 * @brief Saves a json object as .msgpack file.
 *
 * @param filePath The path of the file to be saved, without the extension.
 * @param jObject The json object to be saved.
 */
void saveMsgPack(const std::string& filePath, const json& jObject) {
  std::ofstream file(filePath + ".msgpack", std::ios::out | std::ios::binary);
  auto msgpackObject = json::to_msgpack(jObject);
  file.write(reinterpret_cast<char*>(&msgpackObject[0]), msgpackObject.size() * sizeof(uint8_t));
  file.close();
}

/**
 * @brief Saves a json object as .json file.
 * @note Does some agent specific things.
 * @todo Review code and refactor.
 *
 * @param filePath The path of the file to be saved, without the extension.
 * @param jObject The json object to be saved.
 */
void saveAsJSON(const std::string& filePath, const json& jObject) {
  json existing;
  std::ifstream existingFile(filePath + ".json");
  if (existingFile.fail()) {
    existing["agents"] = json::array();
  } else {
    existingFile >> existing;
  }
  existingFile.close();

  for (auto& agent : jObject["agents"]) {
    existing["agents"].push_back(agent);
  }
  saveJSON(filePath, existing);
}

/**
 * @brief Saves a json object as .msgpack file.
 * @note Does some agent specific things.
 * @todo Review code and refactor.
 *
 * @param filePath The path of the file to be saved, without the extension.
 * @param jObject The json object to be saved.
 */
void saveAsMsgPack(const std::string& filePath, const json& jObject) {
  json existing;
  std::ifstream existingFile(filePath + ".msgpack", std::ios::binary);
  if (existingFile.fail()) {
    existing["agents"] = json::array();
  } else {
    std::vector<uint8_t> vMsgpack((std::istreambuf_iterator<char>(existingFile)),
                                  std::istreambuf_iterator<char>());
    existing = json::from_msgpack(vMsgpack);
  }
  existingFile.close();

  for (auto& agent : jObject["agents"]) {
    existing["agents"].push_back(agent);
  }
  saveMsgPack(filePath, existing);
}

}  // namespace proseco_planning::util