/**
 * @file utilities.h
 * @brief A collection of utility functions that are used throughout the library.
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include <sstream>
#include <string>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include "proseco_planning/util/alias.h"

namespace proseco_planning::util {

bool hasEnding(const std::string& fullString, const std::string& ending);

std::string toStringPrecision(const float number, const unsigned int precision);

std::string actionSetToString(const ActionSet& actionSet);

/**
 * @brief Creates a string using the address of the pointer.
 *
 * @tparam T The type of the pointer.
 * @param pointer The pointer.
 * @return std::string The address of the pointer as a string.
 */
template <typename T>
std::string pointerToString(const T* pointer) {
  std::stringstream ss;
  ss << pointer;
  return ss.str();
}

std::string padNumber(const unsigned int number, const unsigned int length);

void createConfig(const std::string& optionsFilePath, const std::string& scenarioFilePath);

json loadJSON(const std::string& filePath);

json loadMsgPackToJSON(const std::string& filePath);

void saveJSON(const std::string& filePath, const json& jObject);

void saveMsgPack(const std::string& filePath, const json& jObject);

void saveAsJSON(const std::string& filePath, const json& jObject);

void saveAsMsgPack(const std::string& filePath, const json& jObject);

}  // namespace proseco_planning::util
