/**
 * @file json.h
 * @brief A collection of methods that are used for JSON serialization.
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include <memory>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

/**
 * @brief The namespace of the json library.
 *
 */
namespace nlohmann {
/**
 * @brief Function to allow conversion of a shared_ptr<T> to a JSON object.
 * @details Gets called by the json constructor of the nlohmann json library.
 *
 * @tparam T The type of the object to be converted.
 * @param j The JSON object to be filled.
 * @param object The object to be converted.
 */
template <typename T>
struct adl_serializer<std::shared_ptr<T>> {
  static void to_json(json& j, const std::shared_ptr<T>& object) {
    if (object) {
      j = *object;
    } else {
      j = nullptr;
    }
  }
};

/**
 * @brief Function to allow conversion of a unique_ptr<T> to a JSON object.
 * @details Gets called by the json constructor of the nlohmann json library.
 *
 * @tparam T The type of the object to be converted.
 * @param j The JSON object to be filled.
 * @param object The object to be converted.
 */
template <typename T>
struct adl_serializer<std::unique_ptr<T>> {
  static void to_json(json& j, const std::unique_ptr<T>& object) {
    if (object) {
      j = *object;
    } else {
      j = nullptr;
    }
  }
};

}  // namespace nlohmann

namespace proseco_planning {

/**
 * @brief Function to allow conversion of a T* to a JSON object.
 *
 * @tparam T The type of the object to be converted.
 * @param j The JSON object to be filled.
 * @param object The object to be converted.
 */
template <typename T>
void to_json(json& j, const T* object) {
  if (object) {
    j = *object;
  } else {
    j = nullptr;
  }
}

}  // namespace proseco_planning