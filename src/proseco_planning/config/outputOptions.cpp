#include "proseco_planning/config/outputOptions.h"

#include <sys/types.h>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <map>

#include "nlohmann/json.hpp"

namespace proseco_planning::config {

/**
 * @brief Exports the parameters of the outputOptions object to JSON.
 *
 * @return json The parameters.
 */
json OutputOptions::toJSON() const {
  json jOutputOptions;
  jOutputOptions["export_format"] = export_format;
  jOutputOptions["export"]        = export_types;
  jOutputOptions["output_path"]   = output_path;
  return jOutputOptions;
}

/**
 * @brief Checks for the existence of an export type.
 *
 * @param type
 * @return bool True, if export type exists, false otherwise.
 */
bool OutputOptions::hasExportType(const std::string& type) const {
  return std::find(export_types.begin(), export_types.end(), type) != export_types.end();
}

/**
 * @brief Returns a new outputOptions object created from the parameters of the JSON file.
 *
 * @param jOutputOptions The JSON file.
 * @return OutputOptions
 */
OutputOptions OutputOptions::fromJSON(const json& jOutputOptions) {
  std::string outputPath = generateOutputPath(jOutputOptions["output_path"].get<std::string>());

  OutputOptions outputOptions =
      OutputOptions(jOutputOptions["export_format"].get<config::exportFormat>(),
                    jOutputOptions["export"].get<std::vector<std::string>>(), outputPath);
  return outputOptions;
}

/**
 * @brief Prepares output path and folder structure for data export.
 *
 * @param path The file path to export.
 * @return std::string The output path.
 */
std::string OutputOptions::generateOutputPath(const std::string& path) {
  std::string outputPath = path;
  if (outputPath.length() > 0) {
    outputPath = formatOutputPath(outputPath);
  } else {
    outputPath = "~/.ros";
    outputPath = formatOutputPath(outputPath);
    outputPath = outputPath + generateFolderName();
  }
  return outputPath;
}

/**
 * @brief Generates foldername with timestamp.
 *
 * @return std::string The foldername.
 */
std::string OutputOptions::generateFolderName() {
  time_t t;
  struct tm* ptr;
  char stamp[30];
  time(&t);
  ptr = localtime(&t);
  strftime(stamp, 20, "%Y-%m-%d_%H-%M-%S", ptr);
  std::string timestamp = stamp;
  return "/" + timestamp + "_MCTS";
}

/**
 * @brief Formats specified output path, replaces '~' with HOME variable und removes ’/’ in the end
 * if necessary
 *
 * @param outputPath The raw output path.
 * @return std::string The formated output path.
 */
std::string OutputOptions::formatOutputPath(std::string outputPath) {
  if (outputPath.at(0) == '~') {
    std::string home = getenv("HOME");
    outputPath.erase(0, 1);
    outputPath = home + outputPath;
  }

  if (outputPath.back() == '/') {
    outputPath.pop_back();
  }
  return outputPath;
}
}  // namespace proseco_planning::config