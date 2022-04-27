/**
 * @file outputOptions.h
 * @brief This file defines the output configuration.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace proseco_planning::config {
/// The enum for setting the export type flags
enum class exportFormat {
  NONE,
  MSGPACK,
  JSON,
};

/// The export type json serialization. none->false for if-checks
NLOHMANN_JSON_SERIALIZE_ENUM(exportFormat, {
                                               {exportFormat::NONE, "none"},
                                               {exportFormat::MSGPACK, "msgpack"},
                                               {exportFormat::JSON, "json"},
                                           })

struct OutputOptions {
  /// The flag that indicates if exported data is of type json or msgpack
  const exportFormat export_format;
  /// The list that indicates what files should be exported
  const std::vector<std::string> export_types;
  /// The path of the output folder
  const std::string output_path;

  /**
   * @brief Constructs a new Output Options object from output specifying parameters.
   *
   * @param export_format
   * @param export_types
   * @param output_path
   */
  OutputOptions(const exportFormat export_format, std::vector<std::string> export_types,
                std::string output_path)
      : export_format(export_format), export_types(export_types), output_path(output_path) {}

  json toJSON() const;

  static OutputOptions fromJSON(const json& jOutputOptions);

  bool hasExportType(const std::string& type) const;

  static std::string generateOutputPath(const std::string& path);

  static std::string generateFolderName();

  static std::string formatOutputPath(std::string outputPath);
};
}  // namespace proseco_planning::config