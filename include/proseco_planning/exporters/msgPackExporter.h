/**
 * @file msgPackExporter.h
 * @brief This file defines the export of trajectory data to msgpack.
 * @copyright Copyright (c) 2021
 *
 */

#include <string>

#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include "exporter.h"
#include "jsonExporter.h"

namespace proseco_planning {

/**
 * @brief MsgPackExporter class: export trajectory data to msgpack
 *
 */
class MsgPackExporter : public JSONExporter {
 public:
  explicit MsgPackExporter(const std::string& outputPath = "");

  void writeData(const int step, const ExportType exportType) override;
};
}  // namespace proseco_planning