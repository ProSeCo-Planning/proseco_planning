#include "proseco_planning/exporters/exporter.h"

#include <iostream>

#include "proseco_planning/exporters/jsonExporter.h"
#include "proseco_planning/exporters/msgPackExporter.h"

namespace proseco_planning {

/** */
std::string const Exporter::m_fileNames[] = {"trajectory_annotated", "single_shot_",
                                             "irl_trajectory"};

/**
 * @brief Constructs a new Exporter object.
 *
 * @param outputPath The path to the output folder.
 */
Exporter::Exporter(const std::string& outputPath) : m_path{outputPath} {}

/**
 * @brief Factory for the different exporter types.
 *
 * @param outputPath The path to the output folder.
 * @param format The output format.
 * @return std::unique_ptr<Exporter> The pointer to the exporter.
 */
std::unique_ptr<Exporter> Exporter::createExporter(const std::string& outputPath,
                                                   config::exportFormat format) {
  if (format == config::exportFormat::MSGPACK) {
    return std::make_unique<MsgPackExporter>(outputPath);
  } else if (format == config::exportFormat::JSON) {
    return std::make_unique<JSONExporter>(outputPath);
  } else {
    throw std::invalid_argument("Unknown export format");
  }
}
}  // namespace proseco_planning
