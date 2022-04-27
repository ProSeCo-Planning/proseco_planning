#include "proseco_planning/exporters/msgPackExporter.h"

#include <vector>

#include "proseco_planning/exporters/exporter.h"
#include "proseco_planning/exporters/jsonExporter.h"
#include "proseco_planning/util/utilities.h"

namespace proseco_planning {

/**
 * @brief Constructs a new Msg Pack Exporter object.
 *
 * @param outputPath The path to the output folder.
 */
MsgPackExporter::MsgPackExporter(const std::string& outputPath) : JSONExporter{outputPath} {}

/**
 * @brief Exports the data for the specific step to the output folder.
 *
 * @param step The current step.
 * @param exportType The type of the export.
 */
void MsgPackExporter::writeData(const int step, const ExportType exportType) {
  std::string exportPath;
  json data = m_data[exportType];

  switch (exportType) {
    case ExportType::EXPORT_SINGLESHOTPLAN: {
      exportPath = m_path + "/" + m_fileNames[exportType] + std::to_string(step);
      break;
    }
    default: {
      exportPath = m_path + "/" + m_fileNames[exportType];
    }
  }

  util::saveMsgPack(exportPath, data);
}
}  // namespace proseco_planning
