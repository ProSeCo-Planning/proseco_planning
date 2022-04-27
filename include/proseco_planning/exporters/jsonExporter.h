/**
 * @file jsonExporter.h
 * @brief This file defines the export of trajectory data to JSON.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <string>
#include <vector>

#include "nlohmann/json.hpp"
using json = nlohmann::json;
#include "exporter.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Node;

/**
 * @brief JSONExporter class: export trajectory data to JSON
 *
 */
class JSONExporter : public Exporter {
 public:
  explicit JSONExporter(const std::string& outputPath = "");

  void exportTrajectory(const Node* const rootNode, const ActionSet& currentActionSet,
                        const int step) override;

  void exportIrlTrajectory(const Node* const rootNode, const ActionSet& currentActionSet,
                           const int step) override;

  void exportSingleShot(Node* const rootNode, const ActionSetSequence& actionSetSequence,
                        const int step) override;

  void writeData(const int step, const ExportType exportType) override;

 protected:
  void addStep(const Node* const rootNode, const ActionSet& currentActionSet, const int step,
               const ExportType exportType, const float singleShotOffset);

  /// The data members for the complete trajectory and for single shot plan.
  json m_data[3];

  /// The total step counter for the trajectory.
  unsigned int m_ticks;
};
}  // namespace proseco_planning