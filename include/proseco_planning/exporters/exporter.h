/**
 * @file exporter.h
 * @brief This file defines the interface for the specific exporter classes.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "proseco_planning/config/outputOptions.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {
class Node;

/**
 * @brief The export type is an enum that defines the type of export.
 *
 */
enum ExportType : int {
  /// Whether or not to export the trajectory.
  EXPORT_TRAJECTORY = 0,
  /// Whether or not to export the single shot plan.
  EXPORT_SINGLESHOTPLAN = 1,
  /// Whether or not to export the IRL trajectory.
  EXPORT_IRL_TRAJECTORY = 2
};

/**
 * @brief Exporter base class: Defines the interface for specific exporter classes.
 *
 */
class Exporter {
 public:
  explicit Exporter(const std::string& outputPath = "");

  /// The virtual destructor.
  virtual ~Exporter() = default;

  static std::unique_ptr<Exporter> createExporter(const std::string& outputPath,
                                                  config::exportFormat format);

  /// Writes trajectory data for a single MCTS step.
  virtual void exportTrajectory(const Node* const rootNode, const ActionSet& currentActionSet,
                                const int step) = 0;

  /// Writes trajectory and irl-specific data for a single MCTS step.
  virtual void exportIrlTrajectory(const Node* const rootNode, const ActionSet& currentActionSet,
                                   const int step) = 0;

  /// Writes the single shot trajectory. Single shot is planned without timeShift.
  virtual void exportSingleShot(Node* const rootNode, const ActionSetSequence& actionSetSequence,
                                const int step) = 0;

  /// Writes the data.
  virtual void writeData(const int step, const ExportType exportType) = 0;

 protected:
  /// The file extension is added within the export method.
  static std::string const m_fileNames[];

  /// The output path.
  std::string m_path;
};
}  // namespace proseco_planning