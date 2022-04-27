/**
 * @file collisionCheckerCircleApproximation.h
 * @brief This file describes the CollisionCheckerCircleApproximation class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once
#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

#include "collisionChecker.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/scenarioOptions.h"

namespace proseco_planning {
class Trajectory;

/**
 * @brief The Circle struct stores the basic information of a circle.
 * @details Is used for collision checking.
 */
struct Circle {
  Circle(const float x, const float y, const float radius) : m_x(x), m_y(y), m_radius(radius) {}

  /// The x Position of the center of the circle -- [m].
  float m_x;

  /// The y Position of the center of the circle -- [m].
  float m_y;

  /// Radius of the circle -- [m].
  float m_radius;
};

/**
 * @brief The RectangleDecomposition struct stores information regarding the decomposition of
 * rectangles into circles.
 * @details The information is only dependent on the shape of the rectangle and thus is constant
 * over the length of a trajectory.
 */
struct RectangleDecomposition {
  /// The radius of the disk(s) -- [m].
  float m_radius;

  /// The distance between the disks -- [m].
  float m_distance;

  /// The number of disks used for the decomposition.
  unsigned int m_nDisks;
};

/**
 * @brief The struct for extracting the relevant vehicle data.
 */
struct Rectangle {
  /**
   * @brief Construct a new Vehicle Data object.
   *
   * @param agent The agent, and respectively the vehicle the data should be extracted from.
   */
  explicit Rectangle(const Vehicle& vehicle)
      : m_positionX(vehicle.m_positionX),
        m_positionY(vehicle.m_positionY),
        m_heading(vehicle.m_heading),
        m_width(vehicle.m_width),
        m_length(vehicle.m_length){};

  /**
   * @brief Constructs a new Vehicle Data object.
   *
   * @param obstacle The obstacle the data should be extracted from.
   */
  explicit Rectangle(const config::Obstacle& obstacle)
      : m_positionX(obstacle.position_x),
        m_positionY(obstacle.position_y),
        m_heading(obstacle.heading),
        m_width(obstacle.width),
        m_length(obstacle.length){};

  /// The x Position of the vehicle -- [m].
  float m_positionX;

  /// The y Position of the vehicle -- [m].
  float m_positionY;

  /// The heading of the vehicle -- [°].
  float m_heading;

  /// The width of the vehicle -- [m].
  float m_width;

  /// The length of the vehicle -- [m].
  float m_length;

  RectangleDecomposition decompose(unsigned int n) const;
};

/**
 * @brief The collision checker class.
 * @details The collision checker based on a circle approximation of the vehicles.
 */

class CollisionCheckerCircleApproximation : public CollisionChecker {
 public:
  using CollisionChecker::CollisionChecker;

  bool collision(const Vehicle& vehicle0, const Trajectory& trajectory0, const Vehicle& vehicle1,
                 const Trajectory& trajectory1) override;

  bool collision(const Vehicle& vehicle, const Trajectory& trajectory,
                 const Vehicle& obstacle) override;

  bool collision(const Vehicle& vehicle, const Trajectory& trajectory,
                 const config::Obstacle& obstacle) override;

  bool collision(const Vehicle& vehicle0, const Vehicle& vehicle1) override;

  bool collision(const Vehicle& vehicle, const config::Obstacle& obstacle) override;

  /** Indicator if export is active.
   *  @note Set ONLY FOR TEST CASES!
   *  @todo Refactor
   */
  bool m_exporter{false};

  /// Base file name (csv-ending is added within initializeExporter).
  std::string m_fileName{"collisionCheckCircleApproximation"};

 private:
  std::vector<RectangleDecomposition> calculateRectangleDecompositions(
      const Rectangle& vehicleRectangle);

  bool collisionCurrentState(const Rectangle& vehicleRectangle0, const Rectangle& vehicleRectangle1,
                             int trajectoryPoint,
                             const std::vector<RectangleDecomposition>& rectangleDecompositions0,
                             const std::vector<RectangleDecomposition>& rectangleDecompositions1);

  bool calculateCollision(const Rectangle& vehicleRectangle0, const Rectangle& vehicleRectangle1,
                          const RectangleDecomposition& decomposition0,
                          const RectangleDecomposition& decomposition1);

  static std::vector<Circle> calculateDiskCenter(const Rectangle& vehicleRectangle,
                                                 const RectangleDecomposition& decomposition);

  static void setTrajectoryData(Rectangle& vehicleRectangle, const Trajectory& trajectory,
                                size_t i);

  bool circleCollision(const std::vector<Circle>& vehicleCenter0,
                       const std::vector<Circle>& vehicleCenter1) const;

  bool circleCollision(const Circle& circle0, const Circle& circle1) const;

  void initializeExporter(int trajectoryPoint);

  void writeExporter(const Rectangle& vehicleRectangle0, const std::vector<Circle>& vehicleCenter0,
                     const Rectangle& vehicleRectangle1, const std::vector<Circle>& vehicleCenter1,
                     bool collision);
  void closeExporter();

  /// The number of disks for approximating the vehicle shape with the respective level of detail.
  static std::vector<int> nDisks;

  /// The file stream used for exporting the collision checker data.
  std::fstream m_fileStream;
};
}  // namespace proseco_planning
