#include "proseco_planning/collision_checker/collisionCheckerCircleApproximation.h"

#include <memory>

#include "proseco_planning/trajectory/trajectory.h"

namespace proseco_planning {

std::vector<int> CollisionCheckerCircleApproximation::nDisks{1, 3, 7};

/**
 * @brief Sets the relevant data of the trajectory on vehicleRectangle so that it can be used for
 * collision checking.
 *
 * @param vehicleRectangle The data that describes the current state of the vehicle/obstacle.
 * @param trajectory The trajectory that the vehicle is driving.
 * @param i The index of the fraction of the trajectory, indicating a specific point along the
 * trajectory.
 */
void CollisionCheckerCircleApproximation::setTrajectoryData(Rectangle& vehicleRectangle,
                                                            const Trajectory& trajectory,
                                                            size_t i) {
  vehicleRectangle.m_positionX = trajectory.m_sPosition[i];
  vehicleRectangle.m_positionY = trajectory.m_dPosition[i];
  vehicleRectangle.m_heading   = trajectory.m_heading[i];
}

/**
 * @brief Checks for the collision with another vehicle.
 * @param vehicle0 One vehicle.
 * @param vehicle1 The other vehicle.
 * @return True, if a collision is occuring between the two vehicles, false otherwise.
 */
bool CollisionCheckerCircleApproximation::collision(const Vehicle& vehicle0,
                                                    const Vehicle& vehicle1) {
  Rectangle vehicleRectangle0{vehicle0};
  Rectangle vehicleRectangle1{vehicle1};

  // Compute the information about the circles that stays the same for every
  // trajectory point
  const auto& decomp0 = calculateRectangleDecompositions(vehicleRectangle0);
  const auto& decomp1 = calculateRectangleDecompositions(vehicleRectangle1);

  return collisionCurrentState(vehicleRectangle0, vehicleRectangle1, 0, decomp0, decomp1);
}

/**
 * @brief Checks for the collision with an obstacle.
 * @param vehicle One vehicle.
 * @param obstacle The obstacle.
 * @return True, if a collision is occuring between, false otherwise.
 */
bool CollisionCheckerCircleApproximation::collision(const Vehicle& vehicle,
                                                    const config::Obstacle& obstacle) {
  Rectangle vehicleRectangle{vehicle};
  Rectangle obstacleRectangle{obstacle};

  // Compute the information about the circles that stays the same for every trajectory point
  const auto& vehicleDecomp  = calculateRectangleDecompositions(vehicleRectangle);
  const auto& obstacleDecomp = calculateRectangleDecompositions(obstacleRectangle);

  return collisionCurrentState(vehicleRectangle, obstacleRectangle, 0, vehicleDecomp,
                               obstacleDecomp);
}

/**
 * @brief Checks for the collision with another vehicle.
 * @param vehicle0 One vehicle.
 * @param vehicle1 The other vehicle.
 * @return True, if a collision is occuring between the two vehicles, false otherwise.
 */
bool CollisionCheckerCircleApproximation::collision(const Vehicle& vehicle0,
                                                    const Trajectory& trajectory0,
                                                    const Vehicle& vehicle1,
                                                    const Trajectory& trajectory1) {
  Rectangle vehicleRectangle0{vehicle0};
  Rectangle vehicleRectangle1{vehicle1};

  /// Compute the information about the circles that stays the same for every
  /// trajectory point
  const auto& decomp0 = calculateRectangleDecompositions(vehicleRectangle0);
  const auto& decomp1 = calculateRectangleDecompositions(vehicleRectangle1);

  bool collision = false;
  for (size_t i = 0; i <= trajectory0.getFractionIndex(); ++i) {
    /// extract relevant data for collision checking
    setTrajectoryData(vehicleRectangle0, trajectory0, i);
    setTrajectoryData(vehicleRectangle1, trajectory1, i);

    collision = collisionCurrentState(vehicleRectangle0, vehicleRectangle1, i, decomp0, decomp1);

    // Break after the first collision check returns true
    if (collision) {
      break;
    }
  }

  return collision;
}
/**
 * @brief Checks for the collision with static obstacle.
 * @param vehicle0 The vehicle.
 * @param obstacle The static obstacle.
 * @return True, if a collision with a static obstacle is occuring, false otherwise.
 */
bool CollisionCheckerCircleApproximation::collision(const Vehicle& vehicle,
                                                    const Trajectory& trajectory,
                                                    const config::Obstacle& obstacle) {
  Rectangle vehicleRectangle{vehicle};
  Rectangle obstacleRectangle{obstacle};

  /// Compute the information about the circles that stays the same for every trajectory point
  const auto& vehicleDecomp  = calculateRectangleDecompositions(vehicleRectangle);
  const auto& obstacleDecomp = calculateRectangleDecompositions(obstacleRectangle);

  bool collision = false;
  for (size_t i = 0; i <= trajectory.getFractionIndex(); ++i) {
    /// extract relevant data for collision checking
    setTrajectoryData(vehicleRectangle, trajectory, i);

    collision = collisionCurrentState(vehicleRectangle, obstacleRectangle, i, vehicleDecomp,
                                      obstacleDecomp);
    // Break after the first collision check returns true
    if (collision) {
      break;
    }
  }

  return collision;
}

/**
 * @brief Checks for the collision with moving obstacle.
 * @param vehicle0 The vehicle.
 * @param obstacle The moving obstacle (vehicle).
 * @return True, if a collision with a moving obstacle is occuring, false otherwise.
 */
bool CollisionCheckerCircleApproximation::collision(const Vehicle& vehicle,
                                                    const Trajectory& trajectory,
                                                    const Vehicle& obstacle) {
  config::Obstacle configObstacle{0,
                                  false,
                                  obstacle.m_positionX,
                                  obstacle.m_positionY,
                                  obstacle.m_heading,
                                  obstacle.m_length,
                                  obstacle.m_width,
                                  0.f,
                                  0.f,
                                  0.f,
                                  0.f,
                                  0.f};
  return collision(vehicle, trajectory, configObstacle);
}

bool CollisionCheckerCircleApproximation::collisionCurrentState(
    const Rectangle& vehicleRectangle0, const Rectangle& vehicleRectangle1, int trajectoryPoint,
    const std::vector<RectangleDecomposition>& decompositions0,
    const std::vector<RectangleDecomposition>& decompositions1) {
  if (m_exporter) {
    initializeExporter(trajectoryPoint);
  }

  bool collision = false;
  for (auto rD0i = decompositions0.cbegin(), end_m0 = decompositions0.cend(),
            rD1i = decompositions1.cbegin(), end_m1 = decompositions1.cend();
       rD0i != end_m0 || rD1i != end_m1;) {
    collision = calculateCollision(vehicleRectangle0, vehicleRectangle1, *rD0i, *rD1i);
    if (!collision) {
      // no collision has occured break out of the collision check
      break;
    }
    // select the next decomposition for the collision check
    ++rD0i;
    ++rD1i;
  }
  if (m_exporter) {
    closeExporter();
  }

  return collision;
}

bool CollisionCheckerCircleApproximation::calculateCollision(
    const Rectangle& vehicleRectangle0, const Rectangle& vehicleRectangle1,
    const RectangleDecomposition& decomposition0, const RectangleDecomposition& decomposition1)

{
  /// level 1 check: only n_DisksLowLevel disks for shape (n=1)
  std::vector<Circle> disks0{calculateDiskCenter(vehicleRectangle0, decomposition0)};
  std::vector<Circle> disks1{calculateDiskCenter(vehicleRectangle1, decomposition1)};

  bool collision = circleCollision(disks0, disks1);

  // Export circles and collision yes/no
  if (m_exporter) {
    writeExporter(vehicleRectangle0, disks0, vehicleRectangle1, disks1, collision);
  }

  return collision;
}
/**
 * @brief Calculates the center of the disks.
 */
std::vector<Circle> CollisionCheckerCircleApproximation::calculateDiskCenter(
    const Rectangle& vehicleRectangle, const RectangleDecomposition& decomposition) {
  std::vector<Circle> disks;
  disks.reserve(decomposition.m_nDisks);
  /// Distance between origin of vehicle COS and center of first circle == radius/2
  for (unsigned int i = 0; i < decomposition.m_nDisks; ++i) {
    float x = vehicleRectangle.m_positionX +
              (decomposition.m_distance / 2.0f + i * decomposition.m_distance) *
                  std::cos(vehicleRectangle.m_heading);
    float y = vehicleRectangle.m_positionY +
              (decomposition.m_distance / 2.0f + i * decomposition.m_distance) *
                  std::sin(vehicleRectangle.m_heading);

    disks.emplace_back(x, y, decomposition.m_radius);
  }

  return disks;
}
/**
 * @brief Determines if two vectors of circles are in collision.
 * @param vehicleCenter0 Vector of circles enclosing vehicle0.
 * @param vehicleCenter1 Vector of circles enclosing vehicle1.
 * @return True, if a collision occurs, false otherwise.
 */
bool CollisionCheckerCircleApproximation::circleCollision(
    const std::vector<Circle>& vehicleCenter0, const std::vector<Circle>& vehicleCenter1) const {
  for (const auto& circle0 : vehicleCenter0) {
    for (const auto& circle1 : vehicleCenter1) {
      if (circleCollision(circle0, circle1)) {
        return true;
      }
    }
  }
  return false;
}
/**
 * @brief Determines if two circles are in collision.
 * @param circle0
 * @param circle1
 * @return True, if a collision occurs, false otherwise.
 */
bool CollisionCheckerCircleApproximation::circleCollision(const Circle& circle0,
                                                          const Circle& circle1) const {
  // calculate the distance between the center of two circles
  float deltaX{circle0.m_x - circle1.m_x};
  float deltaY{circle0.m_y - circle1.m_y};
  float minDistance{circle0.m_radius + circle1.m_radius};

  // distance of the circles squared
  float circleDistance_2{deltaX * deltaX + deltaY * deltaY};
  // required distance of the circles to be considered safe
  float safeDistance_2{(minDistance + m_safetyDistance) * (minDistance + m_safetyDistance)};

  return circleDistance_2 <= safeDistance_2;
}

/**
 * @brief Calculates rectangle decompositions based on the number of disks specified in
 * CollisionCheckerCircleApproximation::nDisks.
 *
 * @param vehicleRectangle The vehicle data the should be decomposed.
 * @return std::vector<RectangleDecomposition> The decompositions of the vehicle data.
 */
std::vector<RectangleDecomposition>
CollisionCheckerCircleApproximation::calculateRectangleDecompositions(
    const Rectangle& vehicleRectangle) {
  std::vector<RectangleDecomposition> rectangleDecompositions;

  for (const auto& disks : CollisionCheckerCircleApproximation::nDisks) {
    rectangleDecompositions.push_back(vehicleRectangle.decompose(disks));
  }

  return rectangleDecompositions;
}

/**
 * @brief Determines the radius and the distance for the decomposition of the given rectangle with
 * the given number of disks.
 *
 * @param n The number of circles the rectangle shall be composed of.
 * @return RectangleDecomposition
 */
RectangleDecomposition Rectangle::decompose(unsigned int n) const {
  // https://ieeexplore.ieee.org/document/5547976 - Fast Collision Checking for Intelligent Vehicle
  // Motion Planning
  RectangleDecomposition decomposition;

  decomposition.m_radius = std::sqrt(m_length * m_length / 4.f / (n * n) + m_width * m_width / 4.f);

  decomposition.m_distance =
      2.f * std::sqrt(decomposition.m_radius * decomposition.m_radius - m_width * m_width / 4.f);

  decomposition.m_nDisks = n;

  return decomposition;
}
/**
 * @brief Belongs to a group of functions for the export of the collision checking to CSV.
 * @note ONLY FOR TEST CASES.
 */
void CollisionCheckerCircleApproximation::initializeExporter(int trajectoryPoint) {
  // change filename
  std::string currentFilename{m_fileName + std::to_string(trajectoryPoint) + ".csv"};

  m_fileStream.open(currentFilename, std::fstream::out);
  for (size_t i = 0; i < 2; ++i) {
    m_fileStream << "circleCenterX" << std::to_string(i) << ","
                 << "circleCenterY" << std::to_string(i) << ","
                 << "radius" << std::to_string(i) << ","
                 << "vehicle center x" << std::to_string(i) << ","
                 << "vehicle center y" << std::to_string(i) << ","
                 << "vehicle heading " << std::to_string(i) << ","
                 << "vehicle length" << std::to_string(i) << ","
                 << "vehicle width" << std::to_string(i) << ",";
  }

  m_fileStream << "collision" << std::endl;
}
/**
 * @brief Belongs to a group of functions for the export of the collision checking to csv.
 * @note ONLY FOR TEST CASES.
 */
void CollisionCheckerCircleApproximation::writeExporter(const Rectangle& vehicleRectangle0,
                                                        const std::vector<Circle>& vehicleCenter0,
                                                        const Rectangle& vehicleRectangle1,
                                                        const std::vector<Circle>& vehicleCenter1,
                                                        bool collision) {
  for (size_t i = 0; i < vehicleCenter0.size(); ++i) {
    m_fileStream << vehicleCenter0[i].m_x << "," << vehicleCenter0[i].m_y << ","
                 << vehicleCenter0[i].m_radius << "," << vehicleRectangle0.m_positionX << ","
                 << vehicleRectangle0.m_positionY << "," << vehicleRectangle0.m_heading << ","
                 << vehicleRectangle0.m_length << "," << vehicleRectangle0.m_width << ","

                 << vehicleCenter1[i].m_x << "," << vehicleCenter1[i].m_y << ","
                 << vehicleCenter1[i].m_radius << "," << vehicleRectangle1.m_positionX << ","
                 << vehicleRectangle1.m_positionY << "," << vehicleRectangle1.m_heading << ","
                 << vehicleRectangle1.m_length << "," << vehicleRectangle1.m_width << ","
                 << collision << std::endl;
  }
}
/**
 * @brief Belongs to a group of functions for the export of the collision checking to csv.
 * @note ONLY FOR TEST CASES.
 */
void CollisionCheckerCircleApproximation::closeExporter() { m_fileStream.close(); }
}  // namespace proseco_planning
