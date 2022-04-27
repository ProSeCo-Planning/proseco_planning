/**
 * @file costModel.h
 * @brief This file defines the CostModel class.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <string>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace proseco_planning {
class Desire;
class Trajectory;
class Vehicle;
namespace config {
struct CostModel;
}  // namespace config

/**
 * @brief Cost Model class:
 * Factory for creating different types of cost models.
 */

class CostModel {
 public:
  explicit CostModel(const config::CostModel& costModel);

  /// The virtual destructor.
  virtual ~CostModel() = default;

  static std::shared_ptr<CostModel> createCostModel(const config::CostModel& costModel);

  virtual float calculateActionCost(const Trajectory& trajectory) = 0;

  void initializeMaximumStatePotential(const Desire& desire, const Vehicle& vehicle);

  virtual float updateStatePotential(const Desire& desire, const Vehicle& vehicle);

  float updateStateReward(float currentPotential, float oldPotential);

  virtual float calculateCost(const Desire& desire, const Vehicle& vehicle,
                              const Vehicle& vehiclePreviousStep, bool collision, bool invalid,
                              const Trajectory& trajectory);

  virtual float calculatePotentialDeviation(const Desire& desire, const Vehicle& vehicle) const = 0;

  virtual float calculateCooperativeCost(const Desire& desire, const Vehicle& vehicle,
                                         const Trajectory& trajectory, bool collision, bool invalid,
                                         int numberOfAgents, float egoReward,
                                         float cooperationFactor) const = 0;

  virtual float calculateStateCost(const Desire& desire, const Vehicle& vehicle, bool collision,
                                   bool invalid) = 0;

  float costSafeRange(const Vehicle& vehicle0, const Vehicle& vehicle1) const;

  virtual float costAccelerationX(const Trajectory& trajectory) const;

  virtual float costAccelerationY(const Trajectory& trajectory) const;

  virtual float costLaneChange(const Trajectory& trajectory) const;

  virtual float costInvalidAction(const Trajectory& trajectory) const;

  virtual float potentialVelocityDeviation(const Desire& desire, const Vehicle& vehicle) const;

  virtual float potentialLaneDeviation(const Desire& desire, const Vehicle& vehicle) const;

  virtual float potentialLaneCenterDeviation(const Desire& desire, const Vehicle& vehicle) const;

  /// The weight for longitudinal acceleration (action)
  float m_wAccelerationX{0};

  /// The weight for lateral acceleration (action)
  float m_wAccelerationY{0};

  /// The weight for lane changes (action)
  float m_wLaneChange{0};

  /// The cost for an invalid action
  float m_costInvalidAction{0};

  /// The weight for velocity deviations (state)
  float m_wVelocityDeviation{0};

  /// The weight for lane deviations (state)
  float m_wLaneDeviation{0};

  /// The weight for lane center deviations (state)
  float m_wLaneCenterDeviation{0};

  /// The cost for a collision state
  float m_costCollision{0};

  /// The cost for an invalid state
  float m_costInvalidState{0};

  /// The reward for a terminal state
  float m_rewardTerminal{0};

  /// The minimum cost for entering the safety range. Costs increases exponentially till the
  /// collision of two vehicles. The safety range is absolute v(m/s) * 1.
  float m_costEnterSafeRange{0};

  /// The final potential is based on all deviations of the current state from the desired state
  float m_finalPotential{0};

  /// The type of cost model
  std::string m_type;
};
}  // namespace proseco_planning
