/**
 * @file action.h
 * @brief This file defines the Action class as well as the ActionNoise struct.
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <vector>

#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/util/alias.h"

namespace proseco_planning {

class ActionSpace;
class Vehicle;

/**
 * @brief This struct defines the estimations for the behavior of the vehicle when noise was added
 * to the action.
 */
struct ActionNoise {
  /// The likelihood of Y when noise was added to action.
  float m_likelihoodY{0.0f};

  /// The likelihood of Vx when noise was added to action.
  float m_likelihoodVx{0.0f};

  /// The expected value of Y.
  float m_muY{0.0f};

  /// The expected value of Vx.
  float m_muVx{0.0f};

  /// The std. deviation of Y.
  float m_sigmaY{0.0f};

  /// The std. deviation of Vx.
  float m_sigmaVx{0.0f};
};

/**
 * @brief The Action class is a generic class for actions.
 */
class Action {
 public:
  explicit Action(ActionClass actionClass);

  Action(float velocityChange, float lateralChange);

  Action(ActionClass actionClass, float accelerationX, float accelerationY);

  void updateActionClass(const ActionSpace& actionSpace, const Vehicle& vehicle);

  static float getSimilarity(const ActionPtr& x, const ActionPtr& y, const float gamma);

  static float getSimilarity(const ActionPtr& x, const ActionPtr& y);

  float getSquaredDistance(const ActionPtr& action = nullptr) const;

  float getDistance(const ActionPtr& action = nullptr) const;

  /// The class of the action.
  ActionClass m_actionClass;

  /// The longitudinal velocity change.
  const float m_velocityChange;

  /// The lateral position change.
  const float m_lateralChange;

  /// The longitudinal acceleration.
  const float m_accelerationX;

  /// The lateral acceleration.
  const float m_accelerationY;

  /// The validity of an action given the current state of the vehicle.
  bool m_invalidAction{false};

  /// The noise added to the action.
  ActionNoise noise;

  /// The probability for selecting this action. Currently only used for exp_q sampling final
  /// selection.
  float m_selectionLikelihood{0.0f};

  /// The weights for selecting this action.
  std::vector<float> m_selectionWeights;
};

void to_json(json& j, const Action& action);
}  // namespace proseco_planning
