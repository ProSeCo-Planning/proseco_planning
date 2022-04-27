/**
 * @file test_potential.cpp
 * @brief This file defines the test cases for the potential calculation.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <algorithm>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <cstddef>
#include <eigen3/Eigen/Core>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "proseco_planning/agent/cost_model/costModel.h"
#include "proseco_planning/agent/desire.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/computeOptions.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"

using namespace proseco_planning;

BOOST_AUTO_TEST_SUITE(potential)

BOOST_AUTO_TEST_CASE(laneCenterDeviation) {
  Config::create(config::scenarioSimple, config::optionsSimple);

  int lane{1};

  Vehicle vehicle(config::vehicle);
  vehicle.m_positionX = 0;
  vehicle.m_velocityX = 35.0;
  vehicle.setLane(lane);
  Desire desire = Desire(config::Desire(35.0, 0, lane, 0));

  std::vector<float> v1(50, 1.0);
  Eigen::MatrixXd w1 = config::CostModel::convertVectorToEigenMatrix(v1, 10, 5);
  std::vector<float> v2(5, 1.0);
  Eigen::MatrixXd w2 = config::CostModel::convertVectorToEigenMatrix(v2, 5, 1);
  // Set cost Parameter and generate cost model
  auto configCostModel = config::CostModel("costContinuous", -50, 20, 100, 30, -1, 0, -500, -500,
                                           -500, -10, 0, 1, 1, 1, 1, 1, 1, 1, w1, w2);

  auto costModel = CostModel::createCostModel(configCostModel);

  const unsigned int steps = 11;
  std::vector<float> potentialLaneCenterOffset(steps);
  for (size_t offset = 0; offset < potentialLaneCenterOffset.size(); ++offset) {
    vehicle.m_positionY               = sOpt().road.lane_width * (lane + offset / 10.f);
    potentialLaneCenterOffset[offset] = costModel->updateStatePotential(desire, vehicle);
  }

  std::vector<float> potentialLaneCenterOffsetReverse(potentialLaneCenterOffset.size());
  std::reverse_copy(std::begin(potentialLaneCenterOffset), std::end(potentialLaneCenterOffset),
                    std::begin(potentialLaneCenterOffsetReverse));
  for (size_t i = 0; i < potentialLaneCenterOffset.size(); ++i) {
    BOOST_REQUIRE_SMALL(potentialLaneCenterOffsetReverse[i] - potentialLaneCenterOffset[i],
                        float(0.001));
  }
}

BOOST_AUTO_TEST_SUITE_END()
