/**
 * @file test_nonlinear.cpp
 * @brief This file defines the test cases for the nonlinear cost model.
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include <eigen3/Eigen/Core>
#include <string>
#include <vector>

#include "proseco_planning/agent/cost_model/costNonLinear.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/util/utilities.h"

using namespace proseco_planning;

struct NonLinearModelFixture {
  NonLinearModelFixture() {
    inputVector(0) = 1.0;
    inputVector(1) = -2.0;
    inputVector(2) = 3.0;
    inputVector(3) = -0.5;
    inputVector(4) = 1.5;
    inputVector(5) = 2.5;
    inputVector(6) = -27.0;
    inputVector(7) = 12.0;
    inputVector(8) = 0.0;
    inputVector(9) = 439.0;

    reluOutputVector(0) = 1.0;
    reluOutputVector(1) = 0.0;
    reluOutputVector(2) = 3.0;
    reluOutputVector(3) = 0.0;
    reluOutputVector(4) = 1.5;
    reluOutputVector(5) = 2.5;
    reluOutputVector(6) = 0.0;
    reluOutputVector(7) = 12.0;
    reluOutputVector(8) = 0.0;
    reluOutputVector(9) = 439.0;

    leakyReluOutputVector(0) = 1.0;
    leakyReluOutputVector(1) = -0.02;
    leakyReluOutputVector(2) = 3.0;
    leakyReluOutputVector(3) = -0.005;
    leakyReluOutputVector(4) = 1.5;
    leakyReluOutputVector(5) = 2.5;
    leakyReluOutputVector(6) = -0.27;
    leakyReluOutputVector(7) = 12.0;
    leakyReluOutputVector(8) = 0.0;
    leakyReluOutputVector(9) = 439.0;

    hiddenLayerOutputVector(0) = 429.5;
    hiddenLayerOutputVector(1) = 429.5;
    hiddenLayerOutputVector(2) = 429.5;
    hiddenLayerOutputVector(3) = 429.5;
    hiddenLayerOutputVector(4) = 429.5;

    Config::create(config::scenarioSimple, config::optionsSimple);

    std::vector<float> w1Vector(50, 1.0);
    const auto w1 = config::CostModel::convertVectorToEigenMatrix(w1Vector, 10, 5);
    std::vector<float> w2Vector(5, 1.0);
    const auto w2 = config::CostModel::convertVectorToEigenMatrix(w2Vector, 5, 1);
    // Set cost Parameter and generate cost model
    const auto configCostModel = config::CostModel("costNonLinear", -50, 20, 100, 30, -1, 0, -500,
                                                   -500, -500, -10, 0, 1, 1, 1, 1, 1, 1, 1, w1, w2);
    costModel =
        std::dynamic_pointer_cast<CostNonLinear>(CostModel::createCostModel(configCostModel));
  }

  ~NonLinearModelFixture() { Config::get()->reset(); }

  Eigen::VectorXd inputVector{Eigen::VectorXd(10)};
  Eigen::VectorXd reluOutputVector{Eigen::VectorXd(10)};
  Eigen::VectorXd leakyReluOutputVector{Eigen::VectorXd(10)};
  Eigen::VectorXd hiddenLayerOutputVector{Eigen::VectorXd(5)};
  std::shared_ptr<CostNonLinear> costModel;
};

BOOST_FIXTURE_TEST_SUITE(non_linear, NonLinearModelFixture)

BOOST_AUTO_TEST_CASE(ReLU) {
  BOOST_REQUIRE(reluOutputVector.isApprox(costModel->ReLU(inputVector)));
}

BOOST_AUTO_TEST_CASE(LeakyReLU) {
  BOOST_REQUIRE(leakyReluOutputVector.isApprox(costModel->LeakyReLU(inputVector)));
}

BOOST_AUTO_TEST_CASE(forward_pass) {
  BOOST_REQUIRE_SMALL(costModel->forwardPass(inputVector) - 2147.5f, 0.00001f);
}

BOOST_AUTO_TEST_SUITE_END()