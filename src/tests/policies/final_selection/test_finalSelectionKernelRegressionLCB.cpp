/**
 * @file test_finalSelectionKernelRegressionLCB.cpp
 * @brief This file defines the test cases for the final selection kernel regression LCB policy.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <algorithm>
#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/node.h"
#include "proseco_planning/policies/final_selection/finalSelectionKernelRegressionLCB.h"
#include "proseco_planning/util/alias.h"

namespace utf = boost::unit_test;
using namespace proseco_planning;

struct ConfigFixture {
  ConfigFixture() { Config::create(config::scenarioSimple, config::optionsSimple); }
  ~ConfigFixture() { Config::get()->reset(); }
};

struct FinalSelectionKernelRegressionLCBFixture : ConfigFixture, FinalSelectionKernelRegressionLCB {
  FinalSelectionKernelRegressionLCBFixture() : FinalSelectionKernelRegressionLCB("") {}
};

BOOST_FIXTURE_TEST_SUITE(finalSelectionKernelRegressionLCB,
                         FinalSelectionKernelRegressionLCBFixture)

// ----------------------------------------------------------------
// test cases for kernels
// ----------------------------------------------------------------

BOOST_AUTO_TEST_CASE(euclidean_kernel_for_actions, *utf::tolerance(0.00001f)) {
  m_gammaAction = 1.f;

  ActionPtr action_0_0     = std::make_shared<Action>(0.f, 0.f);
  ActionPtr action_1_0     = std::make_shared<Action>(1.f, 0.f);
  ActionPtr action_0_1     = std::make_shared<Action>(0.f, 1.f);
  ActionPtr action_1_1     = std::make_shared<Action>(1.f, 1.f);
  ActionPtr action_m1_0    = std::make_shared<Action>(-1.f, 0.f);
  ActionPtr action_0_m1    = std::make_shared<Action>(0.f, -1.f);
  ActionPtr action_m1_m1   = std::make_shared<Action>(-1.f, -1.f);
  ActionPtr action_0p5_0p5 = std::make_shared<Action>(0.5f, 0.5f);

  // identity
  BOOST_TEST(Action::getSimilarity(action_0_0, action_0_0, m_gammaAction) == 1.f);
  BOOST_TEST(Action::getSimilarity(action_1_0, action_1_0, m_gammaAction) == 1.f);
  BOOST_TEST(Action::getSimilarity(action_0_1, action_0_1, m_gammaAction) == 1.f);
  BOOST_TEST(Action::getSimilarity(action_1_1, action_1_1, m_gammaAction) == 1.f);
  BOOST_TEST(Action::getSimilarity(action_m1_0, action_m1_0, m_gammaAction) == 1.f);
  BOOST_TEST(Action::getSimilarity(action_0_m1, action_0_m1, m_gammaAction) == 1.f);
  BOOST_TEST(Action::getSimilarity(action_m1_m1, action_m1_m1, m_gammaAction) == 1.f);
  BOOST_TEST(Action::getSimilarity(action_0p5_0p5, action_0p5_0p5, m_gammaAction) == 1.f);

  BOOST_TEST(Action::getSimilarity(action_0_0, action_1_1, m_gammaAction) == 0.1353352f);
  BOOST_TEST(Action::getSimilarity(action_1_1, action_0p5_0p5, m_gammaAction) == 0.6065306f);
  BOOST_TEST(Action::getSimilarity(action_m1_m1, action_0p5_0p5, m_gammaAction) == 0.0111089f);

  // symmetry
  BOOST_TEST(Action::getSimilarity(action_0_0, action_1_0, m_gammaAction) == 0.3678794f);
  BOOST_TEST(Action::getSimilarity(action_1_0, action_0_0, m_gammaAction) == 0.3678794f);
}

BOOST_AUTO_TEST_CASE(manhattan_kernel_for_action_classes, *utf::tolerance(0.00001f)) {
  m_gammaActionClass = 1.f;

  using ac = ActionClass;

  BOOST_TEST(useManhattanKernel(ac::DO_NOTHING, ac::DO_NOTHING) == 1.f);
  BOOST_TEST(useManhattanKernel(ac::DO_NOTHING, ac::ACCELERATE) == 0.3678794f);
  BOOST_TEST(useManhattanKernel(ac::DO_NOTHING, ac::DECELERATE) == 0.3678794f);
  BOOST_TEST(useManhattanKernel(ac::DO_NOTHING, ac::CHANGE_LEFT) == 0.3678794f);
  BOOST_TEST(useManhattanKernel(ac::DO_NOTHING, ac::CHANGE_RIGHT) == 0.3678794f);
  BOOST_TEST(useManhattanKernel(ac::DO_NOTHING, ac::CHANGE_LEFT_FAST) == 0.1353352f);
  BOOST_TEST(useManhattanKernel(ac::DO_NOTHING, ac::CHANGE_LEFT_SLOW) == 0.1353352f);
  BOOST_TEST(useManhattanKernel(ac::DO_NOTHING, ac::CHANGE_RIGHT_FAST) == 0.1353352f);
  BOOST_TEST(useManhattanKernel(ac::DO_NOTHING, ac::CHANGE_RIGHT_SLOW) == 0.1353352f);

  BOOST_TEST(useManhattanKernel(ac::CHANGE_LEFT_SLOW, ac::CHANGE_LEFT_FAST) == 0.1353352f);
  BOOST_TEST(useManhattanKernel(ac::CHANGE_LEFT_SLOW, ac::CHANGE_RIGHT_SLOW) == 0.1353352f);
  BOOST_TEST(useManhattanKernel(ac::CHANGE_LEFT_SLOW, ac::ACCELERATE) == 0.0497870f);
  BOOST_TEST(useManhattanKernel(ac::CHANGE_RIGHT_FAST, ac::DECELERATE) == 0.0497870f);

  BOOST_TEST(useManhattanKernel(ac::CHANGE_LEFT_SLOW, ac::CHANGE_RIGHT_FAST) == 0.0183156f);
  BOOST_TEST(useManhattanKernel(ac::CHANGE_RIGHT_FAST, ac::CHANGE_LEFT_SLOW) == 0.0183156f);
  BOOST_TEST(useManhattanKernel(ac::CHANGE_RIGHT_SLOW, ac::CHANGE_LEFT_FAST) == 0.0183156f);
}

// ----------------------------------------------------------------
// test cases for actions
// ----------------------------------------------------------------

BOOST_AUTO_TEST_CASE(best_action_set) {
  m_moveGrouping = false;
  m_gammaAction  = 0.2f;
  m_cpAction     = 0.5f;

  auto root = std::make_unique<Node>(sOpt().agents);
  auto node = root.get();

  ActionPtr accelerate      = std::make_shared<Action>(5.f, 2.f);
  ActionPtr changeLeftFast  = std::make_shared<Action>(5.f, 4.f);
  ActionPtr decelerate      = std::make_shared<Action>(-5.f, -2.f);
  ActionPtr changeRightSlow = std::make_shared<Action>(-5.f, -4.f);

  for (auto& agent : node->m_agents) {
    agent.addAvailableAction(accelerate);
    agent.addAvailableAction(changeLeftFast);
    agent.addAvailableAction(decelerate);
    agent.addAvailableAction(changeRightSlow);

    agent.m_actionValues.at(accelerate)      = 950.f;
    agent.m_actionValues.at(changeLeftFast)  = 900.f;
    agent.m_actionValues.at(decelerate)      = 100.f;
    agent.m_actionValues.at(changeRightSlow) = 1000.f;

    agent.m_actionVisits.at(accelerate)      = 20.f;
    agent.m_actionVisits.at(changeLeftFast)  = 20.f;
    agent.m_actionVisits.at(decelerate)      = 20.f;
    agent.m_actionVisits.at(changeRightSlow) = 5.f;
  }
  setBestActionSet(node);

  for (std::size_t idx = 0; idx < node->m_agents.size(); ++idx) {
    BOOST_TEST(m_bestActionSet.at(idx) == accelerate);
  }
}

BOOST_AUTO_TEST_CASE(best_action_set_random) {
  m_moveGrouping = false;
  m_gammaAction  = 0.2f;
  m_cpAction     = 0.5f;

  auto root = std::make_unique<Node>(sOpt().agents);
  auto node = root.get();

  ActionPtr accelerate     = std::make_shared<Action>(5.f, 2.f);
  ActionPtr changeLeftFast = std::make_shared<Action>(5.f, 4.f);
  ActionPtr decelerate     = std::make_shared<Action>(-5.f, -2.f);

  for (auto& agent : node->m_agents) {
    agent.addAvailableAction(accelerate);
    agent.addAvailableAction(changeLeftFast);
    agent.addAvailableAction(decelerate);
  }
  setBestActionSet(node);

  for (std::size_t idx = 0; idx < node->m_agents.size(); ++idx) {
    BOOST_TEST(m_bestActionSet.at(idx) != nullptr);
  }
}

BOOST_AUTO_TEST_CASE(best_action_set_all_values_equal) {
  m_moveGrouping = false;
  m_gammaAction  = 0.2f;
  m_cpAction     = 0.5f;

  auto root = std::make_unique<Node>(sOpt().agents);
  auto node = root.get();

  ActionPtr accelerate      = std::make_shared<Action>(5.f, 0.5f);
  ActionPtr changeLeftFast  = std::make_shared<Action>(5.f, 1.0f);
  ActionPtr changeRightFast = std::make_shared<Action>(5.f, -1.0f);

  for (auto& agent : node->m_agents) {
    agent.m_actionValues.insert({accelerate, 1000.f});
    agent.m_actionValues.insert({changeLeftFast, 1000.f});
    agent.m_actionValues.insert({changeRightFast, 1000.f});

    agent.m_actionVisits.insert({accelerate, 20.f});
    agent.m_actionVisits.insert({changeLeftFast, 20.f});
    agent.m_actionVisits.insert({changeRightFast, 20.f});
  }
  setBestActionSet(node);

  for (std::size_t idx = 0; idx < node->m_agents.size(); ++idx) {
    BOOST_TEST(m_bestActionSet.at(idx) == accelerate);
  }
}

BOOST_AUTO_TEST_CASE(best_action_set_with_move_grouping) {
  m_moveGrouping = true;
  m_gammaAction  = 0.2f;
  m_cpAction     = 0.5f;

  auto root = std::make_unique<Node>(sOpt().agents);
  auto node = root.get();

  ActionPtr accelerate     = std::make_shared<Action>(ActionClass::ACCELERATE, 5.f, 2.f);
  ActionPtr changeLeftFast = std::make_shared<Action>(ActionClass::CHANGE_LEFT_FAST, 5.f, 4.f);
  ActionPtr doNothing      = std::make_shared<Action>(ActionClass::DO_NOTHING, 0.f, 0.f);

  m_bestActionClassSet.clear();

  for (auto& agent : node->m_agents) {
    m_bestActionClassSet.push_back(ActionClass::DO_NOTHING);

    agent.m_actionValues.insert({accelerate, 1000.f});
    agent.m_actionValues.insert({changeLeftFast, 1000.f});
    agent.m_actionValues.insert({doNothing, 500.f});

    agent.m_actionVisits.insert({accelerate, 20.f});
    agent.m_actionVisits.insert({changeLeftFast, 20.f});
    agent.m_actionVisits.insert({doNothing, 5.f});
  }
  setBestActionSet(node);

  for (std::size_t idx = 0; idx < node->m_agents.size(); ++idx) {
    BOOST_TEST(m_bestActionSet.at(idx) == doNothing);
  }
}

// ----------------------------------------------------------------
// test cases for action classes
// ----------------------------------------------------------------

BOOST_AUTO_TEST_CASE(best_action_class_set) {
  m_gammaActionClass = 1.0f;
  m_cpActionClass    = 0.5f;

  auto root = std::make_unique<Node>(sOpt().agents);
  auto node = root.get();

  for (auto& agent : node->m_agents) {
    agent.m_actionClassValues.insert({ActionClass::ACCELERATE, 950.f});
    agent.m_actionClassValues.insert({ActionClass::CHANGE_LEFT_FAST, 800.f});
    agent.m_actionClassValues.insert({ActionClass::DECELERATE, 100.f});
    agent.m_actionClassValues.insert({ActionClass::CHANGE_RIGHT_SLOW, 1000.f});

    agent.m_actionClassVisits.insert({ActionClass::ACCELERATE, 20.f});
    agent.m_actionClassVisits.insert({ActionClass::CHANGE_LEFT_FAST, 20.f});
    agent.m_actionClassVisits.insert({ActionClass::DECELERATE, 20.f});
    agent.m_actionClassVisits.insert({ActionClass::CHANGE_RIGHT_SLOW, 5.f});
  }
  setBestActionClass(node);

  for (std::size_t idx = 0; idx < node->m_agents.size(); ++idx) {
    BOOST_TEST((m_bestActionClassSet.at(idx) == ActionClass::ACCELERATE));
  }
}

BOOST_AUTO_TEST_CASE(best_action_class_set_random) {
  auto root = std::make_unique<Node>(sOpt().agents);
  auto node = root.get();

  ActionPtr accelerate     = std::make_shared<Action>(ActionClass::ACCELERATE);
  ActionPtr changeLeftFast = std::make_shared<Action>(ActionClass::CHANGE_LEFT_FAST);
  ActionPtr decelerate     = std::make_shared<Action>(ActionClass::DECELERATE);

  for (auto& agent : node->m_agents) {
    agent.addAvailableAction(accelerate);
    agent.addAvailableAction(changeLeftFast);
    agent.addAvailableAction(decelerate);
  }
  BOOST_CHECK_NO_THROW(setBestActionClass(node));
}

BOOST_AUTO_TEST_CASE(best_action_class_set_all_values_equal) {
  m_gammaActionClass = 1.0f;
  m_cpActionClass    = 0.5f;

  auto root = std::make_unique<Node>(sOpt().agents);
  auto node = root.get();

  for (auto& agent : node->m_agents) {
    agent.m_actionClassValues.insert({ActionClass::ACCELERATE, 1000.f});
    agent.m_actionClassValues.insert({ActionClass::CHANGE_LEFT_FAST, 1000.f});
    agent.m_actionClassValues.insert({ActionClass::CHANGE_RIGHT, 1000.f});

    agent.m_actionClassVisits.insert({ActionClass::ACCELERATE, 20.f});
    agent.m_actionClassVisits.insert({ActionClass::CHANGE_LEFT_FAST, 20.f});
    agent.m_actionClassVisits.insert({ActionClass::CHANGE_RIGHT, 20.f});
  }
  setBestActionClass(node);

  for (std::size_t idx = 0; idx < node->m_agents.size(); ++idx) {
    BOOST_TEST((m_bestActionClassSet.at(idx) == ActionClass::ACCELERATE));
  }
}

BOOST_AUTO_TEST_SUITE_END()