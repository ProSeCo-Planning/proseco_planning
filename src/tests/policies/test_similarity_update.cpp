/**
 * @file test_similarity_update.cpp
 * @brief This file defines the test cases for the similarity update.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <cmath>
#include <map>
#include <memory>
#include <vector>

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/monteCarloTreeSearch.h"
#include "proseco_planning/node.h"
#include "proseco_planning/util/alias.h"

using namespace proseco_planning;

struct SimilarityUpdateFixture {
  std::vector<Agent> agents_0;
  std::vector<Agent> agents_1;
  SimilarityUpdateFixture() {
    Config::create(config::scenarioSimple, config::optionsSimple);
    config::Agent defaultAgent{0,
                               false,
                               0.5,
                               config::Desire{25.0, 0, 0, 0},
                               config::vehicle,
                               config::terminalCondition,
                               config::actionSpace,
                               config::costModel};
    agents_0.emplace_back(defaultAgent);
    agents_1.emplace_back(defaultAgent);
  }

  ~SimilarityUpdateFixture() { Config::get()->reset(); }
};

BOOST_FIXTURE_TEST_SUITE(similarityUpdateTest, SimilarityUpdateFixture)

BOOST_AUTO_TEST_CASE(equalActions) {
  auto action_0 = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  auto action_1 = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));

  agents_0[0].m_actionVisits.clear();
  agents_0[0].m_actionVisits[action_0] = 2.0;

  agents_0[0].m_actionValues.clear();
  agents_0[0].m_actionValues[action_0] = 10.0;

  agents_1[0].m_actionVisits.clear();
  agents_1[0].m_actionVisits[action_1] = 3.0;

  agents_1[0].m_actionValues.clear();
  agents_1[0].m_actionValues[action_1] = 5.0;

  auto node_0 = std::make_unique<Node>(agents_0);
  auto node_1 = std::make_unique<Node>(agents_1);
  similarityUpdate(node_0.get(), node_1.get());

  BOOST_REQUIRE(node_0->m_agents[0].m_actionValues.at(action_0) == 7.0);
  BOOST_REQUIRE(node_0->m_agents[0].m_actionVisits.at(action_0) == 5.0);
}

BOOST_AUTO_TEST_CASE(differentActions) {
  auto action_0 = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 0, 0));
  auto action_1 = std::make_shared<Action>(Action(ActionClass::DO_NOTHING, 10, -10));

  agents_0[0].m_actionVisits.clear();
  agents_0[0].m_actionVisits[action_0] = 2.0;

  agents_0[0].m_actionValues.clear();
  agents_0[0].m_actionValues[action_0] = 10.0;

  agents_1[0].m_actionVisits.clear();
  agents_1[0].m_actionVisits[action_1] = 90.0;

  agents_1[0].m_actionValues.clear();
  agents_1[0].m_actionValues[action_1] = 1000.0;

  auto node_0 = std::make_unique<Node>(agents_0);
  auto node_1 = std::make_unique<Node>(agents_1);
  similarityUpdate(node_0.get(), node_1.get());

  BOOST_REQUIRE(node_0->m_agents[0].m_actionVisits.at(action_0) == 2.0);
  BOOST_REQUIRE(node_0->m_agents[0].m_actionValues.at(action_0) == 10.0);
}

BOOST_AUTO_TEST_CASE(sameActionValue) {
  auto action_0 = std::make_shared<Action>(Action(1, 0));
  auto action_1 = std::make_shared<Action>(Action(0, 0));

  agents_0[0].m_actionVisits.clear();
  agents_0[0].m_actionVisits[action_0] = 4.0;

  agents_0[0].m_actionValues.clear();
  agents_0[0].m_actionValues[action_0] = 10.0;

  agents_1[0].m_actionVisits.clear();
  agents_1[0].m_actionVisits[action_1] = 2.0;

  agents_1[0].m_actionValues.clear();
  agents_1[0].m_actionValues[action_1] = 10.0;

  auto node_0 = std::make_unique<Node>(agents_0);
  auto node_1 = std::make_unique<Node>(agents_1);
  similarityUpdate(node_0.get(), node_1.get());

  BOOST_REQUIRE(std::fabs(node_0->m_agents[0].m_actionValues.at(action_0) - 10.0) <= 0.0001);
}

BOOST_AUTO_TEST_SUITE_END()