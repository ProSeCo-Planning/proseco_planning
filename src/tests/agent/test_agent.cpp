/**
 * @file test_agent.cpp
 * @brief This file defines the test cases for the agent.
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>
#include <map>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "proseco_planning/action/action.h"
#include "proseco_planning/action/actionClass.h"
#include "proseco_planning/agent/agent.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/config/scenarioOptions.h"

using namespace proseco_planning;

struct AgentFixture {
  std::vector<Agent> agents;
  AgentFixture() {
    Config::create(config::scenarioSimple, config::optionsSimple);
    config::Agent defaultAgent{0,
                               false,
                               0.5,
                               config::Desire{25.0, 0, 0, 0},
                               config::vehicle,
                               config::terminalCondition,
                               config::actionSpace,
                               config::costModel};
    agents.emplace_back(defaultAgent);
    auto doNothing0 = std::make_shared<Action>(0.0, 0.0);
    auto doNothing1 = std::make_shared<Action>(0.0, 0.0);
    auto doNothing2 = std::make_shared<Action>(0.0, 0.0);
    auto doNothing3 = std::make_shared<Action>(0.0, 0.0);
    agents[0].m_actionValues.insert(std::make_pair(doNothing0, 10.4));
    agents[0].m_actionValues.insert(std::make_pair(doNothing1, 10.1));
    agents[0].m_actionValues.insert(std::make_pair(doNothing2, 11.5));
    agents[0].m_actionValues.insert(std::make_pair(doNothing3, 11.3));
    agents[0].m_actionVisits.insert(std::make_pair(doNothing0, 23));
    agents[0].m_actionVisits.insert(std::make_pair(doNothing1, 0));
    agents[0].m_actionVisits.insert(std::make_pair(doNothing2, 98));
    agents[0].m_actionVisits.insert(std::make_pair(doNothing3, 4));
  }

  ~AgentFixture() { Config::get()->reset(); }
};

BOOST_FIXTURE_TEST_SUITE(agent, AgentFixture)

BOOST_AUTO_TEST_CASE(minActionValue) {
  auto min = agents[0].minActionValue();
  BOOST_CHECK_CLOSE(min, 10.1, 0.0001);
}
BOOST_AUTO_TEST_CASE(maxActionValue) {
  auto max = agents[0].maxActionValue();
  BOOST_CHECK_CLOSE(max, 11.5, 0.0001);
}
BOOST_AUTO_TEST_CASE(cumulativeActionVisits) {
  auto visits = agents[0].cumulativeActionVisits();
  BOOST_CHECK_CLOSE(visits, 125, 0.0001);
}
BOOST_AUTO_TEST_SUITE_END()
