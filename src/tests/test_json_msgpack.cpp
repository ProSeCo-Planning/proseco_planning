/**
 * @file json_msgpack.cpp
 * @brief This file defines the test cases for the the export of json and msgpack.
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_suite.hpp>

#include <string>

#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/defaultConfiguration.h"
#include "proseco_planning/util/utilities.h"

using namespace proseco_planning;

struct DataFixture {
  DataFixture() { Config::create(config::scenarioSimple, config::optionsSimple); }

  std::string file_name = "test_json_msgpack";
  ~DataFixture() { Config::get()->reset(); }
};

BOOST_FIXTURE_TEST_SUITE(json_msgpack, DataFixture)

BOOST_AUTO_TEST_CASE(json_to_file) {
  auto jScenario = sOpt().toJSON();

  util::saveJSON(file_name, jScenario);
  util::saveMsgPack(file_name, jScenario);

  auto loaded_json    = util::loadJSON(file_name + ".json");
  auto loaded_msgpack = util::loadMsgPackToJSON(file_name + ".msgpack");

  BOOST_REQUIRE(loaded_json == jScenario);
  BOOST_REQUIRE(loaded_msgpack == jScenario);
}

BOOST_AUTO_TEST_SUITE_END()