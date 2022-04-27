#include "proseco_planning/scenarioEvaluation.h"

#include <stdexcept>
#include <vector>

#include "proseco_planning/agent/agent.h"
#include "proseco_planning/agent/vehicle.h"
#include "proseco_planning/config/configuration.h"
#include "proseco_planning/config/scenarioOptions.h"
#include "proseco_planning/node.h"

namespace proseco_planning {

/**
 * @brief Evaluates whether a scenario is terminal, i.e. if the terminal conditions for all agents
 * are met specified in the configuration of the scenario.
 *
 * @param rootNode The current state of the scenario.
 * @return true If all terminal conditions are met.
 * @return false If not all terminal conditions are met.
 */
bool isScenarioTerminal(const Node* const rootNode) {
  for (const auto& agent : rootNode->m_agents) {
    for (const auto& scenarioAgent : sOpt().agents) {
      if (scenarioAgent.id == agent.m_id) {
        if (!terminal_condition_reached(agent.m_vehicle.m_positionX,
                                        scenarioAgent.terminal_condition.comparator_position_x,
                                        scenarioAgent.terminal_condition.position_x) ||
            !terminal_condition_reached(agent.m_vehicle.m_positionY,
                                        scenarioAgent.terminal_condition.comparator_position_y,
                                        scenarioAgent.terminal_condition.position_y)) {
          return false;
        }
      }
    }
  }
  return true;
}

/**
 * @brief Evaluates whether a given terminal condition is reached.
 *
 * @param value The value of the property.
 * @param comparator The comparator of the condition.
 * @param condition The condition that needs to be met.
 * @return true If the condition is true.
 * @return false If the condition is false.
 */
bool terminal_condition_reached(const float value, const std::string& comparator,
                                const float condition) {
  if (comparator == "larger")
    return value >= condition;
  else if (comparator == "smaller")
    return value <= condition;
  else if (comparator == "equal")
    return std::fabs(value - condition) <= 0.1;
  else if (comparator == "none")
    return true;
  else
    throw std::invalid_argument("Wrong comparator specified for terminal condition!");
  return true;
}
}  // namespace proseco_planning
