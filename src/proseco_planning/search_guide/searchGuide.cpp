#include "proseco_planning/search_guide/searchGuide.h"

#include <iostream>

#include "proseco_planning/search_guide/searchGuideBlindValue.h"
#include "proseco_planning/search_guide/searchGuideRandom.h"

namespace proseco_planning {

/**
 * @brief Constructs a new Search Guide object.
 *
 * @param type The type of the of the search guide.
 */
SearchGuide::SearchGuide(const std::string& type) : m_type(type) {}

/**
 * @brief Creates a SearchGuide instance according to the specified type. Either blind value or
 * random.
 *
 * @param type The type of the search guide.
 * @return std::shared_ptr<SearchGuide> The search guide instance.
 */
std::shared_ptr<SearchGuide> SearchGuide::createSearchGuide(const std::string& type) {
  if (type == "blindValue") {
    return std::make_shared<SearchGuideBlindValue>(type);
  } else if (type == "random") {
    return std::make_shared<SearchGuideRandom>(type);
  } else {
    throw std::invalid_argument("Unknown search guide type: " + type);
  }
}
}  // namespace proseco_planning
