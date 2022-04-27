cmake_minimum_required(VERSION 3.5.0)
project(proseco_planning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include lib/json/include
        LIBRARIES ${PROJECT_NAME}
        DEPENDS EIGEN3
)

###########
## Build ##
###########

#add_subdirectory(
#	src/${PROJECT_NAME}
#)
## Declare a C++ library
add_library(${PROJECT_NAME}
        src/proseco_planning/action/action.cpp
        src/proseco_planning/action/actionSpace.cpp
        src/proseco_planning/action/actionSpaceRectangle.cpp
        src/proseco_planning/action/noiseGenerator.cpp
        src/proseco_planning/agent/agent.cpp
        src/proseco_planning/agent/cost_model/costContinuous.cpp
        src/proseco_planning/agent/cost_model/costExponential.cpp
        src/proseco_planning/agent/cost_model/costLinear.cpp
        src/proseco_planning/agent/cost_model/costLinearCooperative.cpp
        src/proseco_planning/agent/cost_model/costModel.cpp
        src/proseco_planning/agent/cost_model/costNonLinear.cpp
        src/proseco_planning/agent/desire.cpp
        src/proseco_planning/agent/vehicle.cpp
        src/proseco_planning/collision_checker/collisionChecker.cpp
        src/proseco_planning/collision_checker/collisionCheckerCircleApproximation.cpp
        src/proseco_planning/config/computeOptions.cpp
        src/proseco_planning/config/configuration.cpp
        src/proseco_planning/config/defaultConfiguration.cpp
        src/proseco_planning/config/outputOptions.cpp
        src/proseco_planning/config/scenarioOptions.cpp
        src/proseco_planning/exporters/exporter.cpp
        src/proseco_planning/exporters/jsonExporter.cpp
        src/proseco_planning/exporters/msgPackExporter.cpp
        src/proseco_planning/math/mathlib.cpp
        src/proseco_planning/monteCarloTreeSearch.cpp
        src/proseco_planning/node.cpp
        src/proseco_planning/policies/expansion/expansionUCT.cpp
        src/proseco_planning/policies/expansionPolicy.cpp
        src/proseco_planning/policies/final_selection/finalSelectionKernelRegressionLCB.cpp
        src/proseco_planning/policies/final_selection/finalSelectionMaxActionValue.cpp
        src/proseco_planning/policies/final_selection/finalSelectionMaxVisitCount.cpp
        src/proseco_planning/policies/final_selection/finalSelectionMostTrusted.cpp
        src/proseco_planning/policies/final_selection/finalSelectionSampleExpQ.cpp
        src/proseco_planning/policies/finalSelectionPolicy.cpp
        src/proseco_planning/policies/policy.cpp
        src/proseco_planning/policies/selection/selectionUCTProgressiveWidening.cpp
        src/proseco_planning/policies/selectionPolicy.cpp
        src/proseco_planning/policies/simulation/simulationMultiThread.cpp
        src/proseco_planning/policies/simulation/simulationSingleThread.cpp
        src/proseco_planning/policies/simulationPolicy.cpp
        src/proseco_planning/policies/update/updateUCT.cpp
        src/proseco_planning/policies/updatePolicy.cpp
        src/proseco_planning/scenarioEvaluation.cpp
        src/proseco_planning/search_guide/searchGuide.cpp
        src/proseco_planning/search_guide/searchGuideBlindValue.cpp
        src/proseco_planning/search_guide/searchGuideRandom.cpp
        src/proseco_planning/trajectory/constantacceleration.cpp
        src/proseco_planning/trajectory/polynomialgenerator.cpp
        src/proseco_planning/trajectory/trajectory.cpp
        src/proseco_planning/trajectory/trajectorygenerator.cpp
        src/proseco_planning/util/utilities.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/json/include
        ${Eigen3_INCLUDE_DIR}
        )

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        )
target_link_libraries(${PROJECT_NAME}
        OpenMP::OpenMP_CXX
)

add_subdirectory(
        src/tools
)

add_subdirectory(
        src/tests
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        #   FILES_MATCHING PATTERN "*.h"
        #   PATTERN ".svn" EXCLUDE
        )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_proseco_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
