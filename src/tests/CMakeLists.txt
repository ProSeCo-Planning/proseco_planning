###########
## Build ##
###########

add_executable(${PROJECT_NAME}_test
        test_proseco_planning.cpp
        test_node.cpp
        test_invalid.cpp
        test_json_msgpack.cpp

        action/test_actionClass.cpp
        agent/test_agent.cpp
        agent/test_desire.cpp
        agent/test_vehicle.cpp
        agent/cost_model/test_potential.cpp
        agent/cost_model/test_nonlinear.cpp
        collision_checker/test_collisionChecker.cpp
        math/test_mathlib.cpp
        policies/test_policies.cpp
        policies/final_selection/test_finalSelectionKernelRegressionLCB.cpp
        policies/final_selection/test_finalSelectionSampleExpQ.cpp
        policies/update/test_updatePolicy.cpp
        policies/test_similarity_update.cpp
        trajectory/test_trajectoryGenerator.cpp
        )

add_dependencies(${PROJECT_NAME}_test
        ${PROJECT_NAME}
        )

target_link_libraries(${PROJECT_NAME}_test
        ${PROJECT_NAME}
        pthread
        )

#############
## Test ##
#############

OPTION(PROSECO_RUN_TEST ON)
if (PROSECO_RUN_TEST)
    add_custom_command(
            TARGET ${PROJECT_NAME}_test
            POST_BUILD
            COMMAND ${PROJECT_NAME}_test --log_level=all
    )
endif ()
