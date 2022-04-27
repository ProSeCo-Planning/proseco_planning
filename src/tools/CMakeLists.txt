###########
## Build ##
###########

add_executable(${PROJECT_NAME}_tool_trajectory_analysis
        trajectoryAnalysis.cpp
        )

add_dependencies(${PROJECT_NAME}_tool_trajectory_analysis
        ${PROJECT_NAME}
        )

target_link_libraries(${PROJECT_NAME}_tool_trajectory_analysis
        ${PROJECT_NAME}
        pthread
        )

####

add_executable(${PROJECT_NAME}_tool_action_analysis
        actionAnalysis.cpp
        )

add_dependencies(${PROJECT_NAME}_tool_action_analysis
        ${PROJECT_NAME}
        )

target_link_libraries(${PROJECT_NAME}_tool_action_analysis
        ${PROJECT_NAME}
        pthread
        )

####

add_executable(${PROJECT_NAME}_tool_state_analysis
        stateAnalysis.cpp
        )

add_dependencies(${PROJECT_NAME}_tool_state_analysis
        ${PROJECT_NAME}
        )

target_link_libraries(${PROJECT_NAME}_tool_state_analysis
        ${PROJECT_NAME}
        pthread
        )