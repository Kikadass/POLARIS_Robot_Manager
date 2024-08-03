set(NO_SIGNAL_EXECUTABLE_NAME mock_no_signal_publisher_node)
add_executable(${NO_SIGNAL_EXECUTABLE_NAME} MockNoSignalMain.cpp MockSignalStrengthPublisher.cpp)
add_dependencies(${NO_SIGNAL_EXECUTABLE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${NO_SIGNAL_EXECUTABLE_NAME} ${catkin_LIBRARIES})

set(MIXED_SIGNAL_EXECUTABLE_NAME mock_mixed_signal_publisher_node)
add_executable(${MIXED_SIGNAL_EXECUTABLE_NAME} MockMixedSignalMain.cpp MockSignalStrengthPublisher.cpp)
add_dependencies(${MIXED_SIGNAL_EXECUTABLE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MIXED_SIGNAL_EXECUTABLE_NAME} ${catkin_LIBRARIES})
