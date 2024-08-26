file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

## Declare a C++ executable
set(EXECUTABLE_NAME gps_accuracy_manager_node)

add_executable(${EXECUTABLE_NAME} ${SOURCES})
add_dependencies(${EXECUTABLE_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${robot_manager_msgs_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${EXECUTABLE_NAME} ${catkin_LIBRARIES})

#############
## Testing ##
#############
add_subdirectory(mock)

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gps_accuracy_manager.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
