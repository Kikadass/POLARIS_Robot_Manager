file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

## Declare a C++ executable
set(EXECUTABLE_NAME battery_percentage_publisher_node)

add_executable(${EXECUTABLE_NAME} ${SOURCES})
add_dependencies(${EXECUTABLE_NAME} ${${EXECUTABLE_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${EXECUTABLE_NAME} ${catkin_LIBRARIES})
