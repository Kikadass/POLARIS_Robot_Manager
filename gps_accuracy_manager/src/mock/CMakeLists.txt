file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

set(EXECUTABLE_NAME gps_accuracy_publisher_node)

## Declare a C++ executable
add_executable(${EXECUTABLE_NAME} ${SOURCES})

## Add cmake target dependencies of the executable
add_dependencies(${EXECUTABLE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${EXECUTABLE_NAME} ${catkin_LIBRARIES})
