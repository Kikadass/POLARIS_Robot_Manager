cmake_minimum_required(VERSION 3.0.2)
project(robot_manager_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  genmsg
)

## Generate messages in the 'msg' folder
file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg msg/*.msg)

# Print each item in the list
message(STATUS "MSG_FILES:")
foreach(ITEM ${MSG_FILES})
    message(STATUS "- ${ITEM}")
endforeach()

add_message_files(FILES ${MSG_FILES})

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
