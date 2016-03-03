cmake_minimum_required(VERSION 2.8.3)
project(robotics_simulator)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation tf std_msgs nav_msgs geometry_msgs vrep_common)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS roscpp tf std_msgs nav_msgs geometry_msgs vrep_common message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(joy_control src/joy_control.cpp)
target_link_libraries(joy_control ${catkin_LIBRARIES})
add_dependencies(joy_control vrep_common_generate_messages_cpp)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
