cmake_minimum_required(VERSION 2.8.3)
project(tricycle)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation tf std_msgs nav_msgs geometry_msgs vrep_common)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS roscpp tf std_msgs nav_msgs geometry_msgs vrep_common message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(odom src/odom.cpp) 
target_link_libraries(odom ${catkin_LIBRARIES})
add_dependencies(odom vrep_common_generate_messages_cpp)

add_executable(control src/control.cpp) 
target_link_libraries(control ${catkin_LIBRARIES})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

install(DIRECTORY launch meshes navfiles rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
