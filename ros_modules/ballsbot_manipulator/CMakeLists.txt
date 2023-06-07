cmake_minimum_required(VERSION 3.0.2)
project(ballsbot_manipulator)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  sensor_msgs
  std_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

add_executable(publisher_manipulator src/publisher_manipulator.cpp)
target_link_libraries(publisher_manipulator ${catkin_LIBRARIES})
add_dependencies(publisher_manipulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
