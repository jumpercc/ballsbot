cmake_minimum_required(VERSION 3.0.2)
project(ballsbot_pose)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  nav_msgs
)

add_message_files(
  FILES
  Pose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  tf
  nav_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

add_executable(publisher_pose src/publisher_pose.cpp)
target_link_libraries(publisher_pose ${catkin_LIBRARIES})
add_dependencies(publisher_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
