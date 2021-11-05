cmake_minimum_required(VERSION 3.0.2)
project(ballsbot_ups)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  UpsState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/publisher_ups.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

