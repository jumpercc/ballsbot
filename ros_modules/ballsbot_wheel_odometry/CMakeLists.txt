cmake_minimum_required(VERSION 3.0.2)
project(ballsbot_wheel_odometry)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  OdometryState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS scripts/publisher_wheel_odometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

