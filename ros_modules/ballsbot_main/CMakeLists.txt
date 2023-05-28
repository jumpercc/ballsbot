cmake_minimum_required(VERSION 3.0.2)
project(ballsbot_main)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
)

catkin_package()

install(DIRECTORY launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

