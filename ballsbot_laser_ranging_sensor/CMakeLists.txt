cmake_minimum_required(VERSION 3.0.2)
project(ballsbot_laser_ranging_sensor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  LaserDistance.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_library(I2C i2c REQUIRED HINTS "lib")

catkin_package(
  LIBRARIES I2C
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

add_executable(publisher_laser_sensor src/publisher.cpp src/VL53L0X.cpp)
target_link_libraries(publisher_laser_sensor ${catkin_LIBRARIES} ${I2C})
add_dependencies(publisher_laser_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

