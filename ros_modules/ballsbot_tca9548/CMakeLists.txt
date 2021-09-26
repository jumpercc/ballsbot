cmake_minimum_required(VERSION 3.0.2)
project(ballsbot_tca9548)

add_compile_options(-std=c++14)

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

add_executable(tca9548_controller src/tca9548_controller.cpp src/tca9548.cpp)
target_link_libraries(tca9548_controller ${catkin_LIBRARIES} ${I2C})
add_dependencies(tca9548_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(publisher_tca9548 src/publisher_tca9548.cpp src/tca9548.cpp src/VL53L0X.cpp)
target_link_libraries(publisher_tca9548 ${catkin_LIBRARIES} ${I2C})
add_dependencies(publisher_tca9548 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
