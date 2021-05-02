cmake_minimum_required(VERSION 3.0.2)
project(ballsbot_magnetic_encoder)

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
  EncoderAngle.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_library(I2C i2c REQUIRED HINTS "lib")

catkin_package(
  LIBRARIES "${I2C}"
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)

add_executable(publisher_magnetic_encoder src/magnetic_encoder.cpp src/magnetic_encoder_driver.cpp)
target_link_libraries(publisher_magnetic_encoder ${catkin_LIBRARIES} ${I2C})
add_dependencies(publisher_magnetic_encoder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
