cmake_minimum_required(VERSION 3.0.2)
project(ballsbot_camera)

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
  Image.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_library(OPENCV_CORE opencv_core REQUIRED HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")
find_library(OPENCV_VIDEOIO opencv_videoio REQUIRED HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")

catkin_package(
  INCLUDE_DIRS "/usr/include/opencv4"
  LIBRARIES opencv_core opencv_videoio
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  "/usr/include/opencv4"
)

add_executable(camera_publisher src/publisher.cpp src/cam_capture.cpp)
target_link_libraries(camera_publisher ${catkin_LIBRARIES} ${OPENCV_CORE} ${OPENCV_VIDEOIO})
add_dependencies(camera_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
