cmake_minimum_required(VERSION 3.0.2)
project(ballsbot_detection)

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
  Point.msg
  Detection.msg
  DetectionsList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_library(UFFSSD uffssd REQUIRED HINTS "lib")
find_library(OPENCV_CORE opencv_core REQUIRED HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")
find_library(OPENCV_VIDEOIO opencv_videoio REQUIRED HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")

catkin_package(
  INCLUDE_DIRS "/usr/include/opencv4"
  LIBRARIES opencv_core opencv_videoio uffssd
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  "/usr/include/opencv4"
)

add_executable(publisher src/publisher.cpp src/cam_detector.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES} ${UFFSSD} ${OPENCV_CORE} ${OPENCV_VIDEOIO})
add_dependencies(publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/publisher_detection.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
