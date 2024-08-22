cmake_minimum_required(VERSION 3.0.2)
project(ballsbot_pose_ndt)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  rospy
  std_msgs
  message_generation
  sensor_msgs
)

add_message_files(
  FILES
  FixedPose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

find_library(PCL_IO pcl_io REQUIRED HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")
find_library(PCL_REGISTRATION pcl_registration REQUIRED HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")
find_library(PCL_FILTERS pcl_filters REQUIRED HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")
find_library(PCL_COMMON pcl_common REQUIRED HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")
find_library(PCL_SEARCH pcl_search REQUIRED HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")
find_library(PCL_KDTREE pcl_kdtree REQUIRED HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
  "/usr/local/include/pcl-1.10"
  "/usr/include/pcl-1.8"
  "/usr/include/eigen3"
)

add_executable(publisher_pose_ndt src/publisher_pose_ndt.cpp)
target_link_libraries(publisher_pose_ndt ${catkin_LIBRARIES} ${PCL_IO} ${PCL_REGISTRATION} ${PCL_FILTERS} ${PCL_COMMON} ${PCL_SEARCH} ${PCL_KDTREE})
add_dependencies(publisher_pose_ndt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
