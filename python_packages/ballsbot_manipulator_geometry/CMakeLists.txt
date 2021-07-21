cmake_minimum_required(VERSION 3.19)
project(ballsbot_manipulator_geometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Catch REQUIRED)

include_directories(
        .
        /usr/include/pcl-1.10
        /usr/include/eigen3
        /usr/include/ni
        /usr/include/openni2
        /home/jumper/venv/include
        /usr/include/python3.8
)

add_executable(test
        test.cpp
        ballsbot_manipulator_geometry.h)
set_target_properties(test PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(test
        contrib_catch_main
        pcl_io
        OpenNI
        OpenNI2
        pcl_registration
        pcl_features
        pcl_filters
        pcl_sample_consensus
        pcl_search
        pcl_kdtree
        lz4
        flann
        flann_cpp
        pcl_octree
        pcl_common
        )
