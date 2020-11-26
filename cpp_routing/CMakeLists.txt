project(ballsbot-cpp)

cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Catch REQUIRED)

include_directories(
        "/usr/include/opencv4"
)

find_library(OPENCV_CORE opencv_core HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")
find_library(OPENCV_VIDEOIO opencv_videoio HINTS "/usr/lib/x86_64-linux-gnu" "/usr/lib/aarch64-linux-gnu")

add_executable(test
        test.cpp
        ballsbot/point_cloud.cpp ballsbot/geometry.cpp ballsbot/grid.cpp
        ballsbot/cam_detector.h ballsbot/cam_detector.cpp ballsbot/uffssd.h
        )
set_target_properties(test PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(test
        contrib_catch_main
        ${OPENCV_CORE} ${OPENCV_VIDEOIO}
        )
