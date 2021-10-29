project(ballsbot_localization)

cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
set(LIBS ${LIBS} ${EIGEN3_LIBRARIES})

add_executable(test
        src/test.cpp
        src/grid.cpp src/point_cloud.cpp src/geometry.cpp src/free_distances.cpp
        )
set_target_properties(test PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(test ${EIGEN3})
