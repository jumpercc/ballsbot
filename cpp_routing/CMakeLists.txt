project(ballsbot-cpp)

cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Catch REQUIRED)

add_executable(test
        test.cpp
        ballsbot/point_cloud.cpp ballsbot/geometry.cpp ballsbot/grid.cpp
        )
set_target_properties(test PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(test
        contrib_catch_main
        )
