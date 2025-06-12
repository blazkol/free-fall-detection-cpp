cmake_minimum_required(VERSION 3.0.0)
project(free_fall_detection)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_EXTENSIONS OFF)

add_subdirectory(drivers)
add_subdirectory(simulator)
add_subdirectory(detector)