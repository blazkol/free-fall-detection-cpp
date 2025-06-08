cmake_minimum_required(VERSION 3.0.0)
project(simulator)

find_package(Boost 1.74)
include_directories(${Boost_INCLUDE_DIR})

add_executable(simulator_process src/main.cpp)

target_link_libraries(simulator_process ${Boost_LIBRARIES} rt)