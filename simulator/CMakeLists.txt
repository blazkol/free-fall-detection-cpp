cmake_minimum_required(VERSION 3.0.0)
project(simulator)

set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.74 REQUIRED COMPONENTS log log_setup program_options)
include_directories(${Boost_INCLUDE_DIR})

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} rt drivers)