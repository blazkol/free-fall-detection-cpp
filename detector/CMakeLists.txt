cmake_minimum_required(VERSION 3.0.0)
project(detector)

find_package(Boost 1.74)
include_directories(${Boost_INCLUDE_DIR})

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} rt)
target_link_libraries(${PROJECT_NAME} drivers)