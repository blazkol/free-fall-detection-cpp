cmake_minimum_required(VERSION 3.0.0)
project(drivers)

add_library(${PROJECT_NAME} STATIC 
    src/i2c_driver.c
)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        ${PROJECT_SOURCE_DIR}/include
)