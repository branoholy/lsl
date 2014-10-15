cmake_minimum_required(VERSION 2.8)

set(PROJECT_NAME "lsl")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3")

project(${PROJECT_NAME})

add_definitions("-std=c++11")
# add_definitions("-Wall -Wextra -pedantic")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

file(GLOB SRCS "src/lsl/*/*.cpp")

add_library(${PROJECT_NAME} SHARED ${SRCS})
