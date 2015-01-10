cmake_minimum_required(VERSION 2.8)

set(PROJECT_NAME "lsl")
set(CMAKE_BUILD_TYPE Release)

add_definitions("-std=c++11")
add_definitions("-O3")
add_definitions("-Wall -Wextra -pedantic")

project(${PROJECT_NAME})

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

find_package(Eigen3 3.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

find_package(wxWidgets 3.0 REQUIRED core base)
include(${wxWidgets_USE_FILE})
include_directories(${wxWidgets_INCLUDE_DIRS})
add_definitions(${wxWidgets_CXX_FLAGS})

file(GLOB_RECURSE CPPS "src/lsl/*.cpp")
file(GLOB_RECURSE HPPS "include/lsl/*.hpp")

add_library(${PROJECT_NAME} SHARED ${CPPS} ${HPPS})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES})
