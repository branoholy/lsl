cmake_minimum_required(VERSION 2.8)

# Set project name
project("lsl")

# CMake configuration
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Set build type and definitions
if($ENV{DEBUG})
    # Debug
    set(CMAKE_BUILD_TYPE "Debug")
else()
    # Release
    set(CMAKE_BUILD_TYPE "Release")
    add_definitions("-O3")
endif()

add_definitions("-std=c++11")
# add_definitions("-Wall -Wextra -pedantic")
add_definitions("-Wall -Wextra -pedantic -Wno-deprecated-declarations")

# Find libraries
find_package(Eigen3 3.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

find_package(yaml-cpp)
include_directories(${YAML_CPP_INCLUDE_DIR})

# Find source code
file(GLOB_RECURSE CPPS "src/lsl/*.cpp")
file(GLOB_RECURSE HPPS "include/lsl/*.hpp")

# Include headers
include_directories("include")

# Create library
add_library(${PROJECT_NAME} SHARED ${CPPS} ${HPPS})

# Link libraries
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

# Add examples
add_subdirectory("examples/lsl-test")
