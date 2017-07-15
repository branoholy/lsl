# Set project name
project("lsl-scan-matching")

# Find libraries
find_package(Eigen3 3.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

# Find source code
file(GLOB_RECURSE HPPS "include/*.hpp")
file(GLOB_RECURSE CPPS "src/*.cpp")

# Include headers
include_directories("include")

# Create executable
add_executable(${PROJECT_NAME} ${CPPS} ${HPPS})

# Link libraries
target_link_libraries(${PROJECT_NAME} lsl-core)
target_link_libraries(${PROJECT_NAME} lsl-registration)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
