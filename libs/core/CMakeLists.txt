# lsl-core
set(SUBLIB_NAME ${PROJECT_NAME}-core)

# Find libraries
find_package(Eigen3 3.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

if((${library} STREQUAL "OFF") OR (${library} STREQUAL "core"))
	if(${install-headers} STREQUAL "OFF")
		# Find source code
		file(GLOB_RECURSE HPPS "include/*.hpp")
		file(GLOB_RECURSE CPPS "src/*.cpp")

		# Create library
		add_library(${SUBLIB_NAME} SHARED ${CPPS} ${HPPS})
		set_target_properties(${SUBLIB_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

		# Install library
		install(TARGETS ${SUBLIB_NAME} LIBRARY DESTINATION ${INSTALL_LIB_DIR})
	endif()

	if(${install-library} STREQUAL "OFF")
		install(DIRECTORY "include/${PROJECT_NAME}/" DESTINATION "include/${PROJECT_NAME}")
	endif()
endif()

set(LSL_CORE_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} PARENT_SCOPE)
set(LSL_CORE_LIBRARIES ${SUBLIB_NAME} PARENT_SCOPE)

