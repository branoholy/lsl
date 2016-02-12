# lsl-registration
set(SUBLIB_NAME ${PROJECT_NAME}-registration)

if(${install-headers} STREQUAL "OFF")
	# Find libraries
	include_directories(${LSL_CORE_INCLUDE_DIRS})

	find_package(yaml-cpp)
	include_directories(${YAML_CPP_INCLUDE_DIR})

	# Find source code
	file(GLOB_RECURSE HPPS "include/*.hpp")
	file(GLOB_RECURSE CPPS "src/*.cpp")

	# Create library
	add_library(${SUBLIB_NAME} SHARED ${CPPS} ${HPPS})
	set_target_properties(${SUBLIB_NAME} PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

	# Install library
	install(TARGETS ${SUBLIB_NAME} LIBRARY DESTINATION ${INSTALL_LIB_DIR})

	# Link libraries
	target_link_libraries(${SUBLIB_NAME} ${LSL_CORE_LIBRARIES})
	target_link_libraries(${SUBLIB_NAME} ${YAML_CPP_LIBRARIES})
endif()

if(${install-library} STREQUAL "OFF")
	install(DIRECTORY "include/${PROJECT_NAME}/" DESTINATION "include/${PROJECT_NAME}")
endif()

