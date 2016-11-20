# lsl-imaging
set(SUBLIB_NAME ${PROJECT_NAME}-imaging)

# Find libraries
include_directories(${LSL_CORE_INCLUDE_DIRS})

# Find source code
file(GLOB_RECURSE HPPS "include/*.hpp")

# Create library
add_custom_target(${SUBLIB_NAME} SOURCES ${HPPS})

if(${install-library} STREQUAL "OFF")
	# Install library
	install(DIRECTORY "include/${PROJECT_NAME}/" DESTINATION "include/${PROJECT_NAME}")
endif()

