cmake_minimum_required(VERSION 2.8)

# For compatibility with PROJECT_VERSION* variables
if((${CMAKE_VERSION} VERSION_GREATER "3.0") OR (${CMAKE_VERSION} VERSION_EQUAL "3.0"))
	cmake_policy(SET CMP0048 OLD)
endif()

# Set project name
project("lsl")

# CMake configuration
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Set project version
set(PROJECT_VERSION_MAJOR "0")
set(PROJECT_VERSION_MINOR "1")
set(PROJECT_VERSION_PATCH "")
set(PROJECT_VERSION_TWEAK "")
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

if(${PROJECT_VERSION_PATCH})
	set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")
endif()

if(${PROJECT_VERSION_TWEAK})
	set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}.${PROJECT_VERSION_TWEAK}")
endif()

# Set options
option(library "Specify a library to build")
option(install-headers "Install the headers only (no library)")
option(install-library "Install the library only (no headers)")

option(INSTALL_LIB_DIR "Installation directory for libraries")
if(${INSTALL_LIB_DIR} STREQUAL "OFF")
	set(INSTALL_LIB_DIR "lib")
endif()

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

# Add core sub-library
include_directories("libs/core/include")
add_subdirectory("libs/core")

# Add other libraries
macro(subdirlist result curdir)
	file(GLOB children RELATIVE ${curdir} ${curdir}/*)
	set(dirlist "")
	foreach(child ${children})
		if(IS_DIRECTORY ${curdir}/${child})
			list(APPEND dirlist ${child})
		endif()
	endforeach()
	set(${result} ${dirlist})
endmacro()

subdirlist(subdirs ${CMAKE_SOURCE_DIR}/libs)
list(REMOVE_ITEM subdirs core)

foreach(subdir ${subdirs})
	if((${library} STREQUAL "OFF") OR (${library} STREQUAL ${subdir}))
		message(STATUS "Adding: ${subdir}")
		include_directories("libs/${subdir}/include")
	endif()
endforeach()

foreach(subdir ${subdirs})
	if((${library} STREQUAL "OFF") OR (${library} STREQUAL ${subdir}))
		add_subdirectory("libs/${subdir}")
	endif()
endforeach()

# Add uninstall
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake)

