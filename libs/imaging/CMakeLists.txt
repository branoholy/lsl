# lsl-imaging
set(SUBLIB_NAME ${PROJECT_NAME}-imaging)

# Install library
if(${install-library} STREQUAL "OFF")
	install(DIRECTORY "include/${PROJECT_NAME}/" DESTINATION "include/${PROJECT_NAME}")
endif()

