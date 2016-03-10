find_path(YAML_CPP_INCLUDE_DIR yaml-cpp
	/usr/local/include
	/usr/include
	/opt/include)

set(YAML_CPP_FOUND "NO")
if(YAML_CPP_INCLUDE_DIR)
	set(YAML_CPP_LIBRARIES yaml-cpp)

	message(STATUS "Found yaml-cpp: ${YAML_CPP_INCLUDE_DIR}/yaml-cpp")
    set(YAML_CPP_FOUND "YES")
endif()
