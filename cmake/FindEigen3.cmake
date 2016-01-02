find_path(EIGEN3_INCLUDE_DIR Eigen/Dense
    /usr/local/include/eigen3
    /usr/local/include
    /usr/include/eigen3
    /usr/include
    /opt/include)

SET(EIGEN3_FOUND "NO")
IF(EIGEN3_INCLUDE_DIR)
    SET(EIGEN3_FOUND "YES")
ENDIF()
