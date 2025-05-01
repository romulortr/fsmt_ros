cmake_minimum_required(VERSION 3.10)
set(PROJECT_VERSION "0.0.1")
project(fsmt VERSION ${PROJECT_VERSION} LANGUAGES C)

set(FSMT_SO 0)
include(GNUInstallDirs)
# find_package(LAPACK REQUIRED)

##########################
##        Build         ##
##########################
add_library(${PROJECT_NAME} SHARED
    src/tube.c
    src/cartesian_tube.c
    src/polar_tube.c
    src/sensor_tube.c
    src/sampling.c
    src/score.c
    src/utils.c
    
    src/data_structure/cartesian_point.c
    src/data_structure/lidar.c
    src/data_structure/polar_point.c
    src/data_structure/sensor_point.c

    src/navigation/navigation_with_global_wp_plan.c

)
# add_dependencies(${PROJECT_NAME} data_structure)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
)

target_link_libraries(${PROJECT_NAME} 
PUBLIC
    m
    # ${LAPACK_LIBRARIES}
    # lapacke
)
