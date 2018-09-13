cmake_minimum_required(VERSION 2.8.7)
project(tunnel_opt_2d)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} /usr/share/SFML/cmake/Modules)

# Also add some useful compiler flag
set(CMAKE_CXX_FLAGS
   "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++11"
) 

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen REQUIRED)
find_package(SFML 2 REQUIRED graphics window system)
include(FindG2O.cmake)
include(FindSuiteParse.cmake)

message("-- CHOLMOD_INCLUDE_DIR : " ${CHOLMOD_INCLUDE_DIR})
message("-- CSPARSE_INCLUDE_DIR : " ${CSPARSE_INCLUDE_DIR})

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${G2O_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/thirdparty/g2o
    ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
    ${SFML_INCLUDE_DIR}
    ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
    ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
)

add_executable (tunnel_opt_2d tunnel_opt_2d.cpp types.cpp)
target_link_libraries(tunnel_opt_2d ${G2O_LIBRARIES} ${CSPARSE_LIBRARY} ${CHOLMOD_LIBRARIES} ${SFML_LIBRARIES} ${SFML_DEPENDENCIES})