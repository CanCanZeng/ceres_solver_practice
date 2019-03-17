cmake_minimum_required (VERSION 2.8)
cmake_policy(VERSION 2.8)

project (ceres_solver_practice)

if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_BUILD_TYPE "Debug")
#    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -O0 -Wall -Wno-unused-variable -pthread")
else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O2 -pthread -fopenmp")
endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )
message("Build Type: " ${CMAKE_BUILD_TYPE} ${CMAKE_CXX_FLAGS})

# 设定二进制文件路径
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
# 设定头文件路径
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )
# 设定库文件编译路径
#SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# 设定 .cmake 文件存放路径
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

link_directories(${CMAKE_SOURCE_DIR}/lib)

# find the required packages
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${CERES_INCLUDE_DIRS}
    )

add_executable( main_test_ceres_hello
    test/main_test_ceres_hello.cpp
)

target_link_libraries (main_test_ceres_hello
    ${CERES_LIBRARIES}
    )

add_executable( main_test_pose_graph
    test/main_test_pose_graph.cpp
)

target_link_libraries (main_test_pose_graph
    ${CERES_LIBRARIES}
    )