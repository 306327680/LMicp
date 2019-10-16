cmake_minimum_required(VERSION 3.14)
project(LMICP)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_STANDARD 14)

add_executable(LMICP main.cpp LMicp.cpp LMicp.h)
target_link_libraries (LMICP ${PCL_LIBRARIES})