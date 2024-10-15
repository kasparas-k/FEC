cmake_minimum_required(VERSION 3.10)

project(ClusteringApp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.12 REQUIRED COMPONENTS io common kdtree)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_executable(clustering main.cpp FEC.cpp)

target_link_libraries(clustering ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})