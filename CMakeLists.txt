cmake_minimum_required(VERSION 3.10)

project(FEC)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common kdtree)

include_directories(
    ${PYBIND11_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

pybind11_add_module(_fec MODULE src/FEC_py.cpp src/FEC.cpp)
target_link_libraries(_fec
    PRIVATE pybind11::module
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})