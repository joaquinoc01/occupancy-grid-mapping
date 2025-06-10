cmake_minimum_required(VERSION 3.14)
project(OccupancyGridMapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -O2)

find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(occupancy_grid_mapping
    src/occupancy_grid_mapping.cpp
)
