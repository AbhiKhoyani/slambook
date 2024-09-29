cmake_minimum_required(VERSION 2.8)
project(Geometry)

include_directories(${PROJECT_NAME}
    "/usr/include/eigen3"
)

add_executable(${PROJECT_NAME}
    src/useGeometry.cpp
)