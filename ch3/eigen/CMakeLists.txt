cmake_minimum_required(VERSION 2.8)
project(Eigen)

include_directories(eigenMatrix
    "/usr/include/eigen3"
)

add_executable(eigenMatrix
    src/eigenMatrix.cpp
)