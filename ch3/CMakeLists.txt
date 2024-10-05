cmake_minimum_required(VERSION 2.8)
project(chapter3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(eigen)
# add_subdirectory(geometry)
add_subdirectory(visualizeGeometry)
add_subdirectory(examples)
