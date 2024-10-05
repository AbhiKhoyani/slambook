cmake_minimum_required(VERSION 2.8)
project(chapter4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(sophus_example ${Sophus_INCLUDE_DIRS})
include_directories(sophus_example ${EIGEN3_INCLUDE_DIR})

add_executable(sophus_example src/sophus.cpp)
target_link_libraries(sophus_example Sophus::Sophus)

include_directories(trajectoryError ${Pangolin_INCLUDE_DIRS})
add_executable(trajectoryError src/trajectoryError.cpp)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES})

# Run it as WAYLAND_DISPLAY=0 ./build/examples/plotTrajectory