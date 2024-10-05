cmake_minimum_required(VERSION 2.8)
project(chapter5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

include_directories(joinMap ${Pangolin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(joinMap src/joinMap.cpp)
target_link_libraries(joinMap ${Pangolin_LIBRARIES} Sophus::Sophus ${OpenCV_LIBRARIES})


# Run it as WAYLAND_DISPLAY=0 ./build/examples/plotTrajectory