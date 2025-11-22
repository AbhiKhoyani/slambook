cmake_minimum_required(VERSION 2.8)
project(chapter11)

# Need to use c++11 because of DBOW
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)

# there's no FindDBoW3.cmake generate so set it manually
set(DBoW3_INCLUDE_DIRS "/usr/local/include")
set(DBoW3_LIBRARIES "/usr/local/lib/libDBoW3.so")

include_directories(${DBoW3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(feature_training src/feature_training.cpp)
target_link_libraries(feature_training ${DBoW3_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(loop_closure src/loop_closure.cpp)
target_link_libraries(loop_closure ${DBoW3_LIBRARIES} ${OpenCV_LIBRARIES})

# Run it as WAYLAND_DISPLAY=0 ./build/examples/plotTrajectory