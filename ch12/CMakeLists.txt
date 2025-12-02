cmake_minimum_required(VERSION 2.8)
project(chapter12)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VTK REQUIRED)                  # even though we're not using this lib, we need it for PCL-visualization
find_package(PCL COMPONENTS common io filters search kdtree surface visualization REQUIRED)
find_package(OCTOMAP REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include" ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

add_executable(dense_mono src/dense_mono.cpp src/utils.cpp)
target_link_libraries(dense_mono Eigen3::Eigen Sophus::Sophus ${OpenCV_LIBRARIES})

add_executable(pcd_mapping src/pcd_mapping.cpp)
target_link_libraries(pcd_mapping Eigen3::Eigen ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(surfel_mapping src/surfel_mapping.cpp)
target_link_libraries(surfel_mapping ${PCL_LIBRARIES})

add_executable(octomap_mapping src/octomap_mapping.cpp)
target_link_libraries(octomap_mapping Eigen3::Eigen ${OCTOMAP_LIBRARIES} ${OpenCV_LIBRARIES})