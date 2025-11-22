cmake_minimum_required(VERSION 2.8)
project(chapter10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../3rdparty/g2o/cmake_modules)
find_package(G2O REQUIRED)      # this doesn't work out-of-the-box so need to add above line as FindG2O.cmake is there

include_directories("${PROJECT_SOURCE_DIR}/include" ${EIGEN3_INCLUDE_DIR} ${G2O_INCLUDE_DIR})

add_executable(pose_graph_g2o_se3 src/pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_se3 Eigen3::Eigen fmt ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_TYPES_SLAM3D})

add_executable(pose_graph_g2o_sophus src/pose_graph_g2o_sophus.cpp)
target_link_libraries(pose_graph_g2o_sophus Eigen3::Eigen Sophus::Sophus fmt ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} )