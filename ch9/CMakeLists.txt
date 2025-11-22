cmake_minimum_required(VERSION 2.8)
project(chapter9)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)    # ${CERES_LIBRARIES} = Ceres::ceres
find_package(Sophus REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../3rdparty/g2o/cmake_modules)
find_package(G2O REQUIRED)      # this doesn't work out-of-the-box so need to add above line as FindG2O.cmake is there

include_directories("${PROJECT_SOURCE_DIR}/include" ${EIGEN3_INCLUDE_DIR} ${G2O_INCLUDE_DIR})
add_library(bal_common src/common.cpp)
target_link_libraries(bal_common Eigen3::Eigen)

add_executable(bundle_adjustment_ceres src/ba_ceres.cpp)
target_link_libraries(bundle_adjustment_ceres ${CERES_LIBRARIES} bal_common Eigen3::Eigen)

add_executable(bundle_adjustment_g2o src/ba_g2o.cpp)
target_link_libraries(bundle_adjustment_g2o Eigen3::Eigen Sophus::Sophus fmt bal_common ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_LIB_TYPE} ${G2O_SOLVER_CSPARSE})