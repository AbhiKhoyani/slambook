cmake_minimum_required(VERSION 2.8)
project(chapter7)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED)    # ${CERES_LIBRARIES} = Ceres::ceres

# list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../3rdparty/g2o/cmake_modules)
# find_package(G2O REQUIRED)      # this doesn't work out-of-the-box so need to add above line as FindG2O.cmake is there

# include_directories("/usr/include/eigen3")

include_directories(${OpenCV_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/include")
add_executable(orb_cv src/orb_cv.cpp)
target_link_libraries(orb_cv ${OpenCV_LIBRARIES})

add_executable(pose_estimation_2d2d src/pose_estimation_2d2d.cpp src/get_orb.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBRARIES})

# add_executable(g2oCurveFitting src/g2oCurveFitting.cpp)
# target_link_libraries(g2oCurveFitting ${OpenCV_LIBRARIES} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
