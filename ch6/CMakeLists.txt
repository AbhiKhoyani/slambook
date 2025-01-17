cmake_minimum_required(VERSION 2.8)
project(chapter6)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)
find_package(Ceres REQUIRED)    # ${CERES_LIBRARIES} = Ceres::ceres
include_directories("/usr/include/eigen3")
message(warning ${G2O_INCLUDE_DIRS}) 
message(warning ${G2O_CORE_LIBRARY}) 
message(warning ${G2O_STUFF_LIBRARY}) 

include_directories(${OpenCV_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS})
add_executable(gaussNewton src/gaussNewton.cpp)
target_link_libraries(gaussNewton ${OpenCV_LIBRARIES} Eigen3::Eigen)

add_executable(ceresCurveFitting src/ceresCurveFitting.cpp)
target_link_libraries(ceresCurveFitting ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(g2oCurveFitting src/g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBRARIES} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
