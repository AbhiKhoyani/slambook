include_directories(coordinates
    "/usr/include/eigen3"
)

add_executable(coordinates
    src/coordinates.cpp
)

# find_package(Pangolin REQUIRED)
find_package(Pangolin REQUIRED HINTS 3rdparty/Pangolin/build)
message(WARNING "Pangolin Include dirs: ${Pangolin_INCLUDE_DIRS}")
message(WARNING "Pangolin Library dirs: ${Pangolin_LIBRARIES}")
include_directories(plotTrajectory ${Pangolin_INCLUDE_DIRS})
add_executable(plotTrajectory src/plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})