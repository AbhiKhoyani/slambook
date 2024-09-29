cmake_minimum_required(VERSION 2.8)
project(HelloSLAM)

add_library(lib_helloSLAM
    SHARED src/helloSLAM.cpp
)

add_executable(${PROJECT_NAME}
    src/helloMain.cpp
)

include_directories(${PROJECT_NAME}
    include/
)

target_link_libraries(${PROJECT_NAME}
    lib_helloSLAM
)

set (CMAKE_INSTALL_PREFIX ../deploy)
install(TARGETS ${PROJECT_NAME} DESTINATION bin/)
# install(FILES lib_helloSLAM DESTINATION deploy/lib/)
install(DIRECTORY include DESTINATION include/)