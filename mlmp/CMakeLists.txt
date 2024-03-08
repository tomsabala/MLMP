cmake_minimum_required(VERSION 3.10)
project(mlmp)

find_package(
    ompl REQUIRED
)

include_directories(
    ${EIGEN_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
)

add_executable(mlmp src/main.cpp)

target_link_libraries(mlmp ${OMPL_LIBRARIES})

