cmake_minimum_required(VERSION 3.22)
project(mlmp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ompl 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(nlohmann_json 3.11 REQUIRED)

option(MLMP_ENABLE_URDF "Build URDF/3D support (urdfdom + FCL)" ON)

add_executable(mlmp
    src/main.cpp
    src/planner/src/BiQRRTImpl.cpp
)

target_include_directories(mlmp PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(mlmp PRIVATE
    ompl::ompl
    Boost::program_options
    nlohmann_json::nlohmann_json
)

if(MLMP_ENABLE_URDF)
    # urdfdom's config exports urdfdom::urdf_parser; FCL's exports a plain `fcl` target
    # and pulls Eigen3 and ccd itself.
    find_package(urdfdom REQUIRED)
    find_package(fcl REQUIRED)
    target_compile_definitions(mlmp PRIVATE MLMP_ENABLE_URDF)
    target_link_libraries(mlmp PRIVATE urdfdom::urdf_parser fcl)
endif()
