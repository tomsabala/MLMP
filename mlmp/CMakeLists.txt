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
