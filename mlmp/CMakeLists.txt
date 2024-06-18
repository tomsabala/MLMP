cmake_minimum_required(VERSION 3.10)
project(mlmp)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/src)

find_package(CGAL REQUIRED COMPONENTS Core)
# find_package(nlohmann_json REQUIRED)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

include(FetchContent)

FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.2
)

FetchContent_MakeAvailable(nlohmann_json)

include_directories(
    ${EIGEN_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/tools
    ${CMAKE_SOURCE_DIR}/src/planner
    ${CMAKE_SOURCE_DIR}/src/planner/src
    ${CMAKE_SOURCE_DIR}/src/models
)

add_executable(
    mlmp 
    src/main.cpp
    src/planner/src/BiQRRTImpl.cpp
)

target_link_libraries(mlmp ${OMPL_LIBRARIES} CGAL::CGAL nlohmann_json::nlohmann_json Boost::program_options)

