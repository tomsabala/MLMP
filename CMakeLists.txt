cmake_minimum_required(VERSION 3.0)
project(mlmp)

# Find the OMPL package
find_package(ompl REQUIRED)
find_package (Eigen3 3.3 NO_MODULE)

SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

# Include the OMPL directories
include_directories(/home/toms/workspace/libs/ompl-1.6.0/src)
include_directories(/home/toms/workspace/libs/ompl-1.6.0/build/Release/src)

# Add your executable, replace 'simple_ompl_program.cpp' with your actual file name
add_executable(mlmp main.cpp)

# Link your executable against the OMPL libraries
target_link_libraries(mlmp /home/toms/workspace/libs/ompl-1.6.0/build/Release/lib/libompl.so)
