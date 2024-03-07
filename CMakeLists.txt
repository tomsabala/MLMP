cmake_minimum_required(VERSION 3.0)
project(mlmp)

# Find the OMPL package
find_package(ompl REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include the OMPL directories
include_directories(/home/toms/workspace/libs/ompl-1.6.0/src)
include_directories(/home/toms/workspace/libs/ompl-1.6.0/build/Release/src)

# Add your executable, replace 'simple_ompl_program.cpp' with your actual file name
add_executable(mlmp main.cpp)

# Link your executable against the OMPL libraries
target_link_libraries(mlmp /home/toms/workspace/libs/ompl-1.6.0/build/Release/lib/libompl.so)
target_link_libraries(mlmp Eigen3::Eigen)
