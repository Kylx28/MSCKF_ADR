cmake_minimum_required(VERSION 3.0.2)
project(MSCKF_ADR)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++14")
set(CMAKE_CXX_STANDARD 14)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs sensor_msgs)
find_package(Eigen3 REQUIRED)

## Declare a catkin package
catkin_package()

## These are all the files in the src folder
list(APPEND SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/msckf_node.cpp
    ${PROJECT_SOURCE_DIR}/src/msckf.cpp
    ${PROJECT_SOURCE_DIR}/src/feature_detection.cpp
    ${PROJECT_SOURCE_DIR}/src/propagate.cpp
    ${PROJECT_SOURCE_DIR}/src/update.cpp)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

## These add_library functions add the main code files and include their headers
add_library(msckf src/msckf.cpp)
add_library(feature_detection src/feature_detection.cpp)
add_library(propagate src/propagate.cpp)
add_library(update src/update.cpp)

## These are for the node file
add_executable(msckf_node src/msckf_node.cpp)

## Link the libraries
target_link_libraries(msckf_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(msckf_node msckf feature_detection propagate update ${catkin_LIBRARIES})
