################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(odometry)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    odometry_msgs
    )

find_package(Eigen3 REQUIRED)
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(yaml-cpp REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES odometry
    )

################################################################################
# Build
################################################################################
include_directories(
    include
    catkin_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    )

add_library(${PROJECT_NAME}
    src/odometry.cpp
    )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    )

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
