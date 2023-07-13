cmake_minimum_required(VERSION 2.8.3)

project(ros_dual_arm_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf_conversions
  roslib
)

find_package(Eigen3 REQUIRED)
find_package(dual_arm_control_iam)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    geometry_msgs 
    roscpp 
    std_msgs 
    tf 
    tf_conversions 
    roslib 
)

set(SRC_FOLDER      "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(HEADERS_FOLDER  "${CMAKE_CURRENT_SOURCE_DIR}/include/")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  )

add_executable(dual_arm_control_node ${SRC_FOLDER}/dual_arm_control_sim.cpp)              
target_link_libraries(dual_arm_control_node  ${catkin_LIBRARIES} dual_arm_control_iam) 
