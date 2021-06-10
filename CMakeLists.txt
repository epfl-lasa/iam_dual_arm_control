cmake_minimum_required(VERSION 2.8.3)
project(dual_arm_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf_conversions
  roslib
  sg_differentiation
#   utils
)

# find_package(YARP)
find_package(Eigen3 REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})


catkin_package(
   INCLUDE_DIRS include
   #LIBRARIES yarp2ros_data_publisher
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf tf_conversions roslib sg_differentiation
#   DEPENDS system_lib
)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${YARP_INCLUDE_DIRS})


set(SRC_FOLDER      "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(HEADERS_FOLDER  "${CMAKE_CURRENT_SOURCE_DIR}/include/")
set(SOLVER_BWC      "${CMAKE_CURRENT_SOURCE_DIR}/bwc_cvxgen")

include_directories(${SOLVER_BWC}/)


set(HEADERS_DS  ${HEADERS_FOLDER}/Utils.hpp               # TO DO put this into a thirdarty folder
                ${HEADERS_FOLDER}/dual_arm_control.h 
                ${HEADERS_FOLDER}/dualArmFreeMotionController.h
                ${HEADERS_FOLDER}/dualArmCooperativeController.h)

set(SOURCES_DS  ${SRC_FOLDER}/dual_arm_control.cpp 
                ${SRC_FOLDER}/dualArmFreeMotionController.cpp
                ${SRC_FOLDER}/dualArmCooperativeController.cpp)


set(SOURCES_2  ${SRC_FOLDER}/dual_arm_box_control.cpp 
                ${SRC_FOLDER}/dualArmFreeMotionController.cpp
                ${SRC_FOLDER}/dualArmCooperativeController.cpp)

file(GLOB BWC_SOURCES   "${SOLVER_BWC}/*.c")

SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3")

add_library(dual_arm_control ${HEADERS_DS} ${SOURCES_DS} ${BWC_SOURCES} )
target_link_libraries(dual_arm_control ${catkin_LIBRARIES})

add_executable(dual_arm_control_node ${SRC_FOLDER}/dual_arm_control_node.cpp ${HEADERS_DS})              
target_link_libraries(dual_arm_control_node dual_arm_control ${catkin_LIBRARIES})
# target_link_libraries(dual_arm_control_node ${catkin_LIBRARIES})


#############################################################################################
add_library(dual_arm_box_control ${HEADERS_DS} ${SOURCES_2} ${BWC_SOURCES} )
target_link_libraries(dual_arm_box_control ${catkin_LIBRARIES})

add_executable(dual_arm_box_control_node ${SRC_FOLDER}/dual_arm_box_control_node.cpp ${HEADERS_DS})              
target_link_libraries(dual_arm_box_control_node dual_arm_box_control ${catkin_LIBRARIES})



### ADD Nodes that use the library
add_dependencies(dual_arm_control_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
${PROJECT_NAME}_gencfg)

