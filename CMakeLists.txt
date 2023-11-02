cmake_minimum_required(VERSION 2.8.3)

project(dual_arm_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

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
)

# find_package(YARP)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    geometry_msgs 
    roscpp 
    std_msgs 
    tf 
    tf_conversions 
    roslib 
    sg_differentiation
)

set(SRC_FOLDER      "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(HEADERS_FOLDER  "${CMAKE_CURRENT_SOURCE_DIR}/include/")
set(SOLVER_BWC      "${CMAKE_CURRENT_SOURCE_DIR}/bwc_cvxgen")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${SOLVER_BWC}
  )

set(SOURCES_DS  ${SRC_FOLDER}/dual_arm_control.cpp 
                ${SRC_FOLDER}/dualArmFreeMotionController.cpp
                ${SRC_FOLDER}/dualArmCooperativeController.cpp
                ${SRC_FOLDER}/throwingDS.cpp
                ${SRC_FOLDER}/toss_task_param_estimator.cpp
                )

file(GLOB BWC_SOURCES   "${SOLVER_BWC}/*.c")

add_library(dual_arm_control ${SOURCES_DS} ${BWC_SOURCES} )
target_link_libraries(dual_arm_control ${catkin_LIBRARIES})

add_executable(dual_arm_control_node ${SRC_FOLDER}/dual_arm_control_node.cpp)              
target_link_libraries(dual_arm_control_node dual_arm_control ${catkin_LIBRARIES})
# target_link_libraries(dual_arm_control_node ${catkin_LIBRARIES})


#############################################################################################
add_library(dual_arm_box_control ${SOURCES_DS} ${BWC_SOURCES} )
target_link_libraries(dual_arm_box_control ${catkin_LIBRARIES})

# add_executable(dual_arm_box_control_node ${SRC_FOLDER}/dual_arm_box_control_node.cpp )              
# target_link_libraries(dual_arm_box_control_node dual_arm_box_control ${catkin_LIBRARIES})



### ADD Nodes that use the library
add_dependencies(dual_arm_control_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
${PROJECT_NAME}_gencfg)

