cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

project(dual_arm_control_iam LANGUAGES C CXX VERSION 1.0.0)


find_package(Eigen3 QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(Eigen3_FOUND)
  set(FIND_EIGEN "find_package(Eigen3 REQUIRED NO_CMAKE_PACKAGE_REGISTRY)")
else()
  find_package(Eigen3 MODULE REQUIRED)
  set(FIND_EIGEN "find_package(Eigen3 MODULE REQUIRED)")
endif()

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
## Specify locations of BWC solver
set(SOLVER_BWC      "${CMAKE_CURRENT_SOURCE_DIR}/bwc_cvxgen")
## Specify locations of header files
set(HEADERS_FOLDER  "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
## Specify locations of source files
set(SRC_FOLDER      "${CMAKE_CURRENT_SOURCE_DIR}/src")

set(H_FILES
    ${HEADERS_FOLDER}/DualArmControl.hpp
    ${HEADERS_FOLDER}/DualArmFreeMotionController.hpp
    ${HEADERS_FOLDER}/DualArmCooperativeController.hpp
    ${HEADERS_FOLDER}/ThrowingDS.hpp
    ${HEADERS_FOLDER}/TossTaskParamEstimator.hpp
    ${HEADERS_FOLDER}/TossingTarget.hpp
    ${HEADERS_FOLDER}/RobotVariables.hpp
    ${HEADERS_FOLDER}/ObjectToGrasp.hpp

    ${SOLVER_BWC}/bwc_solver.h
)
    
   

set(TOOLS_FILE
  ${HEADERS_FOLDER}/tools/Utils.hpp
  ${HEADERS_FOLDER}/tools/PdfGMR.hpp
  ${HEADERS_FOLDER}/tools/KalmanFilter3DVelFromPosEstimator.hpp
  ${HEADERS_FOLDER}/tools/KalmanFilter.hpp
  ${HEADERS_FOLDER}/tools/FirstOrderFilter.hpp
)
    
set(SRCS
    ${SRC_FOLDER}/DualArmControl.cpp
    ${SRC_FOLDER}/DualArmFreeMotionController.cpp
    ${SRC_FOLDER}/DualArmCooperativeController.cpp
    ${SRC_FOLDER}/ThrowingDS.cpp
    ${SRC_FOLDER}/TossTaskParamEstimator.cpp
    )

set(BWC_SOURCES
    ${SOLVER_BWC}/bwc_ldl.c
    ${SOLVER_BWC}/bwc_matrix_support.c
    ${SOLVER_BWC}/bwc_solver.c
    ${SOLVER_BWC}/bwc_testsolver.c
    ${SOLVER_BWC}/bwc_util.c
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIR}
    ${SOLVER_BWC}
    ${SG_DIFFERENTATION_INCLUDE_DIRS}
)



add_library(${PROJECT_NAME} SHARED ${H_FILES} ${SRCS} ${BWC_SOURCES} )

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES)

target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

if(TARGET Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${YAML_CPP_LIBRARIES} sg_differentiation)
else()
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}" sg_differentiation)
endif()


install(FILES ${H_FILES} DESTINATION include/${PROJECT_NAME})
install(FILES ${TOOLS_FILE} DESTINATION include/${PROJECT_NAME}/tools)
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
