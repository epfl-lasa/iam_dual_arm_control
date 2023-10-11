cmake_minimum_required(VERSION 3.1.0)

set (CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")


project(wrap_dynamical_system)

set(PY_PROJECT py_dual_arm_control)
set(CPP_LIBRARY dual_arm_control_iam)

find_package(pybind11 REQUIRED)

find_package(Eigen3 QUIET NO_CMAKE_PACKAGE_REGISTRY)
if(Eigen3_FOUND)
  set(FIND_EIGEN "find_package(Eigen3 REQUIRED NO_CMAKE_PACKAGE_REGISTRY)")
else()
  find_package(Eigen3 MODULE REQUIRED)
  set(FIND_EIGEN "find_package(Eigen3 MODULE REQUIRED)")
endif()

# ________________ PYBIND ________________

add_subdirectory(pybind11)

pybind11_add_module(${PY_PROJECT} pythonApi/py_dual_arm_control.cpp)
target_link_libraries(${PY_PROJECT} PUBLIC ${CPP_LIBRARY} pybind11::module Eigen3::Eigen)
# set_target_properties(${PY_PROJECT} PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}" SUFFIX "${PYTHON_MODULE_EXTENSION}")
set_target_properties(${PY_PROJECT} PROPERTIES OUTPUT_NAME ${PY_PROJECT})