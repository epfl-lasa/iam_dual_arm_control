
#include "../pybind11/include/pybind11/eigen.h"
#include "../pybind11/include/pybind11/pybind11.h"
#include "../pybind11/include/pybind11/stl.h"

#include "../../include/dual_arm_control_iam/DualArmControl.hpp"
#include <iostream>
#include <string>

namespace py = pybind11;

PYBIND11_MODULE(py_dual_arm_control, m) {
  py::class_<DualArmControl>(m, "DualArmControl")
      .def(py::init<double>())// constructor
      .def("loadParamFromFile", &DualArmControl::loadParamFromFile)
      .def("init", &DualArmControl::init)
      .def("getStateMachine", &DualArmControl::getStateMachine)
      .def("updateStateMachine", &DualArmControl::updateStateMachine)
      .def("generateCommands", &DualArmControl::generateCommands)
      .def("getDataToSave", &DualArmControl::getDataToSave);

  py::class_<StateMachine>(m, "StateMachine")
      .def(py::init<>())
      .def_readwrite("goHome", &StateMachine::goHome)
      .def_readwrite("goToAttractors", &StateMachine::goToAttractors)
      .def_readwrite("isThrowing", &StateMachine::isThrowing)
      .def_readwrite("isPlacing", &StateMachine::isPlacing)
      .def_readwrite("isPlaceTossing", &StateMachine::isPlaceTossing)
      .def_readwrite("releaseAndretract", &StateMachine::releaseAndretract)
      .def_readwrite("dualTaskSelector", &StateMachine::dualTaskSelector)
      .def_readwrite("desVtoss", &StateMachine::desVtoss)
      .def_readwrite("desiredVelImp", &StateMachine::desiredVelImp)
      .def_readwrite("placingPosHeight", &StateMachine::placingPosHeight)
      .def_readwrite("releasePosY", &StateMachine::releasePosY)
      .def_readwrite("incrementReleasePos", &StateMachine::incrementReleasePos)
      .def_readwrite("deltaRelPos", &StateMachine::deltaRelPos)
      .def_readwrite("deltaPos", &StateMachine::deltaPos)
      .def_readwrite("trackingFactor", &StateMachine::trackingFactor)
      .def_readwrite("adaptationActive", &StateMachine::adaptationActive);

  py::class_<CommandStruct>(m, "CommandStruct")
      .def(py::init<>())
      .def_property(
          "axisAngleDes",
          [](const CommandStruct& cs) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Vector3f& vec : cs.axisAngleDes) { result.append(vec); }
            return result;
          },
          [](CommandStruct& cs, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { cs.axisAngleDes[i] = lst[i].cast<Eigen::Vector3f>(); }
          })
      .def_property(
          "vDes",
          [](const CommandStruct& cs) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Vector3f& vec : cs.vDes) { result.append(vec); }
            return result;
          },
          [](CommandStruct& cs, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { cs.vDes[i] = lst[i].cast<Eigen::Vector3f>(); }
          })
      .def_property(
          "omegaDes",
          [](const CommandStruct& cs) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Vector3f& vec : cs.omegaDes) { result.append(vec); }
            return result;
          },
          [](CommandStruct& cs, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { cs.omegaDes[i] = lst[i].cast<Eigen::Vector3f>(); }
          })
      .def_property(
          "qd",
          [](const CommandStruct& cs) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Vector4f& vec : cs.qd) { result.append(vec); }
            return result;
          },
          [](CommandStruct& cs, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { cs.qd[i] = lst[i].cast<Eigen::Vector4f>(); }
          })
      .def_property(
          "filteredWrench",
          [](const CommandStruct& cs) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix<float, 6, 1>& vec : cs.filteredWrench) { result.append(vec); }
            return result;
          },
          [](CommandStruct& cs, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { cs.filteredWrench[i] = lst[i].cast<Eigen::Matrix<float, 6, 1>>(); }
          })
      .def_property(
          "whgpSpecific",
          [](const CommandStruct& cs) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix4f& vec : cs.whgpSpecific) { result.append(vec); }
            return result;
          },
          [](CommandStruct& cs, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { cs.whgpSpecific[i] = lst[i].cast<Eigen::Matrix4f>(); }
          })
      .def_property(
          "velEESpecific",
          [](const CommandStruct& cs) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix<float, 6, 1>& vec : cs.velEESpecific) { result.append(vec); }
            return result;
          },
          [](CommandStruct& cs, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { cs.velEESpecific[i] = lst[i].cast<Eigen::Matrix<float, 6, 1>>(); }
          })
      .def_property(
          "appliedWrench",
          [](const CommandStruct& cs) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix<float, 6, 1>& vec : cs.appliedWrench) { result.append(vec); }
            return result;
          },
          [](CommandStruct& cs, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { cs.appliedWrench[i] = lst[i].cast<Eigen::Matrix<float, 6, 1>>(); }
          })
      .def_property(
          "normalVectSurfObj",
          [](const CommandStruct& cs) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Vector3f& vec : cs.normalVectSurfObj) { result.append(vec); }
            return result;
          },
          [](CommandStruct& cs, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { cs.normalVectSurfObj[i] = lst[i].cast<Eigen::Vector3f>(); }
          })
      .def_property(
          "err",
          [](const CommandStruct& cs) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const float& vec : cs.err) { result.append(vec); }
            return result;
          },
          [](CommandStruct& cs, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { cs.err[i] = lst[i].cast<float>(); }
          })
      .def_readwrite("nuWr0", &CommandStruct::nuWr0);

  py::class_<tossingTaskVariables>(m, "tossingTaskVariables")
      .def(py::init<>())
      .def_readwrite("releasePosition", &tossingTaskVariables::releasePosition)
      .def_readwrite("releaseOrientation", &tossingTaskVariables::releaseOrientation)
      .def_readwrite("releaseLinearVelocity", &tossingTaskVariables::releaseLinearVelocity)
      .def_readwrite("releaseAngularVelocity", &tossingTaskVariables::releaseAngularVelocity)
      .def_readwrite("restPosition", &tossingTaskVariables::restPosition)
      .def_readwrite("restOrientation", &tossingTaskVariables::restOrientation);

  py::class_<DataToSave>(m, "DataToSave")
      .def(py::init<>())
      .def_property(
          "robotX",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Vector3f& vec : dts.robotX) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.robotX[i] = lst[i].cast<Eigen::Vector3f>(); }
          })
      .def_property(
          "robotQ",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Vector4f& vec : dts.robotQ) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.robotQ[i] = lst[i].cast<Eigen::Vector4f>(); }
          })
      .def_property(
          "robotVelDesEE",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix<float, 6, 1>& vec : dts.robotVelDesEE) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.robotVelDesEE[i] = lst[i].cast<Eigen::Matrix<float, 6, 1>>(); }
          })
      .def_property(
          "robotVelEE",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix<float, 6, 1>& vec : dts.robotVelEE) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.robotVelEE[i] = lst[i].cast<Eigen::Matrix<float, 6, 1>>(); }
          })
      .def_property(
          "robotVDes",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Vector3f& vec : dts.robotVDes) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.robotVDes[i] = lst[i].cast<Eigen::Vector3f>(); }
          })
      .def_property(
          "robotOmegaDes",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Vector3f& vec : dts.robotOmegaDes) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.robotOmegaDes[i] = lst[i].cast<Eigen::Vector3f>(); }
          })
      .def_property(
          "robotFilteredWrench",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix<float, 6, 1>& vec : dts.robotFilteredWrench) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.robotFilteredWrench[i] = lst[i].cast<Eigen::Matrix<float, 6, 1>>(); }
          })
      .def_property(
          "robotJointsPositions",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix<float, 7, 1>& vec : dts.robotJointsPositions) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.robotJointsPositions[i] = lst[i].cast<Eigen::Matrix<float, 7, 1>>(); }
          })
      .def_property(
          "robotJointsVelocities",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix<float, 7, 1>& vec : dts.robotJointsVelocities) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.robotJointsVelocities[i] = lst[i].cast<Eigen::Matrix<float, 7, 1>>(); }
          })
      .def_property(
          "robotJointsAccelerations",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix<float, 7, 1>& vec : dts.robotJointsAccelerations) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.robotJointsAccelerations[i] = lst[i].cast<Eigen::Matrix<float, 7, 1>>(); }
          })
      .def_property(
          "robotJointsTorques",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix<float, 7, 1>& vec : dts.robotJointsTorques) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.robotJointsTorques[i] = lst[i].cast<Eigen::Matrix<float, 7, 1>>(); }
          })

      // Object
      .def_property(
          "objectWHGpSpecific",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix4f& vec : dts.objectWHGpSpecific) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) { dts.objectWHGpSpecific[i] = lst[i].cast<Eigen::Matrix4f>(); }
          })
      .def_readwrite("objectWHDo", &DataToSave::objectWHDo)
      .def_readwrite("objectXo", &DataToSave::objectXo)
      .def_readwrite("objectQo", &DataToSave::objectQo)
      .def_readwrite("objectVo", &DataToSave::objectVo)
      .def_readwrite("objectWo", &DataToSave::objectWo)
      .def_readwrite("objVelDes", &DataToSave::objVelDes)

      // Target
      .def_readwrite("targetXt", &DataToSave::targetXt)
      .def_readwrite("targetQt", &DataToSave::targetQt)
      .def_readwrite("targetVt", &DataToSave::targetVt)
      .def_readwrite("targetXdLanding", &DataToSave::targetXdLanding)
      .def_readwrite("targetXIntercept", &DataToSave::targetXIntercept)
      .def_readwrite("targetXtStateToGo", &DataToSave::targetXtStateToGo)

      // Tossing tossingTaskVariables
      .def_property(
          "tossVar",
          [](const DataToSave& self) { return self.tossVar; },
          [](DataToSave& self, const tossingTaskVariables& value) { self.tossVar = value; })

      // Task
      .def_readwrite("taskXPlacing", &DataToSave::taskXPlacing)
      .def_readwrite("desiredVelImp", &DataToSave::desiredVelImp)
      .def_readwrite("betaVelMod", &DataToSave::betaVelMod)
      .def_readwrite("dualPathLenAvgSpeed", &DataToSave::dualPathLenAvgSpeed)

      // Cooperative control
      .def_property(
          "cooperativeCtrlForceApplied",
          [](const DataToSave& dts) {
            // Convert the array to a list of lists for Python
            py::list result;
            for (const Eigen::Matrix<float, 6, 1>& vec : dts.cooperativeCtrlForceApplied) { result.append(vec); }
            return result;
          },
          [](DataToSave& dts, const py::list& lst) {
            // Convert the Python list back to the array
            if (py::len(lst) != 2) { throw std::runtime_error("Input list length must be 2."); }
            for (int i = 0; i < 2; ++i) {
              dts.cooperativeCtrlForceApplied[i] = lst[i].cast<Eigen::Matrix<float, 6, 1>>();
            }
          })

      // Free motion control
      .def_readwrite("freeMotionCtrlActivationProximity", &DataToSave::freeMotionCtrlActivationProximity)
      .def_readwrite("freeMotionCtrlActivationNormal", &DataToSave::freeMotionCtrlActivationNormal)
      .def_readwrite("freeMotionCtrlActivationTangent", &DataToSave::freeMotionCtrlActivationTangent)
      .def_readwrite("freeMotionCtrlActivationRelease", &DataToSave::freeMotionCtrlActivationRelease)
      .def_readwrite("freeMotionCtrlActivationRetract", &DataToSave::freeMotionCtrlActivationRetract)

      // DS throwing
      .def_readwrite("dsThrowingActivationProximity", &DataToSave::dsThrowingActivationProximity)
      .def_readwrite("dsThrowingActivationNormal", &DataToSave::dsThrowingActivationNormal)
      .def_readwrite("dsThrowingActivationTangent", &DataToSave::dsThrowingActivationTangent)
      .def_readwrite("dsThrowingActivationToss", &DataToSave::dsThrowingActivationToss)

      // State machine
      .def_readwrite("goHome", &DataToSave::goHome)
      .def_readwrite("goToAttractors", &DataToSave::goToAttractors)
      .def_readwrite("releaseAndretract", &DataToSave::releaseAndretract)
      .def_readwrite("isThrowing", &DataToSave::isThrowing)
      .def_readwrite("isPlacing", &DataToSave::isPlacing)
      .def_readwrite("isContact", &DataToSave::isContact)
      .def_readwrite("desVtoss", &DataToSave::desVtoss);
}
