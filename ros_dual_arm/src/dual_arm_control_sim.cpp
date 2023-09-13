
#include "ros/ros.h"
#include <iostream>
#include <ros/package.h>
#include <sstream>
#include <string>
#include <vector>

#include "DataLogging.hpp"
#include "RosDualArmCommunication.hpp"
#include "dual_arm_control_iam/DualArmControlSim.hpp"
#include "keyboard_interaction.hpp"

enum Robot { LEFT = 0, RIGHT = 1 };
const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

void saveData(DataLogging& dataLog, int cycleCount, double dt, DataToSave dataToSave) {
  Eigen::Vector3f xgrL = dataToSave.objectWHGpSpecific[LEFT].block(0, 3, 3, 1);
  Eigen::Vector3f xgrR = dataToSave.objectWHGpSpecific[RIGHT].block(0, 3, 3, 1);
  Eigen::Vector4f qgrL =
      Utils<float>::rotationMatrixToQuaternion(dataToSave.objectWHGpSpecific[LEFT].block(0, 0, 3, 3));
  Eigen::Vector4f qgrR =
      Utils<float>::rotationMatrixToQuaternion(dataToSave.objectWHGpSpecific[RIGHT].block(0, 0, 3, 3));

  Eigen::MatrixXf powerLeft = dataToSave.robotJointsTorques[LEFT].transpose() * dataToSave.robotJointsVelocities[LEFT];
  Eigen::MatrixXf powerRight =
      dataToSave.robotJointsTorques[RIGHT].transpose() * dataToSave.robotJointsVelocities[RIGHT];
  Eigen::Matrix4f wHDoObject = dataToSave.objectWHDo;

  // cycle time
  dataLog.outRecordPose << (float) (cycleCount * dt) << ", ";
  dataLog.outRecordPose << dataToSave.robotX[LEFT].transpose().format(CSVFormat) << " , "
                        << dataToSave.robotQ[LEFT].transpose().format(CSVFormat) << " , ";// left end-effector
  dataLog.outRecordPose << dataToSave.robotX[RIGHT].transpose().format(CSVFormat) << " , "
                        << dataToSave.robotQ[RIGHT].transpose().format(CSVFormat) << " , ";// right end-effector
  dataLog.outRecordPose << dataToSave.objectXo.transpose().format(CSVFormat) << " , "
                        << dataToSave.objectQo.transpose().format(CSVFormat) << " , ";// object
  dataLog.outRecordPose << wHDoObject(0, 3) << " , " << wHDoObject(1, 3) << " , " << wHDoObject(2, 3)
                        << " , ";// desired object
  dataLog.outRecordPose << xgrL.transpose().format(CSVFormat) << " , " << qgrL.transpose().format(CSVFormat)
                        << " , ";// left  grasping point
  dataLog.outRecordPose << xgrR.transpose().format(CSVFormat) << " , " << qgrR.transpose().format(CSVFormat)
                        << " , ";// right grasping point
  dataLog.outRecordPose << dataToSave.tossVar.releasePosition.transpose().format(CSVFormat) << " , "
                        << dataToSave.tossVar.releaseOrientation.transpose().format(CSVFormat) << " , ";// release pose
  dataLog.outRecordPose << dataToSave.tossVar.restPosition.transpose().format(CSVFormat) << " , "
                        << dataToSave.tossVar.restOrientation.transpose().format(CSVFormat) << " , ";// rest pose
  dataLog.outRecordPose << dataToSave.targetXt.transpose().format(CSVFormat) << " , "
                        << dataToSave.targetQt.transpose().format(CSVFormat) << " , ";// target pose
  dataLog.outRecordPose << dataToSave.targetXdLanding.transpose().format(CSVFormat) << " , "
                        << dataToSave.targetXIntercept.transpose().format(CSVFormat)
                        << " , ";// landing and intercept position
  dataLog.outRecordPose << dataToSave.targetXtStateToGo.transpose().format(CSVFormat) << " , "
                        << dataToSave.taskXPlacing.transpose().format(CSVFormat) << std::endl;// target state to go

  dataLog.outRecordVel << (float) (cycleCount * dt) << ", ";
  dataLog.outRecordVel << dataToSave.robotVelDesEE[LEFT].transpose().format(CSVFormat) << " , "
                       << dataToSave.robotVelDesEE[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog.outRecordVel << dataToSave.robotVelEE[LEFT].transpose().format(CSVFormat) << " , "
                       << dataToSave.robotVelEE[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog.outRecordVel << dataToSave.robotVDes[LEFT].transpose().format(CSVFormat) << " , "
                       << dataToSave.robotVDes[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog.outRecordVel << dataToSave.robotOmegaDes[LEFT].transpose().format(CSVFormat) << " , "
                       << dataToSave.robotOmegaDes[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog.outRecordVel << dataToSave.objectVo.transpose().format(CSVFormat) << " , "
                       << dataToSave.objectWo.transpose().format(CSVFormat) << " , ";
  dataLog.outRecordVel << dataToSave.objVelDes.transpose().format(CSVFormat) << " , ";
  dataLog.outRecordVel << dataToSave.tossVar.releaseLinearVelocity.transpose().format(CSVFormat) << " , "
                       << dataToSave.tossVar.releaseAngularVelocity.transpose().format(CSVFormat) << " , ";
  dataLog.outRecordVel << dataToSave.targetVt.transpose().format(CSVFormat) << std::endl;

  dataLog.outRecordEfforts << (float) (cycleCount * dt) << ", ";
  dataLog.outRecordEfforts << dataToSave.robotFilteredWrench[LEFT].transpose().format(CSVFormat) << " , ";
  dataLog.outRecordEfforts << dataToSave.robotFilteredWrench[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog.outRecordEfforts << dataToSave.cooperativeCtrlForceApplied[LEFT].transpose().format(CSVFormat) << " , ";
  dataLog.outRecordEfforts << dataToSave.cooperativeCtrlForceApplied[RIGHT].transpose().format(CSVFormat) << std::endl;

  dataLog.outRecordTasks << (float) (cycleCount * dt) << ", ";
  dataLog.outRecordTasks << dataToSave.desiredVelImp << " , " << dataToSave.desVtoss << " , ";
  dataLog.outRecordTasks << dataToSave.goHome << " , " << dataToSave.goToAttractors << " , "
                         << dataToSave.releaseAndretract << " , " << dataToSave.isThrowing << " , "
                         << dataToSave.isPlacing << " , " << dataToSave.isContact << " , ";
  dataLog.outRecordTasks << dataToSave.freeMotionCtrlActivationProximity << " , "
                         << dataToSave.freeMotionCtrlActivationNormal << " , "
                         << dataToSave.freeMotionCtrlActivationTangent << " , "
                         << dataToSave.freeMotionCtrlActivationRelease << " , "
                         << dataToSave.freeMotionCtrlActivationRetract << " , ";
  dataLog.outRecordTasks << dataToSave.dsThrowingActivationProximity << " , " << dataToSave.dsThrowingActivationNormal
                         << " , " << dataToSave.dsThrowingActivationTangent << " , "
                         << dataToSave.dsThrowingActivationToss << " , ";
  dataLog.outRecordTasks << dataToSave.betaVelMod << " , " << dataToSave.dualPathLenAvgSpeed.transpose() << std::endl;

  dataLog.outRecordJointStates << (float) (cycleCount * dt) << ", ";
  dataLog.outRecordJointStates << dataToSave.robotJointsPositions[LEFT].transpose().format(CSVFormat) << " , "
                               << dataToSave.robotJointsPositions[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog.outRecordJointStates << dataToSave.robotJointsVelocities[LEFT].transpose().format(CSVFormat) << " , "
                               << dataToSave.robotJointsVelocities[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog.outRecordJointStates << dataToSave.robotJointsAccelerations[LEFT].transpose().format(CSVFormat) << " , "
                               << dataToSave.robotJointsAccelerations[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog.outRecordJointStates << dataToSave.robotJointsTorques[LEFT].transpose().format(CSVFormat) << " , "
                               << dataToSave.robotJointsTorques[RIGHT].transpose().format(CSVFormat) << " , ";
  dataLog.outRecordJointStates << powerLeft(0, 0) << " , " << powerRight(0, 0) << std::endl;
}

int main(int argc, char** argv) {

  // =================================================================
  // Instantiation of ROS
  // =================================================================
  ros::init(argc, argv, "ros_dual_arm_control");
  ros::NodeHandle nh;
  double frequency = 200.0f;
  double dt = 1 / frequency;
  ros::Rate loopRate = frequency;// Ros loop rate [Hz]

  CommandStruct commandGenerated;

  RosDualArmCommunication rosDualArm(nh, frequency);
  rosDualArm.init();

  // =================================================================
  // Instantiation of dual arm control object
  // =================================================================
  DualArmControlSim dualArmControlSim(dt);

  std::string pathYamlFile = ros::package::getPath(std::string("ros_dual_arm_control")) + "/config/parameters.yaml";
  std::string pathLearnedModelfolder =
      ros::package::getPath(std::string("ros_dual_arm_control")) + "/LearnedModel/model1";
  if (!dualArmControlSim.loadParamFromFile(pathYamlFile, pathLearnedModelfolder)) {
    std::cerr << "Error loading config file (parameters.yaml)" << std::endl;
    return EXIT_FAILURE;
  }

  dualArmControlSim.init();

  // =================================================================
  // Instantiation data recording
  // =================================================================

  DataLogging dataLog;
  // Data recording:
  dataLog.init(ros::package::getPath(std::string("ros_dual_arm_control")) + "/Data");

  DataToSave dataToSave;

  // =================================================================================
  // Simulation loop
  // =================================================================================

  int cycleCount = 0;
  bool releaseFlag = false;

  keyboardinteraction::InteractionVar interactionVar;
  interactionVar.resetLogging = false;
  interactionVar.startLogging = false;
  StateMachine stateMachine;

  while (nh.ok()) {
    interactionVar.stateMachine = dualArmControlSim.getStateMachine();
    interactionVar.conveyorBeltState = rosDualArm.getConveyorBeltStatus();
    interactionVar = keyboardinteraction::getKeyboard(interactionVar);
    dualArmControlSim.updateStateMachine(interactionVar.stateMachine);
    rosDualArm.updateConveyorBeltStatus(interactionVar.conveyorBeltState);
    if (interactionVar.resetLogging) {
      dataLog.reset(ros::package::getPath(std::string("ros_dual_arm_control")) + "/Data");
      interactionVar.resetLogging = false;
    }

    // Get the first eigen value of the passive ds controller and its updated value
    rosDualArm.updatePassiveDSDamping();

    // Compute generated desired motion and forces
    commandGenerated = dualArmControlSim.generateCommands(rosDualArm.getFirstEigenPassiveDamping(),
                                                          rosDualArm.getRobotWrench(),
                                                          rosDualArm.getEePose(),
                                                          rosDualArm.getEeOrientation(),
                                                          rosDualArm.getObjectPose(),
                                                          rosDualArm.getObjectOrientation(),
                                                          rosDualArm.getTargetPose(),
                                                          rosDualArm.getTargetOrientation(),
                                                          rosDualArm.getEeVelLin(),
                                                          rosDualArm.getEeVelAng(),
                                                          rosDualArm.getJointPosition(),
                                                          rosDualArm.getJointVelocity(),
                                                          rosDualArm.getJointTorques(),
                                                          rosDualArm.getRobotBasePos(),
                                                          rosDualArm.getRobotBaseOrientation(),
                                                          cycleCount);

    // Publish the commands to be exectued
    rosDualArm.publishCommands(commandGenerated.axisAngleDes, commandGenerated.vDes, commandGenerated.qd);
    rosDualArm.publishData(commandGenerated.vDes,
                           commandGenerated.omegaDes,
                           commandGenerated.qd,
                           commandGenerated.filteredWrench,
                           commandGenerated.whgpSpecific,
                           commandGenerated.velEESpecific,
                           commandGenerated.appliedWrench,
                           commandGenerated.normalVectSurfObj,
                           commandGenerated.err,
                           commandGenerated.nuWr0);

    rosDualArm.sendConveyorBeltSpeed(cycleCount);

    // Log data
    dataToSave = dualArmControlSim.getDataToSave();
    if (interactionVar.startLogging) { saveData(dataLog, cycleCount, dt, dataToSave); }

    ros::spinOnce();
    loopRate.sleep();
    cycleCount++;
  }

  // Close the data logging files
  dataLog.closeFiles();

  return 0;
}