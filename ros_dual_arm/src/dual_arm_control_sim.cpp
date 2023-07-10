
#include "ros/ros.h"
#include <iostream>
#include <ros/package.h>
#include <sstream>
#include <string>

#include "RosDualArmCommunication.hpp"
#include "dual_arm_control_iam/DualArmControlSim.hpp"

int main(int argc, char** argv) {

  // =================================================================
  // Instantiation of ROS
  // =================================================================

  ros::init(argc, argv, "dual_arm_control_node");
  ros::NodeHandle nh;
  double frequency = 200.0f;
  double dt = 1 / frequency;
  ros::Rate loopRate = frequency;// Ros loop rate [Hz]

  CommandStruct commandGenerated;

  RosDualArmCommunication rosDualArm(nh, frequency);
  rosDualArm.init();

  // =================================================================
  // Instantiation of dual arm control  object
  // =================================================================
  DualArmControlSim dualArmControlSim;

  std::string pathYamlFile = ros::package::getPath(std::string("ros_dual_arm_control")) + "/config/parameters.yaml";
  std::string pathLearnedModelfolder =
      ros::package::getPath(std::string("ros_dual_arm_control")) + "/LearnedModel/model1";
  if (!dualArmControlSim.loadParamFromFile(pathYamlFile, pathLearnedModelfolder)) {
    std::cerr << "Error loading config file (parameters.yaml)" << std::endl;
    return EXIT_FAILURE;
  }

  dualArmControlSim.init();

  // =================================================================================
  // Simulation loop
  // =================================================================================

  int count = 0;
  bool releaseFlag = false;

  // // while (!((posError <= dsThrowingCart.getTolerancePos()) && releaseFlag) && count <= 2500)
  while (nh.ok()) {

    // Compute generated desired motion and forces
    commandGenerated = dualArmControlSim.generateCommands(rosDualArm.firstEigenPassiveDamping_,
                                                          rosDualArm.robotWrench_,
                                                          rosDualArm.eePose_,
                                                          rosDualArm.eeOrientation_,
                                                          rosDualArm.objectPose_,
                                                          rosDualArm.objectOrientation_,
                                                          rosDualArm.targetPose_,
                                                          rosDualArm.targetOrientation_,
                                                          rosDualArm.eeVelLin_,
                                                          rosDualArm.eeVelAng_,
                                                          rosDualArm.jointPosition_,
                                                          rosDualArm.jointVelocity_,
                                                          rosDualArm.jointTorques_,
                                                          rosDualArm.robotBasePos_,
                                                          rosDualArm.robotBaseOrientation_);

    // Publish the commands to be exectued
    rosDualArm.publishCommands(commandGenerated.axisAngleDes, commandGenerated.vDes, commandGenerated.qd);
    //   // // Publish data through topics for analysis
    //   // publishData();
    //   // // Log data
    //   // if (startlogging_) { saveData(); }

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}