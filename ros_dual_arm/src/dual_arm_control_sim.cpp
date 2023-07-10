
#include "ros/ros.h"
#include <iostream>
#include <ros/package.h>
#include <sstream>
#include <string>

// #include "RosDualArm.hpp"
#include "dual_arm_control_iam/DualArmControlSim.hpp"

int main(int argc, char** argv) {

  // =================================================================
  // Instantiation of ROS
  // =================================================================

  ros::init(argc, argv, "dual_arm_control_node");
  ros::NodeHandle nh;
  double frequency = 200.0f;

  // RosDualArmCommunication dualArmRos(nh, frequency);

  // // =================================================================
  // // Instantiation of dual arm control  object
  // // =================================================================
  // DualArmControlSim DualArmControlSim;

  // std::string pathYamlFile = "./../config/parameters.yaml";
  // std::string pathLearnedModelfolder = "./../LearnedModel/model1";
  // if (!DualArmControlSim.loadParamFromFile(pathYamlFile, pathLearnedModelfolder)) {
  //   std::cerr << "Error loading config file (parameters.yaml)" << std::endl;
  //   return EXIT_FAILURE;
  // }

  // DualArmControlSim.init();

  // // =================================================================
  // // =================================================================

  // // =================================================================================
  // // Simulation loop
  // // =================================================================================

  // int count = 0;
  // bool releaseFlag = false;
  // double dt = DualArmControlSim.getPeriod();
  // float firstEigenPassiveDamping[NB_ROBOTS];
  // Eigen::Vector3f eePose, objectPose, targetPose, eeVelLin, eeVelAng;
  // Eigen::Vector4f eeOrientation, objectOrientation, targetOrientation;
  // Vector7f jointPosition, jointVelocity, jointTorques;
  // Eigen::Matrix<float, 6, 1> robotWrench;
  // float toolOffsetFromEE[NB_ROBOTS];

  // // while (!((posError <= dsThrowingCart.getTolerancePos()) && releaseFlag) && count <= 2500)
  // while (nh_.ok()) {

  //   // Update first eigen value of the passive ds controller and its updated value
  //   // firstEigenPassiveDamping = TODO;

  //   // // Update the poses of the robots and the object TODO
  //   // updatePoses();

  //   // Compute generated desired motion and forces
  //   DualArmControlSim.generateCommands(firstEigenPassiveDamping,
  //                                      robotWrench,
  //                                      eePose,
  //                                      eeOrientation,
  //                                      objectPose,
  //                                      objectOrientation,
  //                                      targetPose,
  //                                      targetOrientation,
  //                                      eeVelLin,
  //                                      eeVelAng,
  //                                      jointPosition,
  //                                      jointVelocity,
  //                                      jointTorques);

  //   // // Publish the commands to be exectued
  //   // publishCommands();
  //   // // Publish data through topics for analysis
  //   // publishData();
  //   // // Log data
  //   // if (startlogging_) { saveData(); }

  //   // ros::spinOnce();
  //   // loopRate_.sleep();
  //   // cycleCount_++;

  //   // // Estimation of the running period
  //   // auto stop = std::chrono::high_resolution_clock::now();
  //   // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

  //   releaseFlag = DualArmControlSim.getReleaseFlag();
  // count++;
  // }

  return 0;
}