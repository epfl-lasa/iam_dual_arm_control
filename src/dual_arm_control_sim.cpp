
#include <iostream>
#include <sstream>
#include <string>

#include "dual_arm_control_iam/DualArmControlSim.hpp"

int main(int argc, char** argv) {

  // =================================================================
  // Instantiation of dual arm control  object
  // =================================================================
  DualArmControlSim DualArmControlSim;

  std::string pathYamlFile = "./../config/parameters.yaml";
  if (!DualArmControlSim.loadParamFromFile(pathYamlFile)) {
    std::cerr << "Error loading config file (parameters.yaml)" << std::endl;
    return EXIT_FAILURE;
  }

  // =================================================================
  // =================================================================
  // =================================================================
  // =================================================================

  //   dsThrowingCart.initTossingDsParam();
  //   dsThrowingCart.init();

  //   // =================================================================
  //   // Get states
  //   // =================================================================
  //   Eigen::Vector3d curPos = dsThrowingCart.getCurPos();
  //   Eigen::Vector4d curOrient = dsThrowingCart.getCurOrient();
  //   Eigen::Vector3d curLinVel = dsThrowingCart.getCurLinVel();
  //   Eigen::Vector3d curAngVel = dsThrowingCart.getCurAngVel();
  //   Eigen::Vector3d restPos = dsThrowingCart.getRestPos();
  //   Eigen::Vector4d releaseOrient = dsThrowingCart.getReleaseOrient();
  //   Eigen::Vector3d releasePos = dsThrowingCart.getReleasePos();
  //   Eigen::Vector3d releaseLinVel = dsThrowingCart.getReleaseLinVel();

  //   // Get position error && orientation error
  //   double posError = (curPos - restPos).norm();
  //   Eigen::Matrix3d rotCurrToReleased = Utils<double>::quaternionToRotationMatrix(releaseOrient).transpose()
  //       * Utils<double>::quaternionToRotationMatrix(curOrient);

  //   // =================================================================
  //   // Log simulation data for analysis
  //   // =================================================================
  //   std::ofstream OutRecordTask;

  //   // Create log file in the build dir
  //   OutRecordTask.open("./log_task.txt");

  //   Vector6d outMotion = Eigen::VectorXd::Zero(6);
  //   Vector6d accEEPrev = Eigen::VectorXd::Zero(6);
  //   Vector6d velEEPrev = Eigen::VectorXd::Zero(6);
  //   Vector6d accEE = Eigen::VectorXd::Zero(6);
  //   Vector6d velEE = Eigen::VectorXd::Zero(6);

  // =================================================================================
  // Simulation loop
  // =================================================================================

  int count = 0;
  bool releaseFlag = false;// DualArmControlSim.getReleaseFlag();
  double dt = DualArmControlSim.getPeriod();
  float firstEigenPassiveDamping[NB_ROBOTS];
  Eigen::Vector3f EEPose;
  Eigen::Vector4f EEOrientation;
  float toolOffsetFromEE[NB_ROBOTS];

  // while (!((posError <= dsThrowingCart.getTolerancePos()) && releaseFlag) && count <= 2500)
  while (!releaseFlag && count <= 2500) {

    // // Keyboard commands
    // updateStatesMachines();

    // Update first eigen value of the passive ds controller and its updated value
    // firstEigenPassiveDamping = TODO;

    // // Update the poses of the robots and the object
    // updatePoses();

    // Compute generated desired motion and forces
    DualArmControlSim.generateCommands(EEPose, EEOrientation);

    // // Publish the commands to be exectued
    // publishCommands();
    // // Publish data through topics for analysis
    // publishData();
    // // Log data
    // if (startlogging_) { saveData(); }

    // ros::spinOnce();
    // loopRate_.sleep();
    // cycleCount_++;

    // // Estimation of the running period
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    releaseFlag = DualArmControlSim.getReleaseFlag();
    count++;
  }

  //   bool releaseFlag = dsThrowingCart.getReleaseFlag();
  //   int count = 0;
  //   double dt = dsThrowingCart.getPeriod();

  //   std::cout << "Tossing DS starting the loop ... " << std::endl;
  //   while (!((posError <= dsThrowingCart.getTolerancePos()) && releaseFlag) && count <= 2500) {

  //     // Generate the DS motion
  //     // Get the desired acceleration (is2ndOrder=True) or velocity (is2nOrder=False)
  //     outMotion = dsThrowingCart.getThrowingMotion(curPos, curOrient, curLinVel, curAngVel);
  //     releaseFlag = dsThrowingCart.getReleaseFlag();

  //     // --------------------------
  //     // For simulation only!! Retrieve velocity or acceleration from desired motion
  //     // --------------------------
  //     if (dsThrowingCart.getDsOrder()) {
  //       accEE = outMotion;
  //       velEE += dt * (accEE);
  //     } else {
  //       velEE = outMotion;
  //       accEE = 1. / dt * (velEE - velEEPrev);
  //     }

  //     accEEPrev = accEE;
  //     velEEPrev = velEE;

  //     // ---------------------------
  //     // Update the robot states :
  //     // ---------------------------
  //     curLinVel = velEE.head(3);
  //     curAngVel = velEE.tail(3);

  //     // Update pose for simulation
  //     updatePoseFromVelocityTwist(dt, velEE, curPos, curOrient);

  //     // Get position error && orientation error
  //     posError = (curPos - restPos).norm();
  //     rotCurrToReleased = Utils<double>::quaternionToRotationMatrix(releaseOrient).transpose()
  //         * Utils<double>::quaternionToRotationMatrix(curOrient);
  //     Eigen::Vector3d orientError = Utils<double>::getOrientationErrorCur2Des(rotCurrToReleased);

  //     std::cout << "--------------------------------------------- " << std::endl;
  //     std::cout << " count : [" << count << "] des accel is : \t" << accEE.transpose() << std::endl;
  //     std::cout << " count : [" << count << "] des velo is : \t" << velEE.transpose() << std::endl;
  //     std::cout << " count : [" << count << "] curPos is : \t" << curPos.transpose() << " curOrient is : \t"
  //               << curOrient.transpose() << std::endl;
  //     std::cout << " count : [" << count << "] Release flag is : \t" << releaseFlag << std::endl;

  //     // log simulation data
  //     // --------------------
  //     OutRecordTask << (double) (count * dt) << "	";
  //     OutRecordTask << curPos.transpose() << "	" << curOrient.transpose() << "	";
  //     OutRecordTask << releasePos.transpose() << "	" << releaseOrient.transpose() << "	";
  //     OutRecordTask << restPos.transpose() << "	" << dsThrowingCart.getRestOrient().transpose() << "	";
  //     OutRecordTask << (curPos - releasePos).transpose() << "	" << orientError.transpose() << "	";
  //     OutRecordTask << velEE.transpose() << "	" << accEE.transpose() << "	";
  //     OutRecordTask << dsThrowingCart.getActivationRadial() << "	" << dsThrowingCart.getActivationNormal() << "	"
  //                   << dsThrowingCart.getActivationTangent() << "	" << dsThrowingCart.getActivationRetract() << "	";
  //     OutRecordTask << releaseFlag << "	" << releaseLinVel.transpose() << std::endl;

  //     count++;
  //   }
  //   OutRecordTask.close();

  return 0;
}