#pragma once

#ifndef _DATA_LOGGING_H_
#define _DATA_LOGGING_H_

#include "ros/ros.h"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>

using namespace std;
//
class data_logging {

public:
  float SimTime;
  std::string _log_pose;
  std::string _log_velo;
  std::string _log_efforts;
  std::string _log_tasks;
  std::string _log_jts_states;

  // data logging
  std::string _DataID;
  std::ofstream _OutRecord_pose;
  std::ofstream _OutRecord_velo;
  std::ofstream _OutRecord_efforts;
  std::ofstream _OutRecord_tasks;
  std::ofstream _OutRecord_jts_states;

  data_logging() {};
  ~data_logging() {};

  bool datalog_init(std::string path2Datafolder) {
    //
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
    std::string DataID = ss.str();
    _OutRecord_pose.open(path2Datafolder + "/log_task_pose_" + DataID + ".csv");
    _OutRecord_velo.open(path2Datafolder + "/log_task_velo_" + DataID + ".csv");
    _OutRecord_efforts.open(path2Datafolder + "/log_robot_efforts_" + DataID + ".csv");
    _OutRecord_tasks.open(path2Datafolder + "/log_robot_tasks_" + DataID + ".csv");
    _OutRecord_jts_states.open(path2Datafolder + "/log_joints_states_" + DataID + ".csv");

    if (!_OutRecord_pose.is_open()) {
      ROS_ERROR("[_OutRecord_pose]: Cannot open output data files, the Data directory might be missing");
      return false;
    }
    if (!_OutRecord_velo.is_open()) {
      ROS_ERROR("[_OutRecord_velo]: Cannot open output data files, the Data directory might be missing");
      return false;
    }
    if (!_OutRecord_efforts.is_open()) {
      ROS_ERROR("[_OutRecord_efforts]: Cannot open output data files, the Data directory might be missing");
      return false;
    }
    if (!_OutRecord_tasks.is_open()) {
      ROS_ERROR("[_OutRecord_tasks]: Cannot open output data files, the Data directory might be missing");
      return false;
    }
    if (!_OutRecord_jts_states.is_open()) {
      ROS_ERROR("[_OutRecord_jts_states]: Cannot open output data files, the Data directory might be missing");
      return false;
    }
    writeColumns();
  }

  bool datalog_reset(std::string path2Datafolder) {
    //
    this->Close_files();

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
    std::string DataID = ss.str();
    _OutRecord_pose.open(path2Datafolder + "/log_task_pose_" + DataID + ".csv");
    _OutRecord_velo.open(path2Datafolder + "/log_task_velo_" + DataID + ".csv");
    _OutRecord_efforts.open(path2Datafolder + "/log_robot_efforts_" + DataID + ".csv");
    _OutRecord_tasks.open(path2Datafolder + "/log_robot_tasks_" + DataID + ".csv");
    _OutRecord_jts_states.open(path2Datafolder + "/log_joints_states_" + DataID + ".csv");

    if (!_OutRecord_pose.is_open()) {
      ROS_ERROR("[_OutRecord_pose]: Cannot open output data files, the Data directory might be missing");
      return false;
    }
    if (!_OutRecord_velo.is_open()) {
      ROS_ERROR("[_OutRecord_velo]: Cannot open output data files, the Data directory might be missing");
      return false;
    }
    if (!_OutRecord_efforts.is_open()) {
      ROS_ERROR("[_OutRecord_efforts]: Cannot open output data files, the Data directory might be missing");
      return false;
    }
    if (!_OutRecord_tasks.is_open()) {
      ROS_ERROR("[_OutRecord_tasks]: Cannot open output data files, the Data directory might be missing");
      return false;
    }
    if (!_OutRecord_jts_states.is_open()) {
      ROS_ERROR("[_OutRecord_jts_states]: Cannot open output data files, the Data directory might be missing");
      return false;
    }
    writeColumns();
  }

  void Write_Data() {};

  void writeColumns() {

    // POSE
    _OutRecord_pose << "timestamp"
                    << ",";
    _OutRecord_pose << "eePosXL" << "," << "eePosYL" << "," << "eePosZL" << ",";
    _OutRecord_pose << "eeQuatWL" << "," << "eeQuatXL" << "," << "eeQuatYL" << "," << "eeQuatZL" << ",";
    _OutRecord_pose << "eePosXR" << "," << "eePosYR" << "," << "eePosZR" << ",";
    _OutRecord_pose << "eeQuatWR" << "," << "eeQuatXR" << "," << "eeQuatYR" << "," << "eeQuatZR" << ",";
    _OutRecord_pose << "objPosX" << "," << "objPosY" << "," << "objPosZ" << ",";
    _OutRecord_pose << "objQuatW" << "," << "objQuatX" << "," << "objQuatY" << "," << "objQuatZ" << ",";
    _OutRecord_pose << "objPosDesX" << "," << "objPosDesY" << "," << "objPosDesZ" << ",";
    _OutRecord_pose << "objGrapPosXL" << "," << "objGrapPosYL" << "," << "objGrapPosZL" << ",";
    _OutRecord_pose << "objGraspQuatWL" << "," << "objGraspQuatXL" << "," << "objGraspQuatYL" << "," << "objGraspQuatZL"
                    << ",";
    _OutRecord_pose << "objGrapPosXR" << "," << "objGrapPosYR" << "," << "objGrapPosZR" << ",";
    _OutRecord_pose << "objGraspQuatWR" << "," << "objGraspQuatXR" << "," << "objGraspQuatYR" << "," << "objGraspQuatZR"
                    << ",";
    _OutRecord_pose << "tossVarReleasePosXL" << "," << "tossVarReleasePosYL" << "," << "tossVarReleasePosZL" << ",";
    _OutRecord_pose << "tossVarReleaseQuatWL" << "," << "tossVarReleaseQuatXL" << "," << "tossVarReleaseQuatYL" << ","
                    << "tossVarReleaseQuatZL" << ",";
    _OutRecord_pose << "tossVarRestPosXL" << "," << "tossVarRestPosYL" << "," << "tossVarRestPosZL" << ",";
    _OutRecord_pose << "tossVarRestQuatWL" << "," << "tossVarRestQuatXL" << "," << "tossVarRestQuatYL" << ","
                    << "tossVarRestQuatZL" << ",";
    _OutRecord_pose << "targetPosXL" << "," << "targetPosYL" << "," << "targetPosZL" << ",";
    _OutRecord_pose << "targetQuatWL" << "," << "targetQuatXL" << "," << "targetQuatYL" << "," << "targetQuatZL" << ",";
    _OutRecord_pose << "targetlandingDesPosXL" << "," << "targetlandingDesPosYL" << "," << "targetlandingDesPosZL"
                    << ",";
    _OutRecord_pose << "targetInterceptPosXL" << "," << "targetInterceptPosYL" << "," << "targetInterceptPosZL" << ",";
    _OutRecord_pose << "targetStateToGoPosXL" << "," << "targetStateToGoPosYL" << "," << "targetStateToGoPosZL" << ",";
    _OutRecord_pose << "xDoPlacingPosXL" << "," << "xDoPlacingPosYL" << "," << "xDoPlacingPosZL" << std::endl;

    // VELOCITIES
    _OutRecord_velo << "timestamp"
                    << ",";
    _OutRecord_velo << "eeVelLinDesXL" << "," << "eeVelLinDesYL" << "," << "eeVelLinDesZL" << ",";
    _OutRecord_velo << "eeVelAngDesXL" << "," << "eeVelAngDesYL" << "," << "eeVelAngDesZL" << ",";
    _OutRecord_velo << "eeVelLinDesXR" << "," << "eeVelLinDesYR" << "," << "eeVelLinDesZR" << ",";
    _OutRecord_velo << "eeVelAngDesXR" << "," << "eeVelAngDesYR" << "," << "eeVelAngDesZR" << ",";
    _OutRecord_velo << "eeVelLinCurXL" << "," << "eeVelLinCurYL" << "," << "eeVelLinCurZL" << ",";
    _OutRecord_velo << "eeVelAngCurXL" << "," << "eeVelLinCurYL" << "," << "eeVelAngCurZL" << ",";
    _OutRecord_velo << "eeVelLinCurXR" << "," << "eeVelLinCurYR" << "," << "eeVelLinCurZR" << ",";
    _OutRecord_velo << "eeVelAngCurXR" << "," << "eeVelLinCurYR" << "," << "eeVelAngCurZR" << ",";
    _OutRecord_velo << "taskVelLinCurXL" << "," << "taskVelLinCurYL" << "," << "taskVelLinCurZL" << ",";
    _OutRecord_velo << "taskVelLinCurXR" << "," << "taskVelLinCurYR" << "," << "taskVelLinCurZR" << ",";
    _OutRecord_velo << "taskVelAngCurXL" << "," << "taskVelAngCurYL" << "," << "taskVelAngCurZL" << ",";
    _OutRecord_velo << "taskVelAngCurXR" << "," << "taskVelAngCurYR" << "," << "taskVelAngCurZR" << ",";
    _OutRecord_velo << "objVelLinX" << "," << "objVelLinY" << "," << "objVelLinZ" << ",";
    _OutRecord_velo << "objVelAngX" << "," << "objVelAngY" << "," << "objVelAngZ" << ",";
    _OutRecord_velo << "objVelLinDesX" << "," << "objVelLinDesY" << "," << "objVelLinDesZ" << ",";
    _OutRecord_velo << "objVelAngDesX" << "," << "objVelAngDesY" << "," << "objVelAngDesZ" << ",";
    _OutRecord_velo << "tossVarReleaseVelLinX" << "," << "tossVarReleaseVelLinY" << "," << "tossVarReleaseVelLinZ"
                    << ",";
    _OutRecord_velo << "tossVarReleaseVelAngX" << "," << "tossVarReleaseVelAngY" << "," << "tossVarReleaseVelAngZ"
                    << ",";
    _OutRecord_velo << "targetVelLinX" << "," << "targetVelLinY" << "," << "targetVelLinZ" << std::endl;

    // EFFORTS
    _OutRecord_efforts << "timestamp"
                       << ",";
    _OutRecord_efforts << "robotFilteredWrenchXL" << "," << "robotFilteredWrenchYL" << "," << "robotFilteredWrenchZL"
                       << "," << "robotFilteredWrenchRXL" << "," << "robotFilteredWrenchRYL" << ","
                       << "robotFilteredWrenchRZL" << ",";
    _OutRecord_efforts << "robotFilteredWrenchXR" << "," << "robotFilteredWrenchYR" << "," << "robotFilteredWrenchZR"
                       << "," << "robotFilteredWrenchRXR" << "," << "robotFilteredWrenchRYR" << ","
                       << "robotFilteredWrenchRZR" << ",";
    _OutRecord_efforts << "robotForceAppliedXL" << "," << "robotForceAppliedYL" << "," << "robotForceAppliedZL"
                       << "," << "robotForceAppliedRXL" << "," << "robotForceAppliedRYL" << ","
                       << "robotForceAppliedRZL" << ",";
    _OutRecord_efforts << "robotForceAppliedXR" << "," << "robotForceAppliedYR" << "," << "robotForceAppliedZR"
                       << "," << "robotForceAppliedRXR" << "," << "robotForceAppliedRYR" << ","
                       << "robotForceAppliedRZR" << std::endl;
    // Tasks
    _OutRecord_tasks << "timestamp"
                     << ",";
    _OutRecord_tasks << "desVelImpact"
                     << ",";
    _OutRecord_tasks << "desVelTask"
                     << ",";
    _OutRecord_tasks << "goHomeFlag"
                     << ",";
    _OutRecord_tasks << "goToAttractorsFlag"
                     << ",";
    _OutRecord_tasks << "releaseAndRetractFlag"
                     << ",";
    _OutRecord_tasks << "isThrowingFlag"
                     << ",";
    _OutRecord_tasks << "isPlacingFlag"
                     << ",";
    _OutRecord_tasks << "sensedContactFlag"
                     << ",";
    _OutRecord_tasks << "freeMotionAttractorProx"
                     << ",";
    _OutRecord_tasks << "freeMotionAttractorNormal"
                     << ",";
    _OutRecord_tasks << "freeMotionAttractorTangent"
                     << ",";
    _OutRecord_tasks << "freeMotionAttractorRelease"
                     << ",";
    _OutRecord_tasks << "freeMotionAttractorRetract"
                     << ",";
    _OutRecord_tasks << "dsThrowingAttractorProx"
                     << ",";
    _OutRecord_tasks << "dsThrowingAttractorNormal"
                     << ",";
    _OutRecord_tasks << "dsThrowingAttractorTangent"
                     << ",";
    _OutRecord_tasks << "dsThrowingAttractorToss"
                     << ",";
    _OutRecord_tasks << "betaVelMod"
                     << ",";
    _OutRecord_tasks << "dualPathlengthAvgSpeedX"
                     << ",";
    _OutRecord_tasks << "dualPathlengthAvgSpeedY"
                     << ",";
    _OutRecord_tasks << "isPregrabbingFlag"
                     << ",";
    _OutRecord_tasks << "isObjectContactTiltFlag"
                     << ",";
    _OutRecord_tasks << "isObjectTopAttractorFlag"
                     << ",";
    _OutRecord_tasks << "isObjectBackwardTiltingFlag"
                     << ",";
    _OutRecord_tasks << "isObjectReleasingAndCatchingFlag"
                     << ",";
    _OutRecord_tasks << "dualPreGrabFlag" << std::endl;

    // Joint States
    _OutRecord_jts_states << "timestamp"
                          << ",";

    _OutRecord_jts_states << "jointStatePos1Left" << "," << "jointStatePos2Left" << "," << "jointStatePos3Left"
                          << "," << "jointStatePos4Left" << "," << "jointStatePos5Left" << ","
                          << "jointStatePos6Left" << "," << "jointStatePos7Left" << ",";
    _OutRecord_jts_states << "jointStatePos1Right" << "," << "jointStatePos2Right" << "," << "jointStatePos3Right"
                          << "," << "jointStatePos4Right" << "," << "jointStatePos5Right" << ","
                          << "jointStatePos6Right" << "," << "jointStatePos7Right" << ",";

    _OutRecord_jts_states << "jointStateVel1Left" << "," << "jointStateVel2Left" << "," << "jointStateVel3Left"
                          << "," << "jointStateVel4Left" << "," << "jointStateVel5Left" << ","
                          << "jointStateVel6Left" << "," << "jointStateVel7Left" << ",";
    _OutRecord_jts_states << "jointStateVel1Right" << "," << "jointStateVel2Right" << "," << "jointStateVel3Right"
                          << "," << "jointStateVel4Right" << "," << "jointStateVel5Right" << ","
                          << "jointStateVel6Right" << "," << "jointStateVel7Right" << ",";

    _OutRecord_jts_states << "jointStateAcc1Left" << "," << "jointStateAcc2Left" << "," << "jointStateAcc3Left"
                          << "," << "jointStateAcc4Left" << "," << "jointStateAcc5Left" << ","
                          << "jointStateAcc6Left" << "," << "jointStateAcc7Left" << ",";
    _OutRecord_jts_states << "jointStateAcc1Right" << "," << "jointStateAcc2Right" << "," << "jointStateAcc3Right"
                          << "," << "jointStateAcc4Right" << "," << "jointStateAcc5Right" << ","
                          << "jointStateAcc6Right" << "," << "jointStateAcc7Right" << ",";

    _OutRecord_jts_states << "jointStateTorque1Left" << "," << "jointStateTorque2Left" << "," << "jointStateTorque3Left"
                          << "," << "jointStateTorque4Left" << "," << "jointStateTorque5Left" << ","
                          << "jointStateTorque6Left" << "," << "jointStateTorque7Left" << ",";
    _OutRecord_jts_states << "jointStateTorque1Right" << "," << "jointStateTorque2Right" << ","
                          << "jointStateTorque3Right"
                          << "," << "jointStateTorque4Right" << "," << "jointStateTorque5Right" << ","
                          << "jointStateTorque6Right" << "," << "jointStateTorque7Right" << ",";

    _OutRecord_jts_states << "powerLeft" << "," << "powerRight" << std::endl;
  }

  bool Close_files() {
    _OutRecord_pose.close();
    _OutRecord_velo.close();
    _OutRecord_efforts.close();
    _OutRecord_tasks.close();
    _OutRecord_jts_states.close();
    return true;
  }

  // function to log data from file
  bool LoadDataFromFile(std::string file_name, Eigen::VectorXf& data_all_val) {
    //
    ifstream inFile;
    inFile.open(file_name);
    if (!inFile) {
      cout << "Unable to open file \n";
      exit(1);// terminate with error
    }
    //
    std::vector<float> data_val;
    float x;
    //
    while (inFile >> x) { data_val.push_back(x); }
    //
    int size_data_val = data_val.size();
    //
    data_all_val.resize(size_data_val);
    for (int i = 0; i < size_data_val; i++) data_all_val(i) = data_val[i];

    return true;
  }

  bool Load_gmm_param(std::string file_name[],
                      int dataDim,
                      int nbStates,
                      Eigen::VectorXf& Priors_,
                      Eigen::MatrixXf& Means_,
                      Eigen::MatrixXf& Covars_) {

    //
    std::string Priors_file_name = file_name[0];// + "_prio.txt";
    std::string Means_file_name = file_name[1]; // + "_mu.txt";
    std::string Covar_file_name = file_name[2]; // + "_sigma.txt";
    //
    Eigen::VectorXf priors_all_val;
    Eigen::VectorXf means_all_val;
    Eigen::VectorXf covars_all_val;
    //
    this->LoadDataFromFile(Priors_file_name, priors_all_val);
    this->LoadDataFromFile(Means_file_name, means_all_val);
    this->LoadDataFromFile(Covar_file_name, covars_all_val);
    //
    // Priors
    Priors_ = priors_all_val;
    // Means
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Means_Mx(means_all_val.data(),
                                                                                               dataDim,
                                                                                               nbStates);
    Means_ = Means_Mx;

    //
    int row_cov = dataDim * nbStates;
    // Covariance
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Covar_Mx(covars_all_val.data(),
                                                                                               row_cov,
                                                                                               dataDim);
    Covars_ = Covar_Mx;

    return true;
  }

  bool Load_gmm_param2(std::string file_name[],
                       Eigen::VectorXf& Priors_,
                       Eigen::MatrixXf& Means_,
                       Eigen::MatrixXf& Covars_) {
    //
    std::string Priors_file_name = file_name[0];// + "_prio.txt";
    std::string Means_file_name = file_name[1]; // + "_mu.txt";
    std::string Covar_file_name = file_name[2]; // + "_sigma.txt";
    //
    Eigen::VectorXf priors_all_val;
    Eigen::VectorXf means_all_val;
    Eigen::VectorXf covars_all_val;
    //
    this->LoadDataFromFile(Priors_file_name, priors_all_val);
    this->LoadDataFromFile(Means_file_name, means_all_val);
    this->LoadDataFromFile(Covar_file_name, covars_all_val);
    //
    // Priors
    Priors_ = priors_all_val;
    //
    int nbStates = priors_all_val.rows();
    int dataDim = int(means_all_val.rows() / nbStates);
    // Means
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Means_Mx(means_all_val.data(),
                                                                                               dataDim,
                                                                                               nbStates);
    Means_ = Means_Mx;

    //
    int row_cov = dataDim * nbStates;
    // Covariance
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Covar_Mx(covars_all_val.data(),
                                                                                               row_cov,
                                                                                               dataDim);
    Covars_ = Covar_Mx;

    return true;
  }
};
#endif// _DATA_LOGGING_H_