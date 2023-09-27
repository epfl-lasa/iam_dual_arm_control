#include "DataLogging.hpp"

bool DataLogging::init(std::string path2Datafolder) {
  auto now = std::chrono::system_clock::now();
  auto inTimeT = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&inTimeT), "%Y-%m-%d-%X");
  std::string dataID = ss.str();

  outRecordPose.open(path2Datafolder + "/log_task_pose_" + dataID + ".csv");
  outRecordVel.open(path2Datafolder + "/log_task_velo_" + dataID + ".csv");
  outRecordEfforts.open(path2Datafolder + "/log_robot_efforts_" + dataID + ".csv");
  outRecordTasks.open(path2Datafolder + "/log_robot_tasks_" + dataID + ".csv");
  outRecordJointStates.open(path2Datafolder + "/log_joints_states_" + dataID + ".csv");

  if (!outRecordPose.is_open()) {
    std::cerr << "[outRecordPose]: Cannot open output data files, the Data directory might be missing" << std::endl;
    return false;
  }
  if (!outRecordVel.is_open()) {
    std::cerr << "[outRecordVel]: Cannot open output data files, the Data directory might be missing" << std::endl;
    return false;
  }
  if (!outRecordEfforts.is_open()) {
    std::cerr << "[outRecordEfforts]: Cannot open output data files, the Data directory might be missing" << std::endl;
    return false;
  }
  if (!outRecordTasks.is_open()) {
    std::cerr << "[outRecordTasks]: Cannot open output data files, the Data directory might be missing" << std::endl;
    return false;
  }
  if (!outRecordJointStates.is_open()) {
    std::cerr << "[outRecordJointStates]: Cannot open output data files, the Data directory might be missing"
              << std::endl;
    return false;
  }
}

bool DataLogging::reset(std::string path2Datafolder) {

  this->closeFiles();

  auto now = std::chrono::system_clock::now();
  auto inTimeT = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&inTimeT), "%Y-%m-%d-%X");
  std::string dataID = ss.str();

  outRecordPose.open(path2Datafolder + "/log_task_pose_" + dataID + ".csv");
  outRecordVel.open(path2Datafolder + "/log_task_velo_" + dataID + ".csv");
  outRecordEfforts.open(path2Datafolder + "/log_robot_efforts_" + dataID + ".csv");
  outRecordTasks.open(path2Datafolder + "/log_robot_tasks_" + dataID + ".csv");
  outRecordJointStates.open(path2Datafolder + "/log_joints_states_" + dataID + ".csv");

  if (!outRecordPose.is_open()) {
    std::cerr << "[outRecordPose]: Cannot open output data files, the Data directory might be missing" << std::endl;
    return false;
  }
  if (!outRecordVel.is_open()) {
    std::cerr << "[outRecordVel]: Cannot open output data files, the Data directory might be missing" << std::endl;
    return false;
  }
  if (!outRecordEfforts.is_open()) {
    std::cerr << "[outRecordEfforts]: Cannot open output data files, the Data directory might be missing" << std::endl;
    return false;
  }
  if (!outRecordTasks.is_open()) {
    std::cerr << "[outRecordTasks]: Cannot open output data files, the Data directory might be missing" << std::endl;
    return false;
  }
  if (!outRecordJointStates.is_open()) {
    std::cerr << "[outRecordJointStates]: Cannot open output data files, the Data directory might be missing"
              << std::endl;
    return false;
  }
}

bool DataLogging::closeFiles() {
  outRecordPose.close();
  outRecordVel.close();
  outRecordEfforts.close();
  outRecordTasks.close();
  outRecordJointStates.close();
  return true;
}
