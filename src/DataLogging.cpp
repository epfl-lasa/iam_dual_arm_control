#include "dual_arm_control_iam/DataLogging.hpp"

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
    ROS_ERROR("[outRecordPose]: Cannot open output data files, the Data directory might be missing");
    return false;
  }
  if (!outRecordVel.is_open()) {
    ROS_ERROR("[outRecordVel]: Cannot open output data files, the Data directory might be missing");
    return false;
  }
  if (!outRecordEfforts.is_open()) {
    ROS_ERROR("[outRecordEfforts]: Cannot open output data files, the Data directory might be missing");
    return false;
  }
  if (!outRecordTasks.is_open()) {
    ROS_ERROR("[outRecordTasks]: Cannot open output data files, the Data directory might be missing");
    return false;
  }
  if (!outRecordJointStates.is_open()) {
    ROS_ERROR("[outRecordJointStates]: Cannot open output data files, the Data directory might be missing");
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
    ROS_ERROR("[outRecordPose]: Cannot open output data files, the Data directory might be missing");
    return false;
  }
  if (!outRecordVel.is_open()) {
    ROS_ERROR("[outRecordVel]: Cannot open output data files, the Data directory might be missing");
    return false;
  }
  if (!outRecordEfforts.is_open()) {
    ROS_ERROR("[outRecordEfforts]: Cannot open output data files, the Data directory might be missing");
    return false;
  }
  if (!outRecordTasks.is_open()) {
    ROS_ERROR("[outRecordTasks]: Cannot open output data files, the Data directory might be missing");
    return false;
  }
  if (!outRecordJointStates.is_open()) {
    ROS_ERROR("[outRecordJointStates]: Cannot open output data files, the Data directory might be missing");
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

// Function to log data from file
bool DataLogging::loadDataFromFile(std::string fileName, Eigen::VectorXf& dataAllVal) {

  ifstream inFile;
  inFile.open(fileName);
  if (!inFile) {
    cout << "Unable to open file \n";
    exit(1);// terminate with error
  }

  std::vector<float> dataVal;
  float x;
  while (inFile >> x) { dataVal.push_back(x); }

  int sizeDataVal = dataVal.size();
  dataAllVal.resize(sizeDataVal);
  for (int i = 0; i < sizeDataVal; i++) dataAllVal(i) = dataVal[i];

  return true;
}

bool DataLogging::loadGMMParam(std::string fileName[],
                               Eigen::VectorXf& priors,
                               Eigen::MatrixXf& means,
                               Eigen::MatrixXf& covars) {

  std::string priorsFileName = fileName[0];
  std::string meansFileName = fileName[1];
  std::string covarFileName = fileName[2];
  Eigen::VectorXf priorsAllVal;
  Eigen::VectorXf meansAllVal;
  Eigen::VectorXf covarsAllVal;

  this->loadDataFromFile(priorsFileName, priorsAllVal);
  this->loadDataFromFile(meansFileName, meansAllVal);
  this->loadDataFromFile(covarFileName, covarsAllVal);

  // Priors
  priors = priorsAllVal;

  int nbStates = priorsAllVal.rows();
  int dataDim = int(meansAllVal.rows() / nbStates);

  // Means
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> meansMx(meansAllVal.data(),
                                                                                            dataDim,
                                                                                            nbStates);
  means = meansMx;

  int rowCov = dataDim * nbStates;

  // Covariance
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> covarMx(covarsAllVal.data(),
                                                                                            rowCov,
                                                                                            dataDim);
  covars = covarMx;

  return true;
}
