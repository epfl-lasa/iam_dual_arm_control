#include "iam_dual_arm_control/DataLogging.hpp"

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
bool DataLogging::loadDataFromFile(std::string file_name, Eigen::VectorXf& data_all_val) {

  ifstream inFile;
  inFile.open(file_name);
  if (!inFile) {
    cout << "Unable to open file \n";
    exit(1);// terminate with error
  }

  std::vector<float> data_val;
  float x;
  while (inFile >> x) { data_val.push_back(x); }

  int size_data_val = data_val.size();
  data_all_val.resize(size_data_val);
  for (int i = 0; i < size_data_val; i++) data_all_val(i) = data_val[i];

  return true;
}

bool DataLogging::loadGMMParam(std::string file_name[],
                               Eigen::VectorXf& Priors_,
                               Eigen::MatrixXf& Means_,
                               Eigen::MatrixXf& Covars_) {

  std::string Priors_file_name = file_name[0];// + "_prio.txt";
  std::string Means_file_name = file_name[1]; // + "_mu.txt";
  std::string Covar_file_name = file_name[2]; // + "_sigma.txt";

  Eigen::VectorXf priors_all_val;
  Eigen::VectorXf means_all_val;
  Eigen::VectorXf covars_all_val;

  this->loadDataFromFile(Priors_file_name, priors_all_val);
  this->loadDataFromFile(Means_file_name, means_all_val);
  this->loadDataFromFile(Covar_file_name, covars_all_val);

  // Priors
  Priors_ = priors_all_val;

  int nbStates = priors_all_val.rows();
  int dataDim = int(means_all_val.rows() / nbStates);

  // Means
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Means_Mx(means_all_val.data(),
                                                                                             dataDim,
                                                                                             nbStates);
  Means_ = Means_Mx;

  int row_cov = dataDim * nbStates;

  // Covariance
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> Covar_Mx(covars_all_val.data(),
                                                                                             row_cov,
                                                                                             dataDim);
  Covars_ = Covar_Mx;

  return true;
}
