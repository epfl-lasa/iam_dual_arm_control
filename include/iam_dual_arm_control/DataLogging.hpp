#pragma once

#include "ros/ros.h"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>

using namespace std;

class DataLogging {
public:
  // Data logging
  std::ofstream outRecordPose;
  std::ofstream outRecordVel;
  std::ofstream outRecordEfforts;
  std::ofstream outRecordTasks;
  std::ofstream outRecordJointStates;

  DataLogging(){};
  ~DataLogging(){};

  bool init(std::string path2Datafolder);
  bool reset(std::string path2Datafolder);

  bool closeFiles();

  // Function to log data from file
  bool loadDataFromFile(std::string file_name, Eigen::VectorXf& data_all_val);

  bool
  loadGMMParam(std::string file_name[], Eigen::VectorXf& Priors_, Eigen::MatrixXf& Means_, Eigen::MatrixXf& Covars_);
};
