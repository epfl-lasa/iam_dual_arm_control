//|
//|    Copyright (C) 2021-2023 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors: Michael Bombile (maintainer)
//|
//|    email:   michael.bombile@epfl.ch/micbombile@gmail.com
//|
//|    Other contributors:
//|             Elise Jeandupeux (elise.jeandupeux@epfl.ch)
//|
//|    website: lasa.epfl.ch
//|
//|    This file is part of iam_dual_arm_control.
//|    This work was supported by the European Community's Horizon 2020 Research and Innovation
//|    programme (call: H2020-ICT-09-2019-2020, RIA), grant agreement 871899 Impact-Aware Manipulation.
//|
//|    iam_dual_arm_control is free software: you can redistribute it and/or modify  it under the terms
//|    of the GNU General Public License as published by  the Free Software Foundation,
//|    either version 3 of the License, or  (at your option) any later version.
//|
//|    iam_dual_arm_control is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|

#pragma once

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
  bool loadDataFromFile(std::string fileName, Eigen::VectorXf& dataAllVal);

  bool loadGMMParam(std::string fileName[], Eigen::VectorXf& priors, Eigen::MatrixXf& means, Eigen::MatrixXf& covars);
};
