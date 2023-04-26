#pragma once

#ifndef _DATA_LOGGING_H_
#define _DATA_LOGGING_H_

#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>
#include <ctime>
#include "ros/ros.h"

using namespace std;
//
class data_logging
{

	public:

		float SimTime;
		std::string _log_pose;
		std::string _log_velo;
		std::string _log_efforts;
		std::string _log_tasks;
		std::string _log_jts_states;

		// data logging
		std::string   _DataID;
		std::ofstream _OutRecord_pose;
		std::ofstream _OutRecord_velo;
		std::ofstream _OutRecord_efforts;
		std::ofstream _OutRecord_tasks;
		std::ofstream _OutRecord_jts_states;

		data_logging(){};
		~data_logging(){};

		bool datalog_init(std::string path2Datafolder){
			//
			auto now = std::chrono::system_clock::now();
			auto in_time_t = std::chrono::system_clock::to_time_t(now);
			std::stringstream ss;
			ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
			std::string DataID = ss.str();
			_OutRecord_pose.open(path2Datafolder       +"/log_task_pose_"    +DataID+".csv");
			_OutRecord_velo.open(path2Datafolder       +"/log_task_velo_"    +DataID+".csv");
			_OutRecord_efforts.open(path2Datafolder    +"/log_robot_efforts_"+DataID+".csv");
			_OutRecord_tasks.open(path2Datafolder      +"/log_robot_tasks_"  +DataID+".csv");
			_OutRecord_jts_states.open(path2Datafolder +"/log_joints_states_"+DataID+".csv");

		  if(!_OutRecord_pose.is_open()){
		    ROS_ERROR("[_OutRecord_pose]: Cannot open output data files, the Data directory might be missing");
		    return false;
		  }
		  if(!_OutRecord_velo.is_open()){
		    ROS_ERROR("[_OutRecord_velo]: Cannot open output data files, the Data directory might be missing");
		    return false;
		  }
		  if(!_OutRecord_efforts.is_open()){
		    ROS_ERROR("[_OutRecord_efforts]: Cannot open output data files, the Data directory might be missing");
		    return false;
		  }
		  if(!_OutRecord_tasks.is_open()){
		    ROS_ERROR("[_OutRecord_tasks]: Cannot open output data files, the Data directory might be missing");
		    return false;
		  }
		  if(!_OutRecord_jts_states.is_open()){
		    ROS_ERROR("[_OutRecord_jts_states]: Cannot open output data files, the Data directory might be missing");
		    return false;
		  }

		}

		bool datalog_reset(std::string path2Datafolder){
			//
			this->Close_files();

			auto now = std::chrono::system_clock::now();
			auto in_time_t = std::chrono::system_clock::to_time_t(now);
			std::stringstream ss;
			ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d-%X");
			std::string DataID = ss.str();
			_OutRecord_pose.open(path2Datafolder       +"/log_task_pose_"    +DataID+".csv");
			_OutRecord_velo.open(path2Datafolder       +"/log_task_velo_"    +DataID+".csv");
			_OutRecord_efforts.open(path2Datafolder    +"/log_robot_efforts_"+DataID+".csv");
			_OutRecord_tasks.open(path2Datafolder      +"/log_robot_tasks_"  +DataID+".csv");
			_OutRecord_jts_states.open(path2Datafolder +"/log_joints_states_"+DataID+".csv");

		  if(!_OutRecord_pose.is_open()){
		    ROS_ERROR("[_OutRecord_pose]: Cannot open output data files, the Data directory might be missing");
		    return false;
		  }
		  if(!_OutRecord_velo.is_open()){
		    ROS_ERROR("[_OutRecord_velo]: Cannot open output data files, the Data directory might be missing");
		    return false;
		  }
		  if(!_OutRecord_efforts.is_open()){
		    ROS_ERROR("[_OutRecord_efforts]: Cannot open output data files, the Data directory might be missing");
		    return false;
		  }
		  if(!_OutRecord_tasks.is_open()){
		    ROS_ERROR("[_OutRecord_tasks]: Cannot open output data files, the Data directory might be missing");
		    return false;
		  }
		  if(!_OutRecord_jts_states.is_open()){
		    ROS_ERROR("[_OutRecord_jts_states]: Cannot open output data files, the Data directory might be missing");
		    return false;
		  }

		}

		void Write_Data(){};

		bool Close_files(){
			_OutRecord_pose.close();
			_OutRecord_velo.close();
			_OutRecord_efforts.close();
			_OutRecord_tasks.close();
			_OutRecord_jts_states.close();
			return true;
		}

		// function to log data from file
		bool LoadDataFromFile(std::string file_name, Eigen::VectorXf &data_all_val)
		{
		    //
		    ifstream inFile;
		    inFile.open(file_name);
		    if (!inFile) {
		        cout << "Unable to open file \n";
		        exit(1);                            // terminate with error
		    }
		    //
		    std::vector<float> data_val;
		    float x; 
		    //
		    while(inFile >> x) {
		        data_val.push_back(x);
		    }
		    //
		    int size_data_val = data_val.size();
		    //
		    data_all_val.resize(size_data_val);
		    for(int i=0; i<size_data_val; i++) 
		        data_all_val(i) = data_val[i];

		    return true;
		}

		bool Load_gmm_param(std::string file_name[], int dataDim, int nbStates, Eigen::VectorXf &Priors_, Eigen::MatrixXf &Means_, Eigen::MatrixXf &Covars_)
		{

			// 
			std::string Priors_file_name  = file_name[0]; // + "_prio.txt";  
			std::string Means_file_name   = file_name[1]; // + "_mu.txt";  
			std::string Covar_file_name   = file_name[2]; // + "_sigma.txt";  
			//
			Eigen::VectorXf priors_all_val;
			Eigen::VectorXf means_all_val;
			Eigen::VectorXf covars_all_val;
			//
			this->LoadDataFromFile(Priors_file_name, priors_all_val);
			this->LoadDataFromFile(Means_file_name,  means_all_val);
			this->LoadDataFromFile(Covar_file_name,  covars_all_val);
			//
			// Priors
			Priors_ = priors_all_val;
			// Means
			Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Means_Mx(means_all_val.data(),dataDim, nbStates);
			Means_ = Means_Mx;
			
			//
			int row_cov = dataDim * nbStates;
			// Covariance
			Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Covar_Mx(covars_all_val.data(),row_cov,dataDim);
			Covars_ = Covar_Mx;


			return true;
		}


		bool Load_gmm_param2(std::string file_name[], Eigen::VectorXf &Priors_, Eigen::MatrixXf &Means_, Eigen::MatrixXf &Covars_)
		{
			// 
			std::string Priors_file_name  = file_name[0]; // + "_prio.txt";  
			std::string Means_file_name   = file_name[1]; // + "_mu.txt";  
			std::string Covar_file_name   = file_name[2]; // + "_sigma.txt";  
			//
			Eigen::VectorXf priors_all_val;
			Eigen::VectorXf means_all_val;
			Eigen::VectorXf covars_all_val;
			//
			this->LoadDataFromFile(Priors_file_name, priors_all_val);
			this->LoadDataFromFile(Means_file_name,  means_all_val);
			this->LoadDataFromFile(Covar_file_name,  covars_all_val);
			//
			// Priors
			Priors_  = priors_all_val;
			//
			int nbStates = priors_all_val.rows();
			int dataDim  = int(means_all_val.rows()/nbStates);
			// Means
			Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Means_Mx(means_all_val.data(),dataDim, nbStates);
			Means_ = Means_Mx;

			//
			int row_cov = dataDim * nbStates;
			// Covariance
			Eigen::Map< Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > Covar_Mx(covars_all_val.data(),row_cov,dataDim);
			Covars_ = Covar_Mx;


			return true;
		}

}; 
#endif // _DATA_LOGGING_H_