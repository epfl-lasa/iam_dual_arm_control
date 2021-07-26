#pragma once

#ifndef _DATA_LOGGING_H_
#define _DATA_LOGGING_H_

#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <chrono>
#include <ctime>

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
}; 
#endif // _DATA_LOGGING_H_