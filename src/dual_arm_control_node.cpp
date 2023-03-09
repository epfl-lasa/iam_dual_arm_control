#include "ros/ros.h"
#include "iam_dual_arm_control/dual_arm_control.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dual_arm_control_node");
	ros::NodeHandle nh;
	double frequency  = 200.0f;
	bool isSimulation = false;
	//
	std::string DataID;
	std::string topic_pose_robot_base[NB_ROBOTS];
	std::string topic_pose_robot_ee[NB_ROBOTS];
	std::string topic_ee_commands[NB_ROBOTS];
	std::string topic_sub_ForceTorque_Sensor[NB_ROBOTS];

	// Parameters
	std::string topic_pose_object	= "/simo_track/object_pose"; 
	topic_pose_robot_base[0]			= "/simo_track/robot_left/pose";
	topic_pose_robot_ee[0]				= "/simo_track/robot_left/ee_pose";
	topic_ee_commands[0]					= "/iiwa1/CustomControllers/command";
	topic_pose_robot_base[1]			= "/simo_track/robot_right/pose";
	topic_pose_robot_ee[1]				= "/simo_track/robot_right/ee_pose";
	topic_ee_commands[1]					= "/iiwa_blue/CustomControllers/command";
	//

	nh.getParam(nh.getNamespace() + "/dual_system/simulation", isSimulation);
	//
	if(isSimulation){
		topic_sub_ForceTorque_Sensor[0]	= "/iiwa1/iiwa1_FTS_topic";
		topic_sub_ForceTorque_Sensor[1]	= "/iiwa_blue/iiwa_blue_FTS_topic";
	}
	else{
		topic_sub_ForceTorque_Sensor[0]	= "/ft_sensor_left/netft_data";
		topic_sub_ForceTorque_Sensor[1]	= "/ft_sensor_right/netft_data";
	}

	// creating the streamer
	dual_arm_control dualArmCtrl(nh, 
															 frequency, //DataID,
															 topic_pose_object, 	
															 topic_pose_robot_base,
															 topic_pose_robot_ee,
															 topic_ee_commands,
															 topic_sub_ForceTorque_Sensor);
	//
	if (!dualArmCtrl.init()) {
		return -1;
	}
	else {
		dualArmCtrl.run();
	}
	//
	return 0;
}