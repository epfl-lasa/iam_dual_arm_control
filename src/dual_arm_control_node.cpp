
#include "ros/ros.h"
#include "dual_arm_control.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "dual_arm_control_node");

	ros::NodeHandle nh;
	double frequency = 200.0f;

	// Parameters
	std::string topic_pose_object			= "/simo_track/object_pose";
	std::string topic_pose_robot_base_left	= "/simo_track/robot_left/pose";
	std::string topic_pose_robot_ee_left	= "/simo_track/robot_left/ee_pose";
	std::string topic_ee_commands_left		= "/iiwa1/CustomControllers/command";
	std::string topic_pose_robot_base_right	= "/simo_track/robot_right/pose";
	std::string topic_pose_robot_ee_right	= "/simo_track/robot_right/ee_pose";
	std::string topic_ee_commands_right		= "/iiwa_blue/CustomControllers/command";


	// LOADING PARAMETERS FROM THE ROS SERVER
	// Topic names
	// -------------
	// if (!nh.getParam("topic_desired_DS_CoM_velocity", topic_pose_object)) {
	// 	ROS_ERROR("Couldn't retrieve the topic name for object pose.");
	// 	return -1;
	// }

	// // -------------
	// if (!nh.getParam("topic_desired_DS_CoM_attractor", topic_pose_robot_left)) {
	// 	ROS_ERROR("Couldn't retrieve the topic name for the left robot pose.");
	// 	return -1;
	// }

	// creating the streamer
	dual_arm_control dualArmCtrl(nh, frequency, topic_pose_object, 	
												topic_pose_robot_base_left,
												topic_pose_robot_ee_left,
												topic_ee_commands_left,
												topic_pose_robot_base_right,
												topic_pose_robot_ee_right,
												topic_ee_commands_right);

	if (!dualArmCtrl.init()) 
	{
		return -1;
	}
	else
	{
		dualArmCtrl.run();
	}

	return 0;
}