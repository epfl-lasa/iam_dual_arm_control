
#include "iam_dual_arm_control/dual_arm_control.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dual_arm_control_node");
  ros::NodeHandle nh;
  double frequency = 200.0f;
  bool isSimulation = false;

  std::string topicPoseObject;
  std::string topicPoseRobotBase[NB_ROBOTS];
  std::string topicPoseRobotEE[NB_ROBOTS];
  std::string topicEECommands[NB_ROBOTS];
  std::string topicSubForceTorqueSensor[NB_ROBOTS];

  // Get parameters
  if (!nh.getParam(nh.getNamespace() + "/dual_system/simulation", isSimulation)) {
    ROS_ERROR("Topic /dual_system/simulation not found");
  }

  // Get ROS topics
  if (!nh.getParam(nh.getNamespace() + "/pose/object", topicPoseObject)) {
    ROS_ERROR("Topic /passive_control not found");
  }
  if (!nh.getParam(nh.getNamespace() + "/pose/robot_base/robot_left", topicPoseRobotBase[0])) {
    ROS_ERROR("Topic pose/robot_base/robot_left not found");
  }
  if (!nh.getParam(nh.getNamespace() + "/pose/robot_base/robot_right", topicPoseRobotBase[1])) {
    ROS_ERROR("Topic pose/robot_base/robot_right not found");
  }
  if (!nh.getParam(nh.getNamespace() + "/pose/robot_ee/robot_left", topicPoseRobotEE[0])) {
    ROS_ERROR("Topic pose/robot_ee/robot_left not found");
  }
  if (!nh.getParam(nh.getNamespace() + "/pose/robot_ee/robot_right", topicPoseRobotEE[1])) {
    ROS_ERROR("Topic pose/robot_ee/robot_right not found");
  }
  if (!nh.getParam(nh.getNamespace() + "/commands/robot_ee/robot_left", topicEECommands[0])) {
    ROS_ERROR("Topic pose/robot_ee/robot_left not found");
  }
  if (!nh.getParam(nh.getNamespace() + "/commands/robot_ee/robot_right", topicEECommands[1])) {
    ROS_ERROR("Topic pose/robot_ee/robot_right not found");
  }
  if (isSimulation) {
    if (!nh.getParam(nh.getNamespace() + "/ft_sensors/simulation/sensor_left", topicSubForceTorqueSensor[0])) {
      ROS_ERROR("Topic /ft_sensors/simulation/sensor_left not found");
    }
    if (!nh.getParam(nh.getNamespace() + "/ft_sensors/simulation/sensor_right", topicSubForceTorqueSensor[1])) {
      ROS_ERROR("Topic /ft_sensors/simulation/sensor_right not found");
    }
  } else {
    if (!nh.getParam(nh.getNamespace() + "/ft_sensors/real/sensor_left", topicSubForceTorqueSensor[0])) {
      ROS_ERROR("Topic /ft_sensors/simulation/sensor_left not found");
    }
    if (!nh.getParam(nh.getNamespace() + "/ft_sensors/real/sensor_right", topicSubForceTorqueSensor[1])) {
      ROS_ERROR("Topic /ft_sensors/simulation/sensor_right not found");
    }
  }

  // Creating the streamer
  dual_arm_control dualArmCtrl(nh,
                               frequency,
                               topicPoseObject,
                               topicPoseRobotBase,
                               topicPoseRobotEE,
                               topicEECommands,
                               topicSubForceTorqueSensor);

  if (!dualArmCtrl.init()) {
    return -1;
  } else {
    dualArmCtrl.run();
  }

  return 0;
}