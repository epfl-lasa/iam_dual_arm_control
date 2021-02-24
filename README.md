# iam_dual_arm_control
This repository contains codes to generate coordinated motion and forces to control a robotic dual arm system (here two iiwa robots)

# Installation

First, make sure that your ssh key are correctly set up. This would be needed for installing the dependencies

Go in the `src` directory of your catkin workspace and clone this package:
```sh
git clone  https://github.com/epfl-lasa/iam_dual_arm_control.git
```
# Dependencies
The main dependencies are the following ones:

 - **ROS**: Robot operating system (indigo distribution)
 - **CMake**: Build system
 - **Eigen**: A library for linear algebra
 - **iiwa_ros**: A ROS-package to control the KUKA IIWA 7 and IIWA 14 (https://github.com/epfl-lasa/iiwa_ros/tree/2kukas_with_force_sensors)
 - **iiwa_sim_models_poses**: A ROS-package that get the poses of the robots and the object in Gazebo and publish them as `ros topics`. This package can be found at https://github.com/epfl-lasa/iiwa_sim_models_poses.

# File hierarchy

The file system is divided in several subfolders:
 - `cfg`: contains _.cfg_ files used by dynamic reconfigure
 - `config`: contains _.yaml_ used by launch files
 - `include`: contains class header files
 - `launch`: contains _.launch_ files
 - `src`: contains class implentations and source files to instantiate them:
    - dualArmFreeMotionControl: A class used to generate the uncontrained coordinated motion of the dual arm robot
    - dualArmFreeMotionControl: A class used to generate the grasp and manipulation wrench of the dual arm robot
    - dual_arm_control: A class that uses the two previous classes and generates both the coordinated motion and forces commands to be sent to the robot through ros topics

# running the controller (simulation)

Open a new terminal and launch the simulated robots by running the following commands:
```sh
roslaunch roslaunch iiwa_gazebo iiwa_double_gazebo.launch
```
Read and publish the poses of the simulated robots and object by running the command below:
```sh
roslaunch roslaunch sim_models_poses sim_models_poses.launch
```
When sim_models_poses is running, you can now start the dual arm controller:
```sh
roslaunch roslaunch dual_arm_control dual_arm_control.launch
```