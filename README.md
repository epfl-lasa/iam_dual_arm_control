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



# Usage of the dual-arm_controller on real robots

## For grabbing and tossing objects

The dual arm DS-based controller is currently being used with three types of torque controllers:
   1. The iiwa_ros CustomControllers    : inputs are desired linear velocity and axis/angle of the EE
   2. The iiwa_toolkit_ns TorqueController : inputs are desired linear and angular velocity of EE
   3. The dual_opspace_ds_controller: inputs are desired linear and angular velocity of EE and the EE wrenches

Step 1: Open a terminal 

```sh
   launch roscore
```

Step 2: launch the sensors packages
      forces sensors
      optitrack vrpn client
   Navigate to and source the appropiate workspace (dual_iiwa_real_ws) and run:

```sh
   roslaunch real_pose hardwares.launch 
```
Step 3: Launch the torque controllers of the robots

   Option 1: CustomControllers
   ----------------------------

   - Open a new terminal and launch the torque controller for IIWA7 and run:

```sh
   roslaunch real_pose bringup_iiwa7.launch
```
   - Open another terminal and through ssh connect to the PC of KUKA IIWA14 robot
      (ssh -X USERID@ip_of_iiwa14_pc) and run:

```sh
   roslaunch real_pose bringup_iiwa14.launch
```
   Option 2: iiwa_toolkit torqueController
   ----------------------------------------
   - Open a new terminal and launch the torque controller for IIWA7 and run:

```sh
   roslaunch iiwa_toolkit passive_track_real.launch
```
   - Open another terminal and through ssh connect to the PC of KUKA IIWA14 robot and run:
      ssh -X USERID@ip_of_iiwa14_pc

```sh
   roslaunch iiwa_toolkit passive_track_real.launch
```
   Option 3: dual_gen_trq_controller dual_iiwa_opspace_ds_controller
   ----------------------------------------
   - Open a new terminal and launch the torque controller for IIWA7 (iiwa_bringup.launch):

```sh
   roslaunch iiwa_bringup.launch
```

   - Open another terminal and through ssh connect to the PC of KUKA IIWA14 robot by running:
      ssh -X USERID@ip_of_iiwa14_pc

```sh
   roslaunch iiwa_bringup.launch
```
- Open another terminal and through ssh connect to the PC of KUKA IIWA14 robot and run:
      ssh -X USERID@ip_of_iiwa14_pc

```sh
   roslaunch roslaunch dual_gen_trq_controller dual_iiwa_opspace_ds_controller_real.launch
```

Step 4: Launch the dual-arm coordinates and variables transformation package
      This package transform individual robot's coordinates and varibales to reference dual-arm coordinates system
      (absolute coordinate of the dual-arm system with orientation of the left robot (IIWA7) base)

   - Open a new terminal and run

```sh
   roslaunch real_pose real_poses.launch
```
Step 5: Launch the dual_arm_controller package

   - Open a new terminal and run 

```sh
   roslaunch dual_arm_control dual_arm_control.launch
```

## For grabbing and tossing objects onto a moving target

Step 6 : Launch the conveyor belt

   - Open new terminal and through ssh connect to the PC of KUKA IIWA14 robot
      (ssh -X USERID@ip_of_iiwa14_pc) and run:

```sh
   roslaunch conveyor_belt_ros conveyor_belt_ros.launch
```

