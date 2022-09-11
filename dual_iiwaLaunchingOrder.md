
# Usage of the dual-arm_controller on real robots

## For grabbing and tossing objects

The dual arm DS-based controller is currently being used with two type of passibity-based torque controllers:
	1. The iiwa_ros CustomControllers    : inputs are desired linear velocity and axis/angle of the EE
	2. The iiwa_toolkit_ns TorqueController : inputs are desired linear and angular velocity of EE

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
		(ssh -X michael@128.178.145.62) and run:

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
		ssh -X michael@128.178.145.62

```sh
	roslaunch iiwa_toolkit passive_track_real.launch
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
		(ssh -X michael@128.178.145.62) and run:

```sh
	roslaunch conveyor_belt_ros conveyor_belt_ros.launch
```

