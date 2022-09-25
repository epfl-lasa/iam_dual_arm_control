# iam_dual_arm_control
This repository contains codes to generate coordinated motion and forces to control a dual arm robotic system (here a pair of two KUKA IIWA robots a  IIWA7 and IIWA14). It is used for grabbing and tossing objects. 
The dual arm DS-based controller is currently being used with two type of passibity-based torque controllers:
   1. The iiwa_ros CustomControllers    : inputs are desired linear velocity and axis/angle of the EE
   2. The iiwa_toolkit_ns TorqueController : inputs are desired linear and angular velocity of EE

# Installation

First, make sure that your ssh key are correctly set up. This would be needed for installing the dependencies

Go in the `src` directory of your catkin workspace and clone this package:

```sh
git clone  https://github.com/epfl-lasa/iam_dual_arm_control.git
```
# Dependencies
The main dependencies are the following ones:

 - **ROS**: Robot operating system (Melodic distribution)
 - **CMake**: Build system
 - **Eigen**: A library for linear algebra
 - **iiwa_ros**: A ROS-package to control the KUKA IIWA 7 and IIWA 14 (https://github.com/epfl-lasa/iiwa_ros/tree/2kukas_with_force_sensors)
 - **iiwa_toolkit_ns**: A package to control the robots KUKA IIWA7 and IIWA14 using the passive troque controller (for real robot) https://github.com/epfl-lasa/iiwa_toolkit_ns/tree/feat_real 
 - **dual_iiwa_toolkit**: A package to control the robots KUKA IIWA7 and IIWA14 using the passive troque controller (for simulation) https://github.com/epfl-lasa/dual_iiwa_toolkit
 - **iiwa_sim_models_poses**: A ROS-package that get the poses of the robots and the object in Gazebo and publish them as `ros topics`. This package can be found at https://github.com/epfl-lasa/iiwa_sim_models_poses.
 - **dual_iiwa_scene_description**: A package that contains objects that constitute the scene of the dual arm simulation environment. https://github.com/MichaelBombile/dual_iiwa_scene_description
 - **sg_differentiation**: A package implementing Savitzky-Golay smoothing and differentiation. https://github.com/epfl-lasa/sg_differentiation

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

# Running the controller in simulation

## Step 1: Spawing the robots and the launching the torque controller

Open a new terminal and launch the simulated robots by running the following commands:

Depending on the simulation scene (environment). there are different options:
   - **Basic simulation environment** 
      - dual-arm robots fixed on the floor using custom torque constroller
      - object to grab 
      - an open box on the floor 

   - **Simulation environment with robot bases and custom torque controller**
      - dual-arm robots fixed on the floor
      - object to grab 
      - an open box on the floor 
      - support tables and bases for the robots

   - **Simulation environment with robot bases and passive torque controller**
      - dual-arm robots fixed on the floor
      - object to grab 
      - an open box on the floor 
      - support tables and bases for the robots


Option 1: Basic simulation environment

```sh
roslaunch roslaunch dual_arm_control main_sim_pt1.launch
```
or

```sh
roslaunch roslaunch iiwa_gazebo iiwa_double_gazebo.launch
```

Option 2: Simulation environment with robot bases and custom torque controller

```sh
roslaunch roslaunch dual_arm_control main_sim_pt2.launch
```
Option 3: Simulation environment with robot bases and passive torque controller

```sh
roslaunch roslaunch dual_arm_control main_sim_pt3.launch
```

## Step 2: Spawing the objects of the scene, read and publish their pose information

Read and publish the poses of the simulated robots and object

Open a new terminal and run the command below:

```sh
roslaunch roslaunch sim_models_poses sim_models_poses.launch
```

## Step 3: Launch the dual-arm DS-based coordination controller

When sim_models_poses is running, you can now start the dual arm controller.

Open a new terminal and run the follow command:

```sh
roslaunch roslaunch dual_arm_control dual_arm_control.launch
```

# Running the controller on Real robots


## For grabbing and tossing objects

### Step 1: Open a terminal 

To run the controller, oprn a new terminal and launch roscore.

```sh
   launch roscore
```

### Step 2: launch the sensors packages (optitrack motion capture and force sensors)

Navigate to and source the appropiate ros workspace and run:

```sh
   roslaunch real_pose hardwares.launch 
```
### Step 3: Launch the torque controllers of the robots

Option 1: CustomControllers

   - Open a new terminal and launch the torque controller for IIWA7 and run:

```sh
   roslaunch real_pose bringup_iiwa7.launch
```
   - Open another terminal and through ssh connect to the PC of KUKA IIWA14 robot and run : 

```sh
   roslaunch real_pose bringup_iiwa14.launch
```
Option 2: iiwa_toolkit torqueController

   - Open a new terminal and launch the torque controller for IIWA7 and run:

```sh
   roslaunch iiwa_toolkit passive_track_real.launch
```
   - Open another terminal and through ssh connect to the PC of KUKA IIWA14 robot and run:

```sh
   roslaunch iiwa_toolkit passive_track_real.launch
```
### Step 4: Launch the dual-arm coordinates and variables transformation package
   
This package transform individual robot's coordinates and varibales to reference dual-arm coordinates system (absolute coordinate of the dual-arm system with orientation of the left robot (IIWA7) base)

   - Open a new terminal and run

```sh
   roslaunch real_pose real_poses.launch
```
### Step 5: Launch the dual_arm_controller package

   - Open a new terminal and run 

```sh
   roslaunch dual_arm_control dual_arm_control.launch
```


## For grabbing and tossing objects onto a moving target

Step 6 : Launch the conveyor belt

   - Open new terminal and run:

```sh
   roslaunch conveyor_belt_ros conveyor_belt_ros.launch
```


# Basic usage

This package uses keyboard commands to interact online with the controller. 
Below are the keyboard commands and their functionalities.

## Dual-arm control modes
-----------------------
- **q**: activate or desactivate the rest pose mode
- **t**: activate the tossing task mode 
- **p**: activate the placing task mode
- **o**: activate the place-tossing task mode (tossing resulting from interrupted placing)
- **l**: activate the lifting mode
- **r**: activate the release and retract task mode
- **g**: activate or desactivate the executqion of the motion

## User control of tasks
----------------------
Tossing speed:

- **v**: decrease the tossing speed by -0.05 m/s
- **b**: increase the tossing speed by +0.05 m/s

Impact speed:

- **y**: decrease the impact speed by -0.05 m/s
- **u**: increase the impact speed by +0.05 m/s

Placing

- **x**: decrease the placing height by -0.01 m
- **n**: increase the placing height by +0.01 m


## Conveyor belt control
---------------------
ctrl_mode_conveyor_belt in yaml : True

- **a**: move the conveyor-belt'speed to the left
- **s**: stop the conveyor-belt
- **d**: move the conveyor-belt'speed to the right
- **h**: decrease the nominal speed of the conveyor-belt by -50 mm/s
- **j**: increase the nominal speed of the conveyor-belt by +50 mm/s

## Interception task
---------------------
- **k**: activate the motion adaptation of the robot
- **m**: decrease the magnitude of the perturbation by -50 mm/s
- **i**: increase the magnitude of the perturbation by +50 mm/s
