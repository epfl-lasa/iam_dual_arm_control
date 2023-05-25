# iam_dual_arm_control
This repository contains codes to generate coordinated motion and forces to control a robotic dual arm system (here two iiwa robots)


- Dual-arm Grabbing with impact and tossing an object 
<p align="center">
   <img src="https://github.com/epfl-lasa/iam_dual_arm_control/blob/main/media/GrabbingAndTossingSmallBox.gif"  width="350"></>
   <img src="https://github.com/epfl-lasa/iam_dual_arm_control/blob/main/media/GrabbingAndTossingBigObject_00.gif"  width="350"></>
</p> 

- Dual-arm tossing: workspace extension 
<p align="center">
   <img src="https://github.com/epfl-lasa/iam_dual_arm_control/blob/main/media/Expansion_of_workspace_side_00.gif"  width="350"></>
   <img src="https://github.com/epfl-lasa/iam_dual_arm_control/blob/main/media/Expansion_of_workspace_front_00.gif"  width="350"></>
</p> 

- DS-based Dual-arm Adaptation and Robustness
<p align="center">
   <img src="https://github.com/epfl-lasa/iam_dual_arm_control/blob/main/media/DualArmDSAdaptationAndRobustness_00.gif"  width="700"></>
</p>


**Reference**   
[1] M. Bombile, A. Billard, Dual-arm control for coordinated fast grabbing and tossing of an object: Proposing a new approach, IEEE Robotics Automation Magazine 29 (3) (2022) 127–138. doi:10.1109/ MRA.2022.3177355. [pdf-link](https://infoscience.epfl.ch/record/294383/files/Michael_Bombile_Dual-arm_fast_grabbing_and_tossing_of_an_object_RAM_final.pdf?ln=fr) 

**Contact**: For questions on the methods and/or implementation contact [Michael Bombile](https://people.epfl.ch/michael.bombile?lang=en) (michael.bombile AT epfl dot ch)

**Acknowledgments**
This work was supported in part by the European Union’s Horizon 2020 research and innovation program under grant agreement No. 871899. [I.AM. project](https://i-am-project.eu/).



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
 - **sg_differentiation**: A package implementing Savitzky-Golay smoothing and differentiation. (https://github.com/epfl-lasa/sg_differentiation)
 - **iiwa_sim_models_poses**: A ROS-package that get the poses of the robots and objects in Gazebo and publish them as `ros topics`. This package can be found at https://github.com/epfl-lasa/iiwa_sim_models_poses.
 - **sim_objects_description**: A ROS-package that contains URDF and SDF models of some objects (FT_sensors, Robot base, table, conveyor belt) used in the simulation environment. This package can be found at https://github.com/epfl-lasa/sim_objects_description.
 - **dual_iiwa_toolkit**: A ROS-package to control dual KUKA IIWA7 and IIW14 robots in **simulation** (https://github.com/epfl-lasa/dual_iiwa_toolkit)
 - **iiwa_toolkit_ns**: A ROS-package to control KUKA IIWA7 and IIW14 robots in  (https://github.com/epfl-lasa/dual_iiwa_toolkit)

 Optional QP-based torque controller
 - **dual_gen_torque_controller**: A QP-based ROS-package to control KUKA IIWA7 and IIW14 robots in  (https://github.com/MichaelBombile/dual_gen_torque_controller)


# File hierarchy

The file system is divided in several subfolders:
 - `cfg`: contains _.cfg_ files used by dynamic reconfigure
 - `config`: contains _.yaml_ used by launch files
 - `Data`: contains recorded data
 - `include`: contains class header files
 - `launch`: contains _.launch_ files
 - `LearnedModel`: contains parameters of learned inverse throwing model
 - `media`: contains _.gif_ files showing real robots experiments 
 - `src`: contains class implentations and source files to instantiate them:
    - dualArmFreeMotionControl: A class used to generate the uncontrained coordinated motion of the dual arm robot
    - dualArmFreeMotionControl: A class used to generate the grasp and manipulation wrench of the dual arm robot
    - dual_arm_control: A class that uses the two previous classes and generates both the coordinated motion and forces commands to be sent to the robot through ros topics

# Running the controller in simulation

Open a new terminal and launch the simulated robots by running the following commands:
```sh
roslaunch dual_arm_control main_sim_pt3.launch
```
Read and publish the poses of the simulated robots and object by running the command below:
```sh
roslaunch sim_models_poses sim_models_poses2.launch
```
When sim_models_poses is running, you can now start the dual arm controller:
```sh
roslaunch dual_arm_control dual_arm_control.launch
```


# Running the controller on real robots

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

   Option 1: CustomControllers (no longer working)
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
   Option 2: iiwa_toolkit_ns torqueController
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

