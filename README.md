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

**Contact**: For questions on the methods and/or implementation contact [Michael Bombile](https://people.epfl.ch/michael.bombile?lang=en) (michael.bombile@epfl.ch)

**Acknowledgments**
This work was supported in part by the European Union’s Horizon 2020 research and innovation program under grant agreement No. 871899. [I.AM. project](https://i-am-project.eu/).


## Dual arm controller
The dual arm control cpp library and the corresponding python bindings. See the [dual_arm_control](./dual_arm_control) folder for the installations instructions.

## Dual arm control using Robot Operating System (ROS)
Control the dual arm in simulation or with real robots using ROS. See the [ros_dual_arm](./ros_dual_arm) folder for the instructions.

## Dual arm control using Algoryx simulator AGX
Control the dual arm in AGX simulation. See the [agx_dual_arm](./agx_dual_arm) folder for the instructions.
