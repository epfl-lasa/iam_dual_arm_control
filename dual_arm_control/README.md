
# Dual arm controller
The dual arm control cpp library and the corresponding python bindings

# Dependencies
The main dependencies are the following ones:

 - **CMake**: Build system
 - **Eigen**: A library for linear algebra, version 3. (https://eigen.tuxfamily.org)
 - **sg_differentiation**: A package implementing Savitzky-Golay smoothing and differentiation. (https://github.com/epfl-lasa/sg_differentiation , branch cmake)


# File hierarchy

The file system is divided in several subfolders:
 - `include`: contains class header files
 - `LearnedModel`: contains parameters of learned inverse throwing model
 - `src`: contains class implentations and source files to instantiate them:
    - DualArmFreeMotionController: A class used to generate the uncontrained coordinated motion of the dual arm robot
    - DualArmCooperativeControllwe: A class used to generate the grasp and manipulation wrench of the dual arm robot
    - DualArmControl: A class that uses the two previous classes and generates both the coordinated motion and forces commands to be sent to the robot


# Installation

To install the cpp library:

```sh
mkdir build && cd build && cmake .. && sudo make install
```

To build the python binding, the cpp library needs to be build first: 

```sh
cd python_binding/build && cmake .. && make -j
```
