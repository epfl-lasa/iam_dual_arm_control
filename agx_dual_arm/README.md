# Dual arm control using Algoryx simulator AGX
Control the dual arm in AGX simulation


# Dependencies
The main dependencies are the following ones:

 - **AGX simulator**: Working AGX urdf-application environnement is required 
 - **AGX custom project scene for IIWA**: Custom AGX scene (https://github.com/Elise-J/iam_sim_agx.git)
 - **dual_iiwa_toolkit**: Optional, if the build of the passive controller is needed: The python binding of dual_iiwa_toolkit (https://github.com/epfl-lasa/dual_iiwa_toolkit)


# File hierarchy

The file system is divided in several subfolders:
 - `builded_libs`: contains the cpp library and the python binding of the passive controller
 - `config`: contains _.yaml_  and _ LearnedModel_ used by the controller
 - `urdf`: contains _.urdf_ of robot used by the controller
 - `script`: 
    - python_agx_dual_arm: Contains the control loop


# Setup environment

Tested with python 3.8.10. It is recommanded to create a venv
1. ` cd agx_dual_arm && pip install -r requirements_agx.txt`
2. Optional: for the keyboard control: `sudo pip3 install keyboard`
3. Add the cpp lib and python bindings to you path
```bash
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path_to_iam_dual_arm_control/dual_arm_control/python_binding/build/' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/path_to_iam_dual_arm_control/agx_dual_arm/builded_libs' >> ~/.bashrc 

. ~/.bashrc

sudo ldconfig
```

If the libraries of i_am_project or iiwa_toolkit need to be build:
```bash
cd python_binding && mkdir build && cd build && cmake .. && make -j
```

# Run simulation

1. Launch AGX simulator with the custom project scene for the double iiwa
2. Run the controller:

```
 python script/python_agx_dual_arm.py
```

With the keyboard commands (sudo rights required)

```
 sudo venv/bin/python3.8 script/python_agx_dual_arm.py
```
