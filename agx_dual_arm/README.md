# Dual arm control using Algoryx simulator AGX
Control the dual arm in AGX simulation


# Dependencies
The main dependencies are the following ones:

 - **AGX simulator**: Working AGX urdf-application environnement is required 
 - **AGX custom project scene for IIWA**: TODO SOMEWHRE ELSE (https://github.com/Elise-J/iam_sim_agx.git)
 - **dual_iiwa_toolkit**: The python binding of dual_iiwa_toolkit (https://github.com/epfl-lasa/dual_iiwa_toolkit)


# File hierarchy

The file system is divided in several subfolders:
 - `config`: contains _.yaml_  and _ LearnedModel_ used by the controller
 - `urdf`: contains _.urdf_ of robot used by the controller
 - `script`: 
    - python_agx_dual_arm: Contains the control loop


# Setup environment

Tested with python 3.8.10. It is recommanded to create a venv
1. ` cd agx_dual_arm && pip install -r requirements_agx.txt`
2. Optional: for the keyboard control: `sudo pip3 install keyboard`


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
