This repo needs to merge with https://github.com/epfl-lasa/iam_dual_arm_control

## AGX simulation:

Having a working AGX urdf-application environnement is required 

#### Get custom Project scene for IIWA
1. `cd urdf-application/PythonApplication/models/Projects`
2. `git clone https://github.com/Elise-J/iam_sim_agx.git`

#### Setup environment
Tested with python 3.8.10. It is recommanded to create a venv
1. ` cd agx_dual_arm && pip install -r requirements_agx.txt`
2. sudo pip3 install keyboard

#### Run simulation
```
 sudo venv/bin/python3.8 script/python_agx_dual_arm.py
```

sudo rights is needed for keyboard commands
