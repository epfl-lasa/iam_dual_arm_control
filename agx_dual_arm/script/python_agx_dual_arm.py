
import numpy as np

# AGX
from pclick import Client
from pclick import MessageFactory
from scipy.spatial.transform import Rotation as R

# Python wrapper lib
import sys
sys.path.append("../dual_arm_control/python_binding/build")
sys.path.append("../../dual_iiwa_toolkit/python_binding/build")
from py_dual_passive_control import PassiveControl
from py_dual_arm_control import DualArmControlSim

# ------------ VARIABLE TO MODIFY ------------
dt = 0.005

# -- passive controller
urdf_path = "urdf/iiwa14.urdf"
end_effector = "iiwa_link_ee"
des_pos = np.array([0.5, -0.25, 0.3])
des_quat = np.array([0.7071068, -0.7071068, 0.0,  0.0])
ds_gain_pos = 6.0
lambda0_pos = 70.0
lambda1_pos = 35.0
ds_gain_ori = 3.0
lambda0_ori = 5.0
lambda1_ori = 2.5

# -- tossing and throwing controller
path_yaml_file = 

# --------------------------------------------


def reset_sim_agx():
    # Send 0 velocity
    message = MessageFactory.create_controlmessage()
    robot1_msg = message.objects["robot1"]
    robot1_msg.torques.extend([0, 0, 0, 0, 0, 0, 0])
    robot2_msg = message.objects["robot2"]
    robot2_msg.torques.extend([0, 0, 0, 0, 0, 0, 0])
    client.send(message)
    response = client.recv()
    # Reset Sim
    message = MessageFactory.create_resetmessage()
    client.send(message)
    response = client.recv()

if __name__ == '__main__':

    # Connect to AGX sim
    addr = f"tcp://localhost:5555"
    client = Client()
    print(f"Connecting to click server {addr}")
    client.connect(addr)
    reset_sim_agx()

    # Init dual arm controller
    dual_arm_control_agx = DualArmControlSim(dt)