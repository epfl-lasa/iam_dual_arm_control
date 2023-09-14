
import sys
import numpy as np
import keyboard
from dataclasses import dataclass
from enum import IntEnum

# AGX
from pclick import Client
from pclick import MessageFactory
from scipy.spatial.transform import Rotation as R
from roboticstoolbox.robot.ERobot import ERobot

# Python wrapper lib
sys.path.append("../dual_arm_control/python_binding/build")
sys.path.append("../../dual_iiwa_toolkit/python_binding/build")
from py_dual_passive_control import PassiveControl
from py_dual_arm_control import DualArmControlSim


# ------------ VARIABLE TO MODIFY ------------
# -- Agx robot order
ROBOT_LEFT = 1
ROBOT_RIGHT = 2

ROBOT_BASE_POSE = np.array([[0, -0.51, 0.75], [0, 0.51, 0.75]])
ROBOT_BASE_ORIENTATION = np.array([[1, 0, 0, 0.0], [1, 0, 0, 0.0]])

DUAL_ROBOT_FRAME = 0.5*(ROBOT_BASE_POSE[0] + ROBOT_BASE_POSE[1])
DELTA_TIME = 0.005

# -- passive controller
URDF_PATH_LEFT = "/home/elise/Documents/IAM/michael/dual_arm_control_iam/agx_dual_arm/urdf/iiwa14.urdf"
END_EFFECTOR_LEFT = "iiwa_link_ee"
URDF_PATH_RIGHT = "/home/elise/Documents/IAM/michael/dual_arm_control_iam/agx_dual_arm/urdf/iiwa14.urdf"
END_EFFECTOR_RIGHT = "iiwa_link_ee"

DES_POS_ROBOT_LEFT = np.array([0.35,  0.20, 0.80])
DES_ORI_ROBOT_LEFT = np.array([0.3827, -0.9239, 0.0, 0.0])
DES_POS_ROBOT_RIGHT = np.array([0.35,  -0.20, 0.80])
DES_ORI_ROBOT_RIGHT = np.array([0.3827,  0.9239, 0.0, 0.0])

DS_GAIN_POS = 6.0
DS_GAIN_ORI = 3.0
LAMBDA_POS = [70.0, 70.0]
LAMBDA_ORI = [6.0, 6.0]

passive_ds_controller = [180.0, 180.0]

# -- tossing and throwing controller
PATH_YAML_FILE = "config/parameters.yaml"
PATH_LEARNED_MODEL_FOLDER = "config/LearnedModel/model1"

# --------------------------------------------

class loggertype(IntEnum):
    REACH = 0
    PICK_AND_LIFT = 1
    TOSSING = 2
    PICK_AND_TOSS = 3
    PICK_AND_PLACE = 4
    PLACE_TOSSING = 5
    THROWING = 6
    HANDINGOVER = 7
    PAUSE_MOTION = 8

@dataclass
class Robot:
    def __init__(self, urdf_path):
        self.robot_toolbox = ERobot.URDF(urdf_path, "iiwa_link_ee")
        self.joint_position = [0, 0, 0, 0, 0, 0, 0]
        self.joint_velocity = [0, 0, 0, 0, 0, 0, 0]
        self.joint_effort = [0, 0, 0, 0, 0, 0, 0]
        self.ee_wrench = [0, 0, 0, 0, 0, 0]

        self.robot_toolbox.q = self.joint_position
        self.ee_vel = self.robot_toolbox.jacob0(self.joint_position) @ self.joint_velocity
    
    def get_ee_vel(self):
        self.robot_toolbox.q = self.joint_position
        self.ee_vel = self.robot_toolbox.jacob0(self.joint_position) @ self.joint_velocity
        return np.array(self.ee_vel)

@dataclass
class Object:
    position = [0, 0, 0]
    orientation = [0, 0, 0, 0]

def keyboard_input(state_machine, keyboard_waiting):
    if keyboard.is_pressed('q'):
        print(f"------------------------------- KEY Q - Go Home {not state_machine.goHome} ")
        state_machine.goHome = not state_machine.goHome
        if state_machine.goHome:
            state_machine.goToAttractors = True
        keyboard_waiting = False
    elif keyboard.is_pressed('g'):
        print(f"------------------------------- KEY G - Go to attractors {not state_machine.goToAttractors} ")
        state_machine.goToAttractors = not state_machine.goToAttractors
        if state_machine.goToAttractors:
            state_machine.goHome = False
            state_machine.releaseAndretract = False
        keyboard_waiting = False
        
    # Release or throwing
    elif keyboard.is_pressed('t'):
        print(f"------------------------------- KEY T - Throwing {not state_machine.isThrowing} ")
        state_machine.isThrowing = not state_machine.isThrowing
        if state_machine.isThrowing: 
          state_machine.dualTaskSelector = int(loggertype.PICK_AND_TOSS)
        else:
          state_machine.dualTaskSelector = int(loggertype.PICK_AND_LIFT)
        keyboard_waiting = False

    elif keyboard.is_pressed('r'):
        print(f"------------------------------- KEY R - Release and Retract {not state_machine.releaseAndretract} ")
        state_machine.releaseAndretract = not state_machine.releaseAndretract
        keyboard_waiting = False

    elif keyboard.is_pressed('l'):
        print(f"------------------------------- KEY L - PICK AND LIFT ")
        state_machine.dualTaskSelector = int(loggertype.PICK_AND_LIFT)
        keyboard_waiting = False

    elif keyboard.is_pressed('p'):
        print(f"------------------------------- KEY P - Is placing {not state_machine.isPlacing} ")
        state_machine.isPlacing = not state_machine.isPlacing
        if state_machine.isPlacing: 
          state_machine.dualTaskSelector = int(loggertype.PICK_AND_PLACE)
        else:
          state_machine.dualTaskSelector = int(loggertype.PICK_AND_LIFT)
        keyboard_waiting = False
    elif keyboard.is_pressed('o'):
        print(f"------------------------------- KEY O - Is place tossing {not state_machine.isPlaceTossing} ")
        state_machine.isPlaceTossing = not state_machine.isPlaceTossing
        if state_machine.isPlaceTossing: 
          state_machine.dualTaskSelector = int(loggertype.PLACE_TOSSING)
        else:
          state_machine.dualTaskSelector = int(loggertype.PICK_AND_LIFT)
        keyboard_waiting = False

    # Impact and tossing velocity
    elif keyboard.is_pressed('v'):
        print(f"------------------------------- KEY V - Lower V toss desired {state_machine.desVtoss - 0.05} ")
        state_machine.desVtoss = state_machine.desVtoss - 0.05
        if state_machine.desVtoss < 0.2:
            state_machine.desVtoss = 0.2
        keyboard_waiting = False
    elif keyboard.is_pressed('b'):
        print(f"------------------------------- KEY B - Higher V toss desired: {state_machine.desVtoss + 0.05} ")
        state_machine.desVtoss = state_machine.desVtoss + 0.05
        if state_machine.desVtoss > 2.0:
            state_machine.desVtoss = 2.0
        keyboard_waiting = False
    elif keyboard.is_pressed('y'):
        print(f"------------------------------- KEY Y - Lower V imp desired: {state_machine.desiredVelImp - 0.05} ")
        state_machine.desiredVelImp = state_machine.desiredVelImp - 0.05
        if state_machine.desiredVelImp < 0.05:
            state_machine.desiredVelImp = 0.05
        keyboard_waiting = False
    elif keyboard.is_pressed('v'):
        print(f"------------------------------- KEY V - Higher V imp desired: {state_machine.desiredVelImp + 0.05} ")
        state_machine.desiredVelImp = state_machine.desiredVelImp + 0.05
        if state_machine.desiredVelImp > 0.6:
            state_machine.desiredVelImp = 0.6
        keyboard_waiting = False

    # Placing height
    elif keyboard.is_pressed('x'):
        if state_machine.dualTaskSelector == int(loggertype.PICK_AND_TOSS):
            print(f"------------------------------- KEY X - Lower release pos y: {state_machine.releasePosY - 0.01} ")
            state_machine.releasePosY = state_machine.releasePosY - 0.01
        else:
            print(f"------------------------------- KEY X - Lower placing pos height: {state_machine.placingPosHeight - 0.01} ")
            state_machine.placingPosHeight = state_machine.placingPosHeight - 0.01
        keyboard_waiting = False

    elif keyboard.is_pressed('n'):
        if state_machine.dualTaskSelector == int(loggertype.PICK_AND_TOSS):
            print(f"------------------------------- KEY X - Higher release pos y: {state_machine.releasePosY + 0.01} ")
            state_machine.releasePosY = state_machine.releasePosY + 0.01
        else:
            print(f"------------------------------- KEY X - Higher placing pos height: {state_machine.placingPosHeight + 0.01} ")
            state_machine.placingPosHeight = state_machine.placingPosHeight + 0.01
        keyboard_waiting = False

    return state_machine, keyboard_waiting

def get_agx_sensors(robot_left, robot_right, box):
    message = MessageFactory.create_sensorrequestmessage()
    client.send(message)
    response = client.recv()

    dual_robot_joint_position = np.array(
        response.objects['dualrobot'].angleSensors) # RAD
    dual_robot_joint_velocity = np.array(
        response.objects['dualrobot'].angleVelocitySensors)
    dual_robot_joint_effort = np.array(response.objects['dualrobot'].torqueSensors)

    robot_left.joint_position = dual_robot_joint_position[(ROBOT_LEFT-1)*7: (ROBOT_LEFT-1)*7 + 7]
    robot_left.joint_velocity = dual_robot_joint_velocity[(ROBOT_LEFT-1)*7: (ROBOT_LEFT-1)*7 + 7]
    robot_left.joint_effort = dual_robot_joint_effort[(ROBOT_LEFT-1)*7: (ROBOT_LEFT-1)*7 + 7]

    robot_right.joint_position = dual_robot_joint_position[(ROBOT_RIGHT-1)*7: (ROBOT_RIGHT-1)*7 + 7]
    robot_right.joint_velocity = dual_robot_joint_velocity[(ROBOT_RIGHT-1)*7: (ROBOT_RIGHT-1)*7 + 7]
    robot_right.joint_effort = dual_robot_joint_effort[(ROBOT_RIGHT-1)*7: (ROBOT_RIGHT-1)*7 + 7]

    # FT sensor
    if ROBOT_LEFT < ROBOT_RIGHT:
        robot_left.ee_wrench = np.array(list(response.objects['dualrobot'].sensors['IiwaForceSensor1'].sensor[0].force.arr) + \
                               list(response.objects['dualrobot'].sensors['IiwaForceSensor1'].sensor[1].directionalTorque.arr))
        robot_right.ee_wrench = np.array(list(response.objects['dualrobot'].sensors['IiwaForceSensor2'].sensor[0].force.arr) + \
                               list(response.objects['dualrobot'].sensors['IiwaForceSensor2'].sensor[1].directionalTorque.arr))

    if ROBOT_RIGHT < ROBOT_LEFT:
        robot_right.ee_wrench = np.array(list(response.objects['dualrobot'].sensors['IiwaForceSensor1'].sensor[0].force.arr) + \
                               list(response.objects['dualrobot'].sensors['IiwaForceSensor1'].sensor[1].directionalTorque.arr))
        robot_left.ee_wrench = np.array(list(response.objects['dualrobot'].sensors['IiwaForceSensor2'].sensor[0].force.arr) + \
                               list(response.objects['dualrobot'].sensors['IiwaForceSensor2'].sensor[1].directionalTorque.arr))


    # Box
    box.position = np.array(response.objects['box'].objectSensors[0].position.arr)
    box_ori = np.array(response.objects['box'].objectSensors[1].rpy.arr)
    box_transfo = R.from_euler('xzy', [box_ori[0], box_ori[1], box_ori[2]], degrees=True)
    box.orientation = np.array([box_transfo.as_quat()[3], box_transfo.as_quat()[0], box_transfo.as_quat()[1], box_transfo.as_quat()[2]])

    # Target
    if 'target' in response.objects:
        target.position = np.array(response.objects['target'].objectSensors[0].position.arr)
        target_ori = np.array(response.objects['target'].objectSensors[1].rpy.arr)
        r = R.from_euler('xzy', [target_ori[0], target_ori[1], target_ori[2]], degrees=True)
        target.orientation = np.array([r.as_quat()[3], r.as_quat()[0], r.as_quat()[1], r.as_quat()[2]])

def send_command_agx(command_left, command_right):
    message = MessageFactory.create_controlmessage()

    if ROBOT_LEFT < ROBOT_RIGHT:
        dual_command = list(command_left) + list(command_right)
    elif ROBOT_RIGHT < ROBOT_LEFT:
        dual_command = list(command_right) + list(command_left)

    dual_robot_msg = message.objects['dualrobot']
    dual_robot_msg.torques.extend(list(dual_command))

    client.send(message)
    response = client.recv()

def reset_sim_agx():
    # Send 0 velocity
    message = MessageFactory.create_controlmessage()
    send_command_agx([0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0])

    # Reset Sim
    message = MessageFactory.create_resetmessage()
    client.send(message)
    response = client.recv()

def init_passive_controllers(robot_left, robot_right):
    controller_left = PassiveControl(open(URDF_PATH_LEFT).read(), END_EFFECTOR_LEFT)
    controller_left.set_desired_pose(DES_POS_ROBOT_LEFT, DES_ORI_ROBOT_LEFT)
    controller_left.set_pos_gains(DS_GAIN_POS, LAMBDA_POS[0], LAMBDA_POS[1])
    controller_left.set_ori_gains(DS_GAIN_ORI, LAMBDA_ORI[0], LAMBDA_ORI[1])
    controller_left.updateRobot(robot_left.joint_position,
                           robot_left.joint_velocity, robot_left.joint_effort)
    

    controller_right = PassiveControl(open(URDF_PATH_RIGHT).read(), END_EFFECTOR_RIGHT)
    controller_right.set_desired_pose(DES_POS_ROBOT_RIGHT, DES_ORI_ROBOT_RIGHT)
    controller_right.set_pos_gains(DS_GAIN_POS, LAMBDA_POS[0], LAMBDA_POS[1])
    controller_right.set_ori_gains(DS_GAIN_ORI, LAMBDA_ORI[0], LAMBDA_ORI[1])
    controller_right.updateRobot(robot_right.joint_position,
                           robot_right.joint_velocity, robot_right.joint_effort)
    
    return controller_left, controller_right

def init_dual_arm_controller():
    dual_arm_control_agx = DualArmControlSim(DELTA_TIME)
    dual_arm_control_agx.loadParamFromFile(PATH_YAML_FILE, PATH_LEARNED_MODEL_FOLDER)
    dual_arm_control_agx.init()
    return dual_arm_control_agx

if __name__ == '__main__':

    # Connect to AGX sim
    addr = f"tcp://localhost:5555"
    client = Client()
    print(f"Connecting to click server {addr}")
    client.connect(addr)
    reset_sim_agx()

    # Init robots and object
    robot_left = Robot(URDF_PATH_LEFT)
    robot_right = Robot(URDF_PATH_RIGHT)
    box = Object()
    target = Object()

    # Init passive controller
    get_agx_sensors(robot_left, robot_right, box)
    controller_left, controller_right = init_passive_controllers(robot_left, robot_right)

    # Init dual arm controller
    dual_arm_control_agx = init_dual_arm_controller()

    max_cycle_init_pos = 10
    cycle_count = 0
    robots_init_pos = False
    cycle_init_pos = 0
    keyboard_waiting = True
    keyboard_waiting_cycle = 0

    while True:
        # Update controllers
        get_agx_sensors(robot_left, robot_right, box)
        controller_left.updateRobot(robot_left.joint_position,
                                    robot_left.joint_velocity, robot_left.joint_effort)
        controller_right.updateRobot(robot_right.joint_position,
                                     robot_right.joint_velocity, robot_right.joint_effort)

        if np.linalg.norm(DES_POS_ROBOT_LEFT - controller_left.getEEpos()) < 0.001 and \
           np.linalg.norm(DES_POS_ROBOT_RIGHT - controller_right.getEEpos()) < 0.001 and \
           (np.linalg.norm(DES_ORI_ROBOT_LEFT - controller_left.getEEquat()) < 0.01 or \
            np.linalg.norm(DES_ORI_ROBOT_LEFT + controller_left.getEEquat()) < 0.01) and \
           (np.linalg.norm(DES_ORI_ROBOT_RIGHT - controller_right.getEEquat()) < 0.01 or \
            np.linalg.norm(DES_ORI_ROBOT_RIGHT + controller_right.getEEquat()) < 0.01):
            cycle_init_pos = cycle_init_pos + 1
            if cycle_init_pos >= max_cycle_init_pos:
                robots_init_pos = True

        if robots_init_pos:
            if not keyboard_waiting and cycle_count - keyboard_waiting_cycle > 10:
                keyboard_waiting = True

            if keyboard_waiting:
                state_machine = dual_arm_control_agx.getStateMachine()
                state_machine, keyboard_waiting = keyboard_input(state_machine, keyboard_waiting)
                dual_arm_control_agx.updateStateMachine(state_machine)
                keyboard_waiting_cycle = cycle_count            

            commandGenerated = dual_arm_control_agx.generateCommands(
                passive_ds_controller,
                [np.array(robot_left.ee_wrench), np.array(robot_right.ee_wrench)],
                [controller_left.getEEpos() + ROBOT_BASE_POSE[0] - DUAL_ROBOT_FRAME,
                 controller_right.getEEpos() + ROBOT_BASE_POSE[1] - DUAL_ROBOT_FRAME],
                [controller_left.getEEquat(), controller_right.getEEquat()],
                box.position - DUAL_ROBOT_FRAME,
                box.orientation,
                target.position - DUAL_ROBOT_FRAME,
                target.orientation,
                [robot_left.get_ee_vel()[0:3], robot_right.get_ee_vel()[0:3]],
                [robot_left.get_ee_vel()[3:7], robot_right.get_ee_vel()[3:7]],
                [robot_left.joint_position, robot_right.joint_position],
                [robot_left.joint_velocity, robot_right.joint_velocity],
                [robot_left.joint_effort, robot_right.joint_effort],
                [np.array(ROBOT_BASE_POSE[0] - DUAL_ROBOT_FRAME), np.array(ROBOT_BASE_POSE[1] - DUAL_ROBOT_FRAME)],
                [np.array(ROBOT_BASE_ORIENTATION[0]), np.array(ROBOT_BASE_ORIENTATION[1])],
                cycle_count
            )

            if np.linalg.norm(commandGenerated.vDes[ROBOT_LEFT-1]) < 3 and np.linalg.norm(commandGenerated.omegaDes[ROBOT_LEFT-1]) < 7.5:
                controller_left.set_desired_twist(commandGenerated.vDes[ROBOT_LEFT-1], commandGenerated.omegaDes[ROBOT_LEFT-1])
            norm_force  = -commandGenerated.nuWr0 * commandGenerated.appliedWrench[ROBOT_LEFT-1][0:3]
            moment = -commandGenerated.nuWr0 * commandGenerated.appliedWrench[ROBOT_LEFT-1][3:6]
            if np.linalg.norm(norm_force) > 0:
                norm_force = norm_force / np.linalg.norm(norm_force)
            controller_left.set_force_normal(norm_force)
            controller_left.set_external_moment(moment)

            if np.linalg.norm(commandGenerated.vDes[ROBOT_RIGHT-1]) < 3 and np.linalg.norm(commandGenerated.omegaDes[ROBOT_RIGHT-1]) < 7.5:
                controller_right.set_desired_twist(commandGenerated.vDes[ROBOT_RIGHT-1], commandGenerated.omegaDes[ROBOT_RIGHT-1])
            norm_force  = -commandGenerated.nuWr0 * commandGenerated.appliedWrench[ROBOT_RIGHT-1][0:3]
            moment = -commandGenerated.nuWr0 * commandGenerated.appliedWrench[ROBOT_RIGHT-1][3:6]
            if np.linalg.norm(norm_force) > 0:
                norm_force = norm_force / np.linalg.norm(norm_force)
            controller_right.set_force_normal(norm_force)
            controller_right.set_external_moment(moment)

            cycle_count = cycle_count + 1


        # Get and send command
        command_left = controller_left.getCmd()
        command_right = controller_right.getCmd()
        send_command_agx(command_left, command_right)

