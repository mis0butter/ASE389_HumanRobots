import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import time, math
from collections import OrderedDict
import copy
import signal
import shutil

import pybullet as p
import numpy as np
np.set_printoptions(precision=2)

from config.dog_config import SimConfig
from pnc.dog_pnc.dog_interface import dogInterface
from util import pybullet_util
from util import util
from util import liegroup


def set_initial_config(robot, joint_id):

    ## ATLAS STUFF 
    # shoulder_x
    # p.resetJointState(robot, joint_id["l_arm_shx"], -np.pi / 4, 0.)
    # p.resetJointState(robot, joint_id["r_arm_shx"], np.pi / 4, 0.)
    # # elbow_y
    # p.resetJointState(robot, joint_id["l_arm_ely"], -np.pi / 2, 0.)
    # p.resetJointState(robot, joint_id["r_arm_ely"], np.pi / 2, 0.)
    # # elbow_x
    # p.resetJointState(robot, joint_id["l_arm_elx"], -np.pi / 2, 0.)
    # p.resetJointState(robot, joint_id["r_arm_elx"], -np.pi / 2, 0.)
    # # hip_y
    # p.resetJointState(robot, joint_id["l_leg_hpy"], -np.pi / 4, 0.)
    # p.resetJointState(robot, joint_id["r_leg_hpy"], -np.pi / 4, 0.)
    # # knee
    # p.resetJointState(robot, joint_id["l_leg_kny"], np.pi / 2, 0.)
    # p.resetJointState(robot, joint_id["r_leg_kny"], np.pi / 2, 0.)
    # # ankle
    # p.resetJointState(robot, joint_id["l_leg_aky"], -np.pi / 4, 0.)
    # p.resetJointState(robot, joint_id["r_leg_aky"], -np.pi / 4, 0.)

    ## DOG STUFF  
    # hip (pelvis) joints? 
    # front left 
    p.resetJointState(robot, joint_id["fl.hx"], 0, 0)   # hip socket? 
    p.resetJointState(robot, joint_id["fl.hy"], 0, 0)   # hip socket? 
    p.resetJointState(robot, joint_id["fl.kn"], 0, 0)   # knee 
    # front right 
    p.resetJointState(robot, joint_id["fr.hx"], 0, 0)   # hip socket? 
    p.resetJointState(robot, joint_id["fr.hy"], 0, 0)   # hip socket? 
    p.resetJointState(robot, joint_id["fr.kn"], 0, 0)   # knee 
    # hind left 
    p.resetJointState(robot, joint_id["hl.hx"], 0, 0)   # hip socket? 
    p.resetJointState(robot, joint_id["hl.hy"], 0, 0)   # hip socket? 
    p.resetJointState(robot, joint_id["hl.kn"], 0, 0)   # knee 
    # hind right 
    p.resetJointState(robot, joint_id["hr.hx"], 0, 0)   # hip socket? 
    p.resetJointState(robot, joint_id["hr.hy"], 0, 0)   # hip socket? 
    p.resetJointState(robot, joint_id["hr.kn"], 0, 0)   # knee 



def signal_handler(signal, frame):
    # if SimConfig.VIDEO_RECORD:
    # pybullet_util.make_video(video_dir)
    p.disconnect()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":

    # Environment Setup
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(cameraDistance=1.5,
                                 cameraYaw=120,
                                 cameraPitch=-30,
                                 cameraTargetPosition=[1, 0.5, 1.5])
    p.setGravity(0, 0, -9.8)
    p.setPhysicsEngineParameter(fixedTimeStep=SimConfig.CONTROLLER_DT,
                                numSubSteps=SimConfig.N_SUBSTEP)
    # if SimConfig.VIDEO_RECORD:
    # video_dir = 'video/dog_pnc'
    # if os.path.exists(video_dir):
    # shutil.rmtree(video_dir)
    # os.makedirs(video_dir)

    # Create Robot, Ground
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    robot = p.loadURDF(cwd + "/robot_model/dog/dog.urdf",
                       SimConfig.INITIAL_POS_WORLD_TO_BASEJOINT,
                       SimConfig.INITIAL_QUAT_WORLD_TO_BASEJOINT)

    p.loadURDF(cwd + "/robot_model/ground/plane.urdf", [0, 0, 0])
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        robot, SimConfig.INITIAL_POS_WORLD_TO_BASEJOINT,
        SimConfig.INITIAL_QUAT_WORLD_TO_BASEJOINT, SimConfig.PRINT_ROBOT_INFO)

    # Initial Config
    set_initial_config(robot, joint_id)

    # Link Damping
    pybullet_util.set_link_damping(robot, link_id.values(), 0., 0.)

    # Joint Friction
    pybullet_util.set_joint_friction(robot, joint_id, 0)

    # Construct Interface
    interface = dogInterface()

    # Run Sim
    t = 0
    dt = SimConfig.CONTROLLER_DT
    count = 0

    while (1):

        # Get SensorData
        # if count % (SimConfig.CAMERA_DT / SimConfig.CONTROLLER_DT) == 0:
        # camera_img = pybullet_util.get_camera_image_from_link(
        # robot, link_id['head'], 60., 2., 0.1, 10)
        sensor_data = pybullet_util.get_sensor_data(robot, joint_id, link_id,
                                                    pos_basejoint_to_basecom,
                                                    rot_basejoint_to_basecom)

        ## ATLAST STUFF 
        # rf_height = pybullet_util.get_link_iso(robot, link_id['r_sole'])[2, 3]
        # lf_height = pybullet_util.get_link_iso(robot, link_id['l_sole'])[2, 3]
        # sensor_data['b_rf_contact'] = True if rf_height <= 0.01 else False
        # sensor_data['b_lf_contact'] = True if lf_height <= 0.01 else False

        ## DOG STUFF 
        fr_height = pybullet_util.get_link_iso(robot, link_id['fr.sole'])[2, 3]
        fl_height = pybullet_util.get_link_iso(robot, link_id['fl.sole'])[2, 3]
        hr_height = pybullet_util.get_link_iso(robot, link_id['hr.sole'])[2, 3]
        hl_height = pybullet_util.get_link_iso(robot, link_id['hl.sole'])[2, 3]
        sensor_data['b_fr_contact'] = True if fr_height <= 0.01 else False
        sensor_data['b_fl_contact'] = True if fl_height <= 0.01 else False
        sensor_data['b_hr_contact'] = True if hr_height <= 0.01 else False
        sensor_data['b_hl_contact'] = True if hl_height <= 0.01 else False

        import pdb; pdb.set_trace()


        # Get Keyboard Event
        keys = p.getKeyboardEvents()
        if pybullet_util.is_key_triggered(keys, '8'):
            interface.interrupt_logic.b_interrupt_button_eight = True
        elif pybullet_util.is_key_triggered(keys, '5'):
            interface.interrupt_logic.b_interrupt_button_five = True
        elif pybullet_util.is_key_triggered(keys, '4'):
            interface.interrupt_logic.b_interrupt_button_four = True
        elif pybullet_util.is_key_triggered(keys, '2'):
            interface.interrupt_logic.b_interrupt_button_two = True
        elif pybullet_util.is_key_triggered(keys, '6'):
            interface.interrupt_logic.b_interrupt_button_six = True
        elif pybullet_util.is_key_triggered(keys, '7'):
            interface.interrupt_logic.b_interrupt_button_seven = True
        elif pybullet_util.is_key_triggered(keys, '9'):
            interface.interrupt_logic.b_interrupt_button_nine = True

        # Compute Command
        if SimConfig.PRINT_TIME:
            start_time = time.time()
        command = interface.get_command(copy.deepcopy(sensor_data))

        if SimConfig.PRINT_TIME:
            end_time = time.time()
            print("ctrl computation time: ", end_time - start_time)

        # Apply Trq
        pybullet_util.set_motor_trq(robot, joint_id, command)

        p.stepSimulation()

        # time.sleep(dt)
        t += dt
        count += 1
