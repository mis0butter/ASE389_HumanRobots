import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import time, math
import matplotlib 

import numpy as np

from pnc.interface import Interface
from config.manipulator_config import ManipulatorConfig
from pnc.robot_system.pinocchio_robot_system import PinocchioRobotSystem


class ManipulatorInterface_q3(Interface):
    def __init__(self):
        super(ManipulatorInterface_q3, self).__init__()

        self._robot = PinocchioRobotSystem(
            cwd + "/robot_model/manipulator/three_link_manipulator.urdf",
            cwd + "/robot_model/manipulator", True,
            ManipulatorConfig.PRINT_ROBOT_INFO)

    def get_command(self, sensor_data):
        # Update Robot
        self._robot.update_system(
            sensor_data["base_com_pos"], sensor_data["base_com_quat"],
            sensor_data["base_com_lin_vel"], sensor_data["base_com_ang_vel"],
            sensor_data["base_joint_pos"], sensor_data["base_joint_quat"],
            sensor_data["base_joint_lin_vel"],
            sensor_data["base_joint_ang_vel"], sensor_data["joint_pos"],
            sensor_data["joint_vel"])

        # Operational Space Control
        jtrq_cmd = self._compute_osc_command()
        jpos_cmd = np.zeros_like(jtrq_cmd)
        jvel_cmd = np.zeros_like(jtrq_cmd)

        # Compute Cmd
        command = self._robot.create_cmd_ordered_dict(jpos_cmd, jvel_cmd,
                                                      jtrq_cmd)

        # Increase time variables
        self._count += 1
        self._running_time += ManipulatorConfig.DT

        return command

    def _compute_osc_command(self):
        ## TODO : Implement Operational Space Control
        # jtrq = np.zeros(self._robot.n_a)

        # proportional K 
        KP = ManipulatorConfig.KP 

        # derivative K 
        KD = ManipulatorConfig.KD 

        # q segment stuff 
        q = self._robot.get_q() 
        q_des = np.array( [ np.pi/4, np.pi/6, np.pi/12 ] )
        q_dot = self._robot.get_q_dot() 
        q_dot_des = np.array( [0, 0, 0])

        # coriolis forces 
        cf = self._robot.get_coriolis() 

        # control law 
        q_dot_dot = - KP * ( q - q_des ) - KD * ( q_dot - q_dot_des )

        # mass matrix 
        A = self._robot.get_mass_matrix() 

        # calculate torque 
        jtrq = A.dot(q_dot_dot) 

        # position = 0 
        # position.append = xytheta 
        
        # print(self._robot.get_mass_matrix()) 

        return jtrq
