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


class ManipulatorInterface(Interface):
    def __init__(self):
        super(ManipulatorInterface, self).__init__()

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

        # QUESTION 2 --------------------------------------------------------------- # 

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
        # jtrq = A.dot(q_dot_dot) 

        # QUESTION 3 --------------------------------------------------------------- # 
        
        l1 = 1 
        l2 = 1
        l3 = 1 

        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        
        ee = self._robot.get_link_iso('ee') 

        ee = ee[0:3, 3]
        ee_dot = self._robot.get_link_vel('ee') 
        ee_dot = ee_dot[0:3]

        ee_des = ManipulatorConfig.DES_EE_POS 
        ee_des_dot = np.array([0, 0, 0])

        # control law 

        ee_dot_dot = - KP * ( ee - ee_des ) - KD * ( ee_dot - ee_des_dot )

        ee_Jxq1 = -l1 * np.sin( q1 ) - l2 * np.sin( q1 - q2 ) - l3 * np.sin( q1 - q2 + q3 ) 
        ee_Jxq2 = -l2 * np.sin( q1 + q2 ) - l3 * np.sin( q1 + q2 + q3 ) 
        ee_Jxq3 = -l3 * np.sin( q1 + q2 + q3 )

        ee_Jyq1 = l1 * np.cos( q1 ) + l2 * np.cos( q1 - q2 ) + l3 * np.cos( q1 - q2 + q3 ) 
        ee_Jyq2 = l2 * np.cos( q1 + q2 ) + l3 * np.cos( q1 + q2 + q3 ) 
        ee_Jyq3 = l3 * np.cos( q1 + q2 + q3 ) 

        ee_Jthq1 = 1
        ee_Jthq2 = 1 
        ee_Jthq3 = 1 

        ee_J = np.array( [[ee_Jxq1, ee_Jxq2, ee_Jxq3], [ ee_Jyq1, ee_Jyq2, ee_Jyq3 ], [ ee_Jthq1, ee_Jthq2, ee_Jthq3 ] ] ) 

        # ee_J = self._robot.get_link_jacobian('ee')
        # ee_J = ee_J[3:6,:]

        ee_JT = np.transpose(np.asmatrix(ee_J))
        # ee_J.asmatrix()  
        A_mat = np.asmatrix(A) 
        ee_dot_dotT_mat = np.transpose(np.asmatrix(ee_dot_dot)) 

        F = np.linalg.pinv(A_mat).dot(ee_dot_dotT_mat)

        # jtrq = F.dot(ee_JT)

        # this one 
        jtrq = ee_JT.dot(F)

        # QUESTION 4 --------------------------------------------------------------- # 

        # import pdb ; pdb.set_trace() 

        N = np.eye(3) - np.linalg.pinv(ee_J) * ee_J

        J1 = ee_J; 

        T1 = J1 * F 

        T2 = A.dot(q_dot_dot); 

        # jtrq = T1 + N * T2 






        # WBC-EOM
        # A * gen_acc + Nc' * (b + g) + Jc * Mc * Jc_dot * gen_pos = (U * Nc)' * T 

        # choose T s.t. task coordinate xi will follow ref trajectories 
        # transform WBC-EOM into task-space - multiply by ( U * Nc * A^-1 )

        # ( U * Nc * A^-1 ) * A * gen_acc + ( U * Nc * A^-1 ) * Nc' * (b + g) + ( U * Nc * A^-1 ) * Jc * Mc * Jc_dot * gen_pos = ( U * Nc * A^-1 ) * (U * Nc)' * T 

        # ( U * Nc * gen_acc ) + ( U * Nc * A^-1 * Nc' * (b + g) ) + ( U * Nc * A^-1 * Jc * Mc * Jc_dot * gen_pos ) = U * Nc * A^-1 * (U _ Nc)' * T 

        # q_dot_dot = U * Nc * gen_acc + d/dt(U * Nc) * gen_vel
        # q_dot_dot - d/dt(U * Nc) * gen_vel + ( U * Nc * A^-1 * Nc' * (b + g) ) + ( U * Nc * A^-1 * Jc * Mc * Jc_dot * gen_pos ) = U * Nc * A^-1 * (U _ Nc)' * T 

        # left-multiply by J_dot 
        # x_dot_dot - J_dot * q_dot - J_dot * d/dt(U * Nc) * gen_vel + J_dot * ( U * Nc * A^-1 * Nc' * (b + g) ) + J_dot * ( U * Nc * A^-1 * Jc * Mc * Jc_dot * gen_pos ) = J_dot * U * Nc * A^-1 * (U _ Nc)' * T 

        # plug in linear control T = J' * F
        # x_dot_dot - J_dot * q_dot - J_dot * d/dt(U * Nc) * gen_vel + J_dot * ( U * Nc * A^-1 * Nc' * (b + g) ) + J_dot * ( U * Nc * A^-1 * Jc * Mc * Jc_dot * gen_pos ) = J_dot * U * Nc * A^-1 * (U _ Nc)' * J' * F 

        # phi = U * Nc * A^-1 * (U _ Nc)' 
        # M = ( J * phi * J' )^-1
        # x_dot_dot - J_dot * q_dot - J_dot * d/dt(U * Nc) * gen_vel + J_dot * ( U * Nc * A^-1 * Nc' * (b + g) ) + J_dot * ( U * Nc * A^-1 * Jc * Mc * Jc_dot * gen_pos ) = M^-1 * F 

        # simplify 
        # d/dt(U * Nc) = 0 (unchanging)
        # x_dot_dot - J_dot * q_dot + J_dot * ( U * Nc * A^-1 * Nc' * (b + g) ) + J_dot * ( U * Nc * A^-1 * Jc * Mc * Jc_dot * gen_pos ) = M^-1 * F 

        # rewrite for F 
        # F = M * ( x_dot_dot - J_dot * q_dot + J_dot * ( U * Nc * A^-1 * Nc' * (b + g) ) + J_dot * ( U * Nc * A^-1 * Jc * Mc * Jc_dot * gen_pos ) )

        # decoupled control of task accelerations, i.e. "other terms" go to 0 
        # x_dot_dot + "other terms" = M^-1 * M^+ * (a_ref + "other terms")
        # --> 
        # x_dot_dot = a_ref 

        # F = M * a_ref 

        return jtrq
