import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import time, math

from pnc.interface import Interface
from config.manipulator_config import ManipulatorConfig


class ManipulatorInterface(Interface):
    def __init__(self):
        super(ManipulatorInterface, self).__init__()

        if ManipulatorConfig.DYN_LIB == "dart":
            from pnc.robot_system.dart_robot_system import DartRobotSystem
            self._robot = DartRobotSystem(
                cwd + "/robot_model/manipulator/three_link_manipulator.urdf",
                True, ManipulatorConfig.PRINT_ROBOT_INFO)
        elif ManipulatorConfig.DYN_LIB == "pinocchio":
            from pnc.robot_system.pinocchio_robot_system import PinocchioRobotSystem
            self._robot = PinocchioRobotSystem(
                cwd + "/robot_model/manipulator/three_link_manipulator.urdf",
                cwd + "/robot_model/manipulator", True,
                ManipulatorConfig.PRINT_ROBOT_INFO)
        else:
            raise ValueError("wrong dynamics library")

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
        jtrq = self._compute_osc_command()

        # Compute Cmd
        command = self._robot.create_cmd_ordered_dict()

        # Increase time variables
        self._count += 1
        self._running_time += ManipulatorConfig.DT

        return command

    def _compute_osc_command(self):
        ## TODO : Implement Operational Space Control
        jtrq = np.zeros(self._robot.n_a)
        return jtrq
