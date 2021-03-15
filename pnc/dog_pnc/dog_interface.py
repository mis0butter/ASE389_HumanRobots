import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import time, math
import copy

import pybullet as p

from pnc.interface import Interface
from config.dog_config import PnCConfig
from pnc.dog_pnc.dog_interrupt_logic import dogInterruptLogic
from pnc.dog_pnc.dog_state_provider import dogStateProvider
from pnc.dog_pnc.dog_state_estimator import dogStateEstimator
from pnc.dog_pnc.dog_control_architecture import dogControlArchitecture
from pnc.data_saver import DataSaver


class dogInterface(Interface):
    def __init__(self):
        super(dogInterface, self).__init__()

        if PnCConfig.DYN_LIB == "dart":
            from pnc.robot_system.dart_robot_system import DartRobotSystem
            self._robot = DartRobotSystem(
                cwd + "/robot_model/dog/dog_rel_path.urdf", False,
                PnCConfig.PRINT_ROBOT_INFO)
        elif PnCConfig.DYN_LIB == "pinocchio":
            from pnc.robot_system.pinocchio_robot_system import PinocchioRobotSystem
            self._robot = PinocchioRobotSystem(
                cwd + "/robot_model/dog/dog.urdf",
                cwd + "/robot_model/dog", False, PnCConfig.PRINT_ROBOT_INFO)
        else:
            raise ValueError("wrong dynamics library")

        self._sp = dogStateProvider(self._robot)
        self._se = dogStateEstimator(self._robot)
        self._control_architecture = dogControlArchitecture(self._robot)
        self._interrupt_logic = dogInterruptLogic(self._control_architecture)
        if PnCConfig.SAVE_DATA:
            self._data_saver = DataSaver()

    def get_command(self, sensor_data):
        if PnCConfig.SAVE_DATA:
            self._data_saver.add('time', self._running_time)
            self._data_saver.add('phase', self._control_architecture.state)

        # Update State Estimator
        if self._count == 0:
            print("=" * 80)
            print("Initialize")
            print("=" * 80)
            self._se.initialize(sensor_data)
        self._se.update(sensor_data)

        # Process Interrupt Logic
        self._interrupt_logic.process_interrupts()

        # Compute Cmd
        command = self._control_architecture.get_command()

        if PnCConfig.SAVE_DATA and (self._count % PnCConfig.SAVE_FREQ == 0):
            self._data_saver.advance()

        # Increase time variables
        self._count += 1
        self._running_time += PnCConfig.CONTROLLER_DT
        self._sp.curr_time = self._running_time
        self._sp.prev_state = self._control_architecture.prev_state
        self._sp.state = self._control_architecture.state

        return copy.deepcopy(command)

    @property
    def interrupt_logic(self):
        return self._interrupt_logic
