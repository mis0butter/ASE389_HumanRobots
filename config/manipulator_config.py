import numpy as np

from util import util


class ManipulatorConfig(object):
    DT = 0.01
    N_SUBSTEP = 1

    PRINT_ROBOT_INFO = True
    VIDEO_RECORD = False
    DYN_LIB = "pinocchio"  # "dart"

    DES_EE_POS = np.array([0., 0., 0.])
    # KP = .15
    # KD = .04

    KP = 25
    KD = 10
