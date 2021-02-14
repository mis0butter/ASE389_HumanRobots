import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import time, math
from collections import OrderedDict

import pybullet as p
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from util import pybullet_util
from config.manipulator_config import ManipulatorConfig
from pnc.manipulator_pnc.manipulator_interface import ManipulatorInterface

if __name__ == "__main__":

    # Environment Setup
    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(cameraDistance=4.0,
                                 cameraYaw=0,
                                 cameraPitch=-45,
                                 cameraTargetPosition=[1.5, 0., 0.])
    p.setGravity(0, 0, -9.8)
    p.setPhysicsEngineParameter(fixedTimeStep=ManipulatorConfig.DT,
                                numSubSteps=ManipulatorConfig.N_SUBSTEP)
    if ManipulatorConfig.VIDEO_RECORD:
        if not os.path.exists('video'):
            os.makedirs('video')
        for f in os.listdir('video'):
            os.remove('video/' + f)
        p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "video/atlas.mp4")

    # Create Robot, Ground
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    robot = p.loadURDF(cwd +
                       "/robot_model/manipulator/three_link_manipulator.urdf",
                       useFixedBase=True)

    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    nq, nv, na, joint_id, link_id, pos_basejoint_to_basecom, rot_basejoint_to_basecom = pybullet_util.get_robot_config(
        robot)

    # Set Initial Config
    p.resetJointState(robot, 0, -np.pi / 6., 0.)
    p.resetJointState(robot, 1, np.pi / 6., 0.)
    p.resetJointState(robot, 2, np.pi / 3., 0.)

    # Joint Friction
    pybullet_util.set_joint_friction(robot, joint_id, 0.1)

    # Construct Interface
    interface = ManipulatorInterface()

    # Run Sim
    t = 0
    dt = ManipulatorConfig.DT
    count = 0

    # initialize position array 
    xytheta = pybullet_util.get_link_iso(robot, 0) 
    j0_pos = [ xytheta[0:3, 3] ]

    xytheta = pybullet_util.get_link_iso(robot, 1) 
    j1_pos = [ xytheta[0:3, 3] ]

    xytheta = pybullet_util.get_link_iso(robot, 2) 
    j2_pos = [ xytheta[0:3, 3] ]

    # position = np.array( [[ 0, 0, 0 ]] ) 

    while (t < 1 ):

        # Get SensorData
        sensor_data = pybullet_util.get_sensor_data(robot, joint_id, link_id,
                                                    pos_basejoint_to_basecom,
                                                    rot_basejoint_to_basecom)

        # Compute Command
        command = interface.get_command(sensor_data)

        # Apply Trq
        pybullet_util.set_motor_trq(robot, joint_id, command)

        p.stepSimulation()

        time.sleep(dt)
        t += dt
        count += 1

        # extract position 

        # # end effector configuration 
        # ee = self._robot.get_link_iso('ee')
        xytheta = pybullet_util.get_link_iso(robot, 0)
        xytheta = xytheta[0:3, 3]
        j0_pos = np.append( j0_pos, [xytheta], axis = 0 )

        xytheta = pybullet_util.get_link_iso(robot, 1)
        xytheta = xytheta[0:3, 3]
        j1_pos = np.append( j1_pos, [xytheta], axis = 0 )

        xytheta = pybullet_util.get_link_iso(robot, 2)
        xytheta = xytheta[0:3, 3]
        j2_pos = np.append( j2_pos, [xytheta], axis = 0 )


# plot things here 
print('x')
print(j0_pos[:,0])

ax = plt.axes(projection='3d')

line1, = ax.plot3D(j0_pos[:,0], j0_pos[:,1], j0_pos[:,2])
line2, = ax.plot3D(j1_pos[:,0], j1_pos[:,1], j1_pos[:,2])
line3, = ax.plot3D(j2_pos[:,0], j2_pos[:,1], j2_pos[:,2])
ax.set_title('j0, j1, j2 position')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.legend(( line1, line2, line3 ), ('j0', 'j1', 'j2'))
plt.show()

