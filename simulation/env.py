import re
import time
import math
import random

import numpy as np
import pybullet as pb
import pybullet_data

from utilities import Models, Camera
from tqdm import tqdm
from task import load_bricks, draw_boundaries
from mp import *
from constants import *

class FailToReachTargetError(RuntimeError):
    pass

class ClutteredPushGrasp:

    SIMULATION_STEP_DELAY = 1 / 240.

    def __init__(self, robot, models: Models, camera=None, vis=False) -> None:
        # define environment
        self.vis = vis
        if self.vis:
            self.p_bar = tqdm(ncols=0, disable=False)
        self.physicsClient = pb.connect(pb.GUI if self.vis else pb.DIRECT)
        ## set data path and gravity
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -10)
        ## load the workspace and plane
        self.workspace = pb.loadURDF("./urdf/workspace/workspace.urdf", basePosition=(0.0, 0, 0), useFixedBase=True)
        self.planeID = pb.loadURDF("plane.urdf", basePosition=(0, 0, -0.0005), useFixedBase=True)

        # build the robot
        self.robot = robot
        self.robot.load()
        self.robot.step_simulation = self.step_simulation
        ## custom sliders to tune parameters (name of the parameter,range,initial value)
        self.set_debug_params()

        # set debug and monitor cameras
        self.camera = camera
        ## adjust the camera parameters to be closer to the robot
        pb.resetDebugVisualizerCamera(
            cameraDistance=1,  # Reduced camera distance to be closer
            cameraYaw=0, 
            cameraPitch=-30, 
            cameraTargetPosition=[0, 0, 0]
        )

        # load the cluttered objects
        self.load_objects(workspace_limits=WORKSPACE_LIMITS)

    def set_debug_params(self):
        self.debug_x = pb.addUserDebugParameter("x", -0.224, 0.224, 0)
        self.debug_y = pb.addUserDebugParameter("y", -0.224, 0.224, 0)
        self.debug_z = pb.addUserDebugParameter("z", 0, 1., 0.5)
        self.debug_r = pb.addUserDebugParameter("roll", -3.14, 3.14, 0)
        self.debug_p = pb.addUserDebugParameter("pitch", -3.14, 3.14, 1/2)
        self.debug_y = pb.addUserDebugParameter("yaw", -1/2, 1/2, 1/2)
        self.debug_gripper_open_length = pb.addUserDebugParameter("gripper_opening_length", 0, 0.085, 0)
        # joint angles
        self.debug_ja0 = pb.addUserDebugParameter("joint_angle0", -1, 1, 0.5)
        self.debug_ja1 = pb.addUserDebugParameter("joint_angle1", -1, 1, -0.25)
        self.debug_ja2 = pb.addUserDebugParameter("joint_angle2", -1, 1, -0.6)
        self.debug_ja3 = pb.addUserDebugParameter("joint_angle3", -1, 1, -0.65)
        self.debug_ja4 = pb.addUserDebugParameter("joint_angle4", -1, 1, 0.5)
        self.debug_ja5 = pb.addUserDebugParameter("joint_angle5", -1, 1, 0)

    def read_debug_params(self, method):
        gripper_opening_length = pb.readUserDebugParameter(self.debug_gripper_open_length)
        # read the value of task parameter
        if method == 'ee':
            x = pb.readUserDebugParameter(self.debug_x)
            y = pb.readUserDebugParameter(self.debug_y)
            z = pb.readUserDebugParameter(self.debug_z)
            r = pb.readUserDebugParameter(self.debug_r)
            p = pb.readUserDebugParameter(self.debug_p)
            y = pb.readUserDebugParameter(self.debug_y)

            return x, y, z, r, p, y, gripper_opening_length
        elif method == 'joint':
            joint_angle0 = pb.readUserDebugParameter(self.debug_ja0) * np.pi
            joint_angle1 = pb.readUserDebugParameter(self.debug_ja1) * np.pi
            joint_angle2 = pb.readUserDebugParameter(self.debug_ja2) * np.pi
            joint_angle3 = pb.readUserDebugParameter(self.debug_ja3) * np.pi
            joint_angle4 = pb.readUserDebugParameter(self.debug_ja4) * np.pi
            joint_angle5 = pb.readUserDebugParameter(self.debug_ja5) * np.pi

            return joint_angle0, joint_angle1, joint_angle2, joint_angle3, joint_angle4, joint_angle5, gripper_opening_length
        else:
            raise NotImplementedError

    def load_objects(self, workspace_limits):
        self.object_ids = []
        def wait():
            for _ in range(30):
                pb.stepSimulation()
        
        for urdf_file in OBJECT_FILES:
            # drop_x = np.random.uniform(-0.15, 0.15)
            # drop_y = np.random.uniform(-0.15, 0.15)
            drop_x = (
                    (workspace_limits[0][1] - workspace_limits[0][0] - 0.2) * np.random.random_sample()
                    + workspace_limits[0][0]
                    + 0.1
                )
            drop_y = (
                    (workspace_limits[1][1] - workspace_limits[1][0] - 0.2) * np.random.random_sample()
                    + workspace_limits[1][0]
                    + 0.1
                )
            object_position = [drop_x, drop_y, 0.2]
            object_orientation = [
                2 * np.pi * np.random.random_sample(),
                2 * np.pi * np.random.random_sample(),
                2 * np.pi * np.random.random_sample(),
            ]
            object_id = pb.loadURDF(
                f'../vlg/assets/simplified_objects/{urdf_file}.urdf',
                object_position, 
                pb.getQuaternionFromEuler(object_orientation)
            )
            self.object_ids.append(object_id)
            self.wait_static()
    
    def wait_static(self, timeout=3):
        """Step simulator asynchronously until objects settle."""
        pb.stepSimulation()
        t0 = time.time()
        while (time.time() - t0) < timeout:
            if self.is_static:
                return True
            pb.stepSimulation()
        print(f"Warning: Wait static exceeded {timeout} second timeout. Skipping.")
        return False

    @property
    def is_static(self):
        """Return true if objects are no longer moving."""
        v = [
            np.linalg.norm(pb.getBaseVelocity(i, physicsClientId=self.physicsClient)[0])
            for i in self.object_ids
        ]
        return all(np.array(v) < 5e-3)

    def step_simulation(self):
        """
        Hook pb.stepSimulation()
        """
        pb.stepSimulation()
        if self.vis:
            time.sleep(self.SIMULATION_STEP_DELAY)
            self.p_bar.update(1)

    def step(self, action, control_method='joint'):
        """
        action: (x, y, z, roll, pitch, yaw, gripper_opening_length) for End Effector Position Control
                (a1, a2, a3, a4, a5, a6, a7, gripper_opening_length) for Joint Position Control
        control_method:  'end' for end effector position control
                         'joint' for joint position control
        """
        assert control_method in ('joint', 'end')
        self.robot.move_ee(action[:-1], control_method)
        self.robot.move_gripper(action[-1])
        for _ in range(120):  # Wait for a few steps
            self.step_simulation()

        reward = self.update_reward()
        done = True if reward == 1 else False
        # info = dict(box_opened=self.box_opened, btn_pressed=self.btn_pressed, box_closed=self.box_closed)
        return self.get_observation(), reward, done, {}#info

    def update_reward(self):
        # reward = 0
        # if not self.box_opened:
        #     if pb.getJointState(self.boxID, 1)[0] > 1.9:
        #         self.box_opened = True
        #         print('Box opened!')
        # elif not self.btn_pressed:
        #     if pb.getJointState(self.boxID, 0)[0] < - 0.02:
        #         self.btn_pressed = True
        #         print('Btn pressed!')
        # else:
        #     if pb.getJointState(self.boxID, 1)[0] < 0.1:
        #         print('Box closed!')
        #         self.box_closed = True
        #         reward = 1
        # return reward
        pass

    def get_observation(self):
        obs = dict()
        if isinstance(self.camera, Camera):
            rgb, depth, seg = self.camera.shot()
            obs.update(dict(rgb=rgb, depth=depth, seg=seg))
        else:
            assert self.camera is None
        obs.update(self.robot.get_joint_obs())

        return obs

    def reset(self):
        self.robot.reset()

        # self.robot.move_ee(npb.array([0, 0.8, 0.5, -0.2, -0.5, 0]) * 1, 'joint')
        # npb.array([0.5, -0.25, -0.5, -0.8, 0.5, 0])
        # for _ in range(120):
        #     self.step_simulation()

        # # raise up the end effector a little bit
        # tcp_pos = self.robot.get_tcp_pos()
        # tcp_pos[2] += 0.1
        # slave_pos = tcp_pos.copy()
        # self.robot.move_ee(slave_pos, 'end')
        # self.robot.close_gripper()
        # for _ in range(120):  # Wait for a few steps
        #     self.step_simulation()

        # grasping(self.robot)
        
        return self.get_observation()

    def close(self):
        pb.disconnect(self.physicsClient)
