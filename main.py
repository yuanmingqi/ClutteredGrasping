import os

import numpy as np
import pybullet as p

from tqdm import tqdm
from env import ClutteredPushGrasp
from robot import Panda, UR5Robotiq85, UR5Robotiq140
from utilities import YCBModels, Camera
import time
import math


def user_control_demo():
    ycb_models = YCBModels(
        os.path.join('./data/ycb', '**', 'textured-decmp.obj'),
    )

    # Define camera parameters
    # RealSense D415
    cam_pos = (0, 0, 0.5)  # Camera position above the target
    cam_tar = (0, 0, 0)    # Camera target directly below the camera position
    cam_up_vector = (0, 1, 0)  # Up direction along the y-axis
    near = 0.01
    far = 10.0
    size = (480, 640)
    fov = 65.0

    # Create Camera instance
    camera = Camera(cam_pos, cam_tar, cam_up_vector, near, far, size, fov)

    robot = UR5Robotiq140((0, 0.5, 0), (0, 0, 0))
    env = ClutteredPushGrasp(robot, ycb_models, camera, vis=True)

    env.reset()
    # env.SIMULATION_STEP_DELAY = 0
    while True:
        obs, reward, done, info = env.step(env.read_debug_params('joint'), 'joint')
        pass
        # print(obs, reward, done, info)


if __name__ == '__main__':
    user_control_demo()
