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
    cam_pos = [0, -0.5, 0.5]
    cam_tar = [0, 0, 0]
    cam_up_vector = [0, 0, 1]
    near = 0.2
    far = 100
    size = (640, 480)
    fov = 60

    # Create Camera instance
    camera = Camera(cam_pos, cam_tar, cam_up_vector, near, far, size, fov)

    robot = UR5Robotiq140((0, 0.5, 0), (0, 0, 0))
    env = ClutteredPushGrasp(robot, ycb_models, camera, vis=True)

    env.reset()
    # env.SIMULATION_STEP_DELAY = 0
    while True:
        # obs, reward, done, info = env.step(env.read_debug_parameter(), 'end')
        pass
        # print(obs, reward, done, info)


if __name__ == '__main__':
    user_control_demo()
