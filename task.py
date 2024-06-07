import pybullet as pb
import numpy as np
import time

from constants import *

def load_bricks():
    # Load the brick tower
    cube_size = 0.05 # size of the cube
    gap = 0.01 # gap between cubes
    positions = [
        (-(cube_size + gap), -(cube_size + gap), 0),
        (0, -(cube_size + gap), 0),
        (cube_size + gap, -(cube_size + gap), 0),
        (-(cube_size + gap), 0, 0),
        (0, 0, 0),
        (cube_size + gap, 0, 0),
        (-(cube_size + gap), cube_size + gap, 0),
        (0, cube_size + gap, 0),
        (cube_size + gap, cube_size + gap, 0),
        # Add bricks on top of the cross bricks
        (0, -(cube_size + gap), cube_size),
        (-(cube_size + gap), 0, cube_size),
        (0, 0, cube_size),
        (cube_size + gap, 0, cube_size),
        (0, cube_size + gap, cube_size),
        # Add bricks on top of the center bricks
        (0, 0, 2 * cube_size)
    ]

    brick_ids = [pb.loadURDF("./urdf/objects/cube_small.urdf", basePosition=pos) for pos in positions]

    return brick_ids

def draw_boundaries():
    # Define the size of the square (half length)
    half_length = 0.3  # 30 cm
    # Define the vertices of the square
    vertices = [
        [-half_length, -half_length, 0],
        [half_length, -half_length, 0],
        [half_length, half_length, 0],
        [-half_length, half_length, 0]
    ]
    # Draw the lines to form the square
    for i in range(4):
        start_point = vertices[i]
        end_point = vertices[(i + 1) % 4]
        pb.addUserDebugLine(start_point, end_point, lineColorRGB=[0, 0, 0], lineWidth=5.0)

def load_objects(self, workspace_limits):
    object_ids = []
        
    for urdf_file in OBJECT_FILES:
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
        object_ids.append(object_id)
    
    return object_ids