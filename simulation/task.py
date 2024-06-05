import pybullet as p

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

    brick_ids = [p.loadURDF("./urdf/objects/cube_small.urdf", basePosition=pos) for pos in positions]

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
        p.addUserDebugLine(start_point, end_point, lineColorRGB=[0, 0, 0], lineWidth=5.0)