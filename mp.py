import pybullet as p
import numpy as np
from robot import UR5Robotiq140

def pushing(robot: UR5Robotiq140, target_pos=None):
    # move the end effector to the target position
    # slave_pos = robot.get_slave_pos()
    # lower the end effector
    slave_pos = robot.get_tcp_pos()
    slave_pos[2] -= 0.3
    while True:
        robot.move_ee(slave_pos, 'end')
        p.stepSimulation()
        tcp_pos = robot.get_tcp_pos()
        # print((tcp_pos[2] - slave_pos[2]))
        if (tcp_pos[2] - slave_pos[2]) < 0.001:
            break
    
    # rotate the end effector
    tgt_joint_pos = robot.get_joint_pos()
    tgt_joint_pos[4] -= np.deg2rad(10)
    robot.move_ee(tgt_joint_pos, 'joint')

    # for _ in range(240):
    # while True:
        # robot.move_ee(tgt_joint_pos, 'joint')
        # p.stepSimulation()
        # print(robot.get_joint_pos()[4], tgt_joint_pos[4])
        # if np.abs(robot.get_joint_pos()[4] - tgt_joint_pos[4]) < 0.001:
        #     break

def grasping(robot: UR5Robotiq140, target_pos=None):
    # move the end effector to the target position
    # slave_pos = robot.get_slave_pos()

    # open the gripper
    robot.open_gripper()
    
    # lower the end effector, based on the depth data, target_pos[2]
    slave_pos = robot.get_tcp_pos()
    slave_pos[2] -= 0.3
    while True:
        robot.move_ee(slave_pos, 'end')
        p.stepSimulation()
        tcp_pos = robot.get_tcp_pos()
        # print((tcp_pos[2] - slave_pos[2]))
        if (tcp_pos[2] - slave_pos[2]) < 0.001:
            break

    # close the gripper
    robot.close_gripper()