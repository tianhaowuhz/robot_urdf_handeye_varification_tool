from urdfpy import URDF
from math import pi as PI
from termcolor import cprint
import numpy as np

def get_link_names(robot):
    link_names = []
    for link in robot.links:
        link_names.append(link.name)
    return link_names

def get_joint_names(robot):
    joint_names = []
    for joint in robot.joints:
        joint_names.append(joint.name)
    return joint_names

def get_actuated_joint_names(robot):
    joint_names = []
    for joint in robot.actuated_joints:
        joint_names.append(joint.name)
    return joint_names

def vis_robot_trajectory(robot, actuated_joint_names, traj):
    for idx, joint_traj in enumerate(traj):
        config = {}
        for idx, actuated_joint_name in enumerate(actuated_joint_names):
            config[actuated_joint_name] = joint_traj[idx]
        robot.animate(cfg_trajectory=config, loop_time=13)

if __name__ == "__main__":
    urdf_path = "assets/jakamini_leaphand.urdf"

    robot = URDF.load(urdf_path)
    
    cprint("Robot Links:", "green")
    link_names = get_link_names(robot)
    print(link_names)
    
    cprint("Robot Joints:", "green")
    joint_names = get_joint_names(robot)
    print(joint_names)

    cprint("Robot Actuated Joints:", "green")
    actuated_joint_names = get_actuated_joint_names(robot)
    print(actuated_joint_names)
    
    cprint("Show Robot with Zero Joint Configuration", "green")
    zero_joint_config = {}
    for actuated_joint_name in actuated_joint_names:
        zero_joint_config[actuated_joint_name] = 0
    robot.show(zero_joint_config)

    cprint("Show Robot Traj", "green")
    example_joint_traj = [0, 0.1, 0.2, 0.3, 0.4, 0.5]
    joint_traj_config = {}
    for actuated_joint_name in actuated_joint_names:
        joint_traj_config[actuated_joint_name] = example_joint_traj
    robot.animate(cfg_trajectory=joint_traj_config, loop_time=5)

    
    