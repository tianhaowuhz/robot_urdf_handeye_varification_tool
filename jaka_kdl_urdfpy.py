from urdfpy import URDF
from math import pi as PI
import os

# robot = URDF.load('jakamini_leaphand.urdf')

# for link in robot.links:
#     print(link.name)

# config = {}
# for joint in robot.actuated_joints:
#     print(joint.name)
#     config[joint.name] = 0.0

# config['joint_1'] = -PI/2
# config['joint_2'] = -PI*2.2/180
# config['joint_3'] = -PI*90/180
# config['joint_4'] = 0
# config['joint_5'] = -PI*90/180
# config['joint_6'] = -PI*90.5/180

# # robot.show(config)


# import torch
# traj = torch.load("demonstration_1.pth")

# for idx, joint_traj in enumerate(traj['arm_abs_joint'][0]):
#     joint_name = f"joint_{idx+1}"
#     # copy last element of joint_traj for 10 steps
#     # joint_traj = torch.cat([joint_traj, joint_traj[-1].repeat(10)])
#     config[joint_name] = joint_traj.cpu().numpy()
# fk = robot.link_fk(config)

# arm_joint_traj = traj['arm_abs_joint'].permute(1,0)
# hand_joint_traj = traj['hand_abs_joint'].permute(1,0)

# for idx, joint_traj in enumerate(arm_joint_traj):
#     joint_name = f"joint_{idx+1}"
#     # copy last element of joint_traj for 10 steps
#     # joint_traj = torch.cat([joint_traj, joint_traj[-1].repeat(10)])
#     config[joint_name] = joint_traj.cpu().numpy()

# for idx, joint_traj in enumerate(hand_joint_traj):
#     if idx == 0:
#         joint_name = "1"
#     elif idx == 1:
#         joint_name = "0"
#     elif idx == 4:
#         joint_name = "5"
#     elif idx == 5:
#         joint_name = "4"
#     elif idx == 8:
#         joint_name = "9"
#     elif idx == 9:
#         joint_name = "8"
#     else:
#         joint_name = f"{idx}"
#     # joint_traj = torch.cat([joint_traj, joint_traj[-1].repeat(10)])
#     config[joint_name] = joint_traj.cpu().numpy()

# robot.animate(cfg_trajectory=config, loop_time=13)
# print(traj.shape)

class JakaLeapKDL(object):
    def __init__(self,
                 urdf_path: str = "jakamini_leaphand_ori.urdf",
                 arm_joints: list = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
                 hand_joints: list = ['1', '0', '2', '3', '5', '4', '6', '7', '9', '8', '10', '11', '12', '13', '14', '15']):
        
        # Getting the URDF path
        self.robot = URDF.load(urdf_path)
        print('finished loading robot urdf')
        
        self.robot_links_info = {}
        for link in self.robot.links:
            print(link.name)
            self.robot_links_info[link.name] = link

        self.config = {}
        for joint in self.robot.actuated_joints:
            print(f'assigning joint values:{joint.name} to 0.0')
            self.config[joint.name] = 0.0
        
        self.arm_joint_names = arm_joints
        self.hand_joint_names = hand_joints
            
    def forward_kinematics(self, joint_values, link_name=None):
        arm_joint_values = joint_values['arm']
        hand_joint_values = joint_values['hand']
        
        for idx, joint_name in enumerate(self.arm_joint_names):
            self.config[joint_name] = arm_joint_values[idx]
        
        for idx, joint_name in enumerate(self.hand_joint_names):
            self.config[joint_name] = hand_joint_values[idx]

        fk = self.robot.link_fk(self.config)

        if link_name is not None:
            return fk[self.robot_links_info[link_name]]
        else:
            return fk

