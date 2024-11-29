from urdfpy import URDF
from math import pi as PI
import os

class RobotKDL(object):
    def __init__(self,
                 urdf_path: str = "jakamini_leaphand.urdf",
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

