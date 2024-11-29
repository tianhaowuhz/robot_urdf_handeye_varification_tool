import torch
import numpy as np
import cv2
import os

from project_pcl import project_point2image, draw_point, draw_arrow, draw_axes
from constants import INTRINSICS, CAM2BASE, URDF_PATH, ARM_JOINTS_ORDER, HAND_JOINTS_ORDER, BASE_AXIS
from robot_kdl_urdfpy import RobotKDL

if __name__ == "__main__":
    robot_kdl = RobotKDL(urdf_path=URDF_PATH, arm_joints_name_order=ARM_JOINTS_ORDER, hand_joints_name_order=HAND_JOINTS_ORDER)

    traj = torch.load("data/demonstration_1.pth")
    
    source_path = 'data/camera_1_color_image'
    target_path = source_path + '_axes'
    
    image_names = os.listdir(source_path)
    image_names.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))
    for image_name in image_names:
        id = int(image_name.split('.')[0]) - 1
        image = cv2.imread(os.path.join(source_path, image_name))

        arm_joint_state = traj['arm_abs_joint'][id].cpu().numpy()
        hand_joint_state = traj['hand_abs_joint'][id].cpu().numpy()
        draw_axes(image, arm_joint_state, hand_joint_state, image_name, target_path, robot_kdl, INTRINSICS, CAM2BASE, BASE_AXIS)
        