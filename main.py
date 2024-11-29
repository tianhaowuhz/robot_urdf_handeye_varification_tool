import numpy as np
import cv2
import os

from project2image import project_point2image, draw_point, draw_arrow, draw_axes
from constants import INTRINSICS, CAM2BASE, URDF_PATH, ARM_JOINTS_ORDER, HAND_JOINTS_ORDER, BASE_AXIS
from robot_kdl_urdfpy import RobotKDL

if __name__ == "__main__":
    # load robot model
    robot_kdl = RobotKDL(urdf_path=URDF_PATH, arm_joints_name_order=ARM_JOINTS_ORDER, hand_joints_name_order=HAND_JOINTS_ORDER)

    # load trajectory and image
    traj = np.load("example_data/traj.npy", allow_pickle=True).item()
    source_path = 'example_data/camera_1_color_image'

    # create target path
    target_path = source_path + '_axes'
    os.makedirs(target_path, exist_ok=True)
    
    # sort image names
    image_names = os.listdir(source_path)
    image_names.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))

    # draw axes
    for image_name in image_names:
        # load image
        id = int(image_name.split('.')[0]) - 1
        image = cv2.imread(os.path.join(source_path, image_name))

        # get joint values
        arm_joint_state = traj['arm_abs_joint'][id]
        hand_joint_state = traj['hand_abs_joint'][id]
        joint_values = {}
        joint_values['arm'] = arm_joint_state
        joint_values['hand'] = hand_joint_state

        # draw axes
        draw_axes(image, joint_values, image_name, target_path, robot_kdl, INTRINSICS, CAM2BASE, BASE_AXIS)
        