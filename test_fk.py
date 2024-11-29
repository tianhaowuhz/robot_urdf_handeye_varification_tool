import torch
import numpy as np
import cv2
import os

from project_pcl import project_point2image, draw_point, draw_arrow, draw_axes, draw_tactile

intrinsics = {
        "fx": 909.935,
        "fy": 908.769,
        "cx": 649.89550781,
        "cy": 350.27832031,
    }

cam2base = np.array(
    [
    [0.01015681,  0.68000359, -0.73313843,  0.56647628],
    [ 0.9997567,  -0.02126201, -0.00587052, -0.32926053],
    [-0.01957997, -0.73290043, -0.6800541,   0.57872199],
    [ 0.,          0.,          0.,          1.        ],
    ]
)

# if __name__ == "__main__":
#     from jaka_kdl import JakaKDL
#     jaka_kdl = JakaKDL()

#     traj = torch.load("data/demonstration_1.pth")
#     joint_state = traj['arm_abs_joint'][34].cpu().numpy()
#     joint_pose = jaka_kdl.forward_kinematics(joint_state, full_kinematics=True)
#     ee_pose = joint_pose[-1]

#     points = np.array([ee_pose[:3,3]])

#     image = cv2.imread('data/35.PNG')

#     pixels = project_point2image(points, intrinsics, cam2base)
#     image = draw_point(image, pixels)

#     cv2.imshow('Image with Points', image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

if __name__ == "__main__":
    from jaka_kdl_urdfpy import JakaLeapKDL

    robot_kdl = JakaLeapKDL()

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
        # arm_joint_state = np.array([-1.5707487,   0.24192421, -1.4037328,   0.02739489, -1.8208425,  -2.1729174])
        # hand_joint_state = np.array([-0.01073527,  0.02301216,  0.00613856,  0.00000262,  0.00000262,  0.02147818,
        # 0.00613856,  0.23470163, -0.00613332,  0.02301216,  0.00307059,  0.00000262,
        # 0.02301216, -0.00306535, -0.00306535, -0.00153136])

        draw_axes(image , arm_joint_state, hand_joint_state, robot_kdl, intrinsics, cam2base, image_name, target_path=target_path)
        
        # tactile_link_list = ['thumb_fingertip', 'fingertip', 'dip', 'fingertip_2', 'dip_2', 'fingertip_3', 'dip_3']
        # draw_tactile(image, arm_joint_state, hand_joint_state, robot_kdl, intrinsics, cam2base, image_name, target_path=target_path, tactile_link_list=tactile_link_list)

        # link_pose = robot_kdl.forward_kinematics(joint_values, link_name='thumb_tip_head')

        # points = np.array([link_pose[:3,3]])

        # image = cv2.imread('data/init.jpg')

        # pixels = project_point2image(points, intrinsics, cam2base)
        # image = draw_point(image, pixels)

        # cv2.imwrite('test.jpg', image)
        # cv2.imshow('Image with Points', image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
