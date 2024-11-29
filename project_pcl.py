import numpy as np
import torch
import cv2
import os
from constants import PAXINI_PULP_COORD, PAXINI_TIP_COORD, PAXINI_THUMB_PULP_COORD, PAXINI_THUMB_TIP_COORD

def project_point2image(points2base, intrinsics, extrinsics):
    # intrinsics matrix
    K = np.array([[intrinsics["fx"], 0, intrinsics["cx"]],
                [0, intrinsics["fy"], intrinsics["cy"]],
                [0, 0, 1]])

    # pcl to homogeneous coordinates
    points2base = np.hstack((points2base, np.ones((points2base.shape[0], 1))))

    # point from world to camera, here extrinsics is cam2base, inv: base2cam@point2base=point2cam
    points2cam = np.linalg.inv(extrinsics) @ points2base.T

    # cam2rgb
    points2image = K @ points2cam[:3, :]

    # normalize
    points2image = points2image[:2, :] / points2image[2, :]

    # pixel coord (N x 2)
    pixels = points2image.T
    return pixels

def draw_point(image, pixels):
    for point in pixels:
        x, y = int(point[0]), int(point[1])
        cv2.circle(image, (x, y), radius=2, color=(0, 0, 255), thickness=-1)
    return image

def draw_arrow(image, point1, point2, axis_type=None):
    if axis_type == 'x':
        color = (255, 0, 0)
    elif axis_type == 'y':
        color = (0, 255, 0)
    elif axis_type == 'z':
        color = (0, 0, 255)
    
    point1 = (int(point1[0]), int(point1[1]))
    point2 = (int(point2[0]), int(point2[1]))

    cv2.arrowedLine(image, tuple(point1), tuple(point2), color, 2)
    return image

def draw_axes(image, arm_joint_state, hand_joint_state, robot_kdl, intrinsics, cam2base, image_name, target_path='data/camera_1_color_image_with_axes', axis_length = 0.1):
    os.makedirs(target_path, exist_ok=True)

    joint_values = {}
    joint_values['arm'] = arm_joint_state
    joint_values['hand'] = hand_joint_state

    for link in robot_kdl.robot_links_info.keys():
        # if link != 'base' and link != 'palm_lower':
        #     continue
        local_axes = np.array([
            [0, 0, 0],         #
            [0, 0, axis_length],  #
            [0, axis_length, 0],  #
            [axis_length, 0, 0],  #
            
        ])
        if link == 'fingertip':
            tactile_ori = [-0.015529999999999995, -0.013980000000000006, 0.01592]
            local_axes = np.append(local_axes, [tactile_ori], axis=0)
            tactile_points = tactile_ori + PAXINI_TIP_COORD
            local_axes = np.append(local_axes, tactile_points, axis=0)
        elif link == 'dip':
            tactile_ori = [-0.0158, -0.012369999999999999, 0.0159]
            local_axes = np.append(local_axes, [tactile_ori], axis=0)
            tactile_points = tactile_ori + PAXINI_PULP_COORD
            local_axes = np.append(local_axes, tactile_points, axis=0)
        elif link == 'thumb_fingertip':
            tactile_ori = [-0.016320000000000005, 0.00010000000000001674, -0.0159]
            local_axes = np.append(local_axes, [tactile_ori], axis=0)
            tactile_ori_2 = [-0.015520000000000006, -0.029579999999999995, -0.0159]
            local_axes = np.append(local_axes, [tactile_ori_2], axis=0)
        link_pose = robot_kdl.forward_kinematics(joint_values, link_name=link)
        
        rotation = link_pose[:3,:3]
        translation = link_pose[:3,3]
        # 将局部坐标轴转换到世界坐标系
        axes_world = (rotation @ local_axes.T).T + translation

        pixels = project_point2image(axes_world, intrinsics, cam2base)

        pixels_origin = pixels[0]
        pixels_x_axis = pixels[1]
        pixels_y_axis = pixels[2]
        pixels_z_axis = pixels[3]
        if link == 'fingertip' or link == 'dip':
            pixels_tactile_origin = pixels[4]
        elif link == 'thumb_fingertip':
            pixels_tactile_origin = pixels[4]
            pixels_tactile_origin_2 = pixels[5]

        # image = draw_point(image, pixels)
        image = draw_arrow(image, pixels_origin, pixels_x_axis, axis_type='x')
        image = draw_arrow(image, pixels_origin, pixels_y_axis, axis_type='y')
        image = draw_arrow(image, pixels_origin, pixels_z_axis, axis_type='z')
        if link == 'fingertip' or link == 'dip':
            print(link)
            for pixel in pixels[5:,:]:
                image = draw_point(image, [pixel])
            # image = draw_point(image, [pixels_tactile_origin])
        elif link == 'thumb_fingertip':
            image = draw_point(image, [pixels_tactile_origin])
            image = draw_point(image, [pixels_tactile_origin_2])
        
        cv2.imwrite(os.path.join(target_path, image_name), image)

def draw_tactile(image, arm_joint_state, hand_joint_state, robot_kdl, intrinsics, cam2base, image_name, target_path='data/camera_1_color_image_with_axes', tactile_link_list=[]):
    os.makedirs(target_path, exist_ok=True)

    joint_values = {}
    joint_values['arm'] = arm_joint_state
    joint_values['hand'] = hand_joint_state

    for link in robot_kdl.robot_links_info.keys():
        if link in tactile_link_list:
            import time
            st = time.time()
            link_pose = robot_kdl.forward_kinematics(joint_values, link_name=link)
            print('forward kinematics time:', time.time()-st)
            if 'thumb' in link:
                if 'fingertip' in link:
                    tactile_pulp_ori = [-0.016320000000000005, 0.00010000000000001674, -0.0159]
                    tactile_tip_ori = [-0.015520000000000006, -0.029579999999999995, -0.0159]
                    tactile_pulp_real_points = get_tactile_points(tactile_pulp_ori, PAXINI_THUMB_PULP_COORD, link_pose)
                    tactile_tip_real_points = get_tactile_points(tactile_tip_ori, PAXINI_THUMB_TIP_COORD, link_pose)
                    draw_tactile2img(image, tactile_pulp_real_points, intrinsics, cam2base)
                    draw_tactile2img(image, tactile_tip_real_points, intrinsics, cam2base)
            elif 'fingertip' in link:
                tactile_tip_ori = [-0.015529999999999995, -0.013980000000000006, 0.01592]
                tactile_tip_real_points = get_tactile_points(tactile_tip_ori, PAXINI_TIP_COORD, link_pose)
                draw_tactile2img(image, tactile_tip_real_points, intrinsics, cam2base)
            elif 'dip' in link:
                tactile_pulp_ori = [-0.0158, -0.012369999999999999, 0.0159]
                tactile_pulp_real_points = get_tactile_points(tactile_pulp_ori, PAXINI_PULP_COORD, link_pose)
                draw_tactile2img(image, tactile_pulp_real_points, intrinsics, cam2base)
    
    cv2.imwrite(os.path.join(target_path, image_name), image)

def draw_tactile2img(image, real_points, intrinsics, cam2base):    
    # world to pixel
    pixels = project_point2image(real_points, intrinsics, cam2base)

    pixels = transform_point_to_processed_image(pixels)
    # draw all points
    for pixel in pixels:
        image = draw_point(image, [pixel])

    return image

def transform_point_to_processed_image(pixels):
    # Original image dimensions
    original_width, original_height = 1280, 720

    # Crop region [left, top, right, bottom]
    crop_left, crop_top, crop_right, crop_bottom = 450, 0, 1000, 550

    # Dimensions of the cropped image
    cropped_width = crop_right - crop_left
    cropped_height = crop_bottom - crop_top

    # New image dimensions
    new_width, new_height = 224, 224

    # Original point coordinates
    original_x, original_y = pixels[:,0], pixels[:,1]

    # Calculate the coordinates of the point in the cropped image
    cropped_x = original_x - crop_left
    cropped_y = original_y - crop_top

    # Scaling factors
    scale_x = new_width / cropped_width
    scale_y = new_height / cropped_height

    # Calculate the coordinates of the point in the new scaled image
    new_x = cropped_x * scale_x
    new_y = cropped_y * scale_y

    # convert to int
    new_x = new_x.astype(int)
    new_y = new_y.astype(int)

    new_pixels = np.stack((new_x, new_y), axis=1)
    return new_pixels


def get_tactile_points(tactile_ori, tactile_points, link_pose):
    local_points = tactile_ori + tactile_points

    # local to world
    rotation = link_pose[:3,:3]
    translation = link_pose[:3,3]
    real_points = (rotation @ local_points.T).T + translation

    return real_points
    
if __name__ == "__main__":
    intrinsics = {
        "fx": 909.935,
        "fy": 908.769,
        "cx": 649.89550781,
        "cy": 350.27832031,
    }

    cam2base = np.array(
        [
            [ 0.00195837,  0.6677521,   -0.74438115,  0.6026231 ],
            [ 0.99993265, -0.00982321, -0.00618128, -0.32561635],
            [-0.01143978, -0.74431891, -0.66772636,  0.58901597],
            [ 0.,          0.,          0.,          1.        ],
        ]
    )

    traj = torch.load("data/demonstration_1.pth")
    points2world = np.array([traj['arm_ee_pose'][34,:3].cpu().numpy()/1000])

    image = cv2.imread('data/35.PNG')

    pixels = project_point2image(points2world, intrinsics, cam2base)
    image = draw_point(image, pixels)

    cv2.imshow('Image with Points', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
