import numpy as np
import cv2
import os

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

def draw_axes(image, arm_joint_state, hand_joint_state, image_name, target_path, robot_kdl, intrinsics, cam2base, base_axis):
    os.makedirs(target_path, exist_ok=True)
    
    joint_values = {}
    joint_values['arm'] = arm_joint_state
    joint_values['hand'] = hand_joint_state

    for link in robot_kdl.robot_links_info.keys():
        
        link_pose = robot_kdl.forward_kinematics(joint_values, link_name=link)
        
        rotation = link_pose[:3,:3]
        translation = link_pose[:3,3]
        axes_world = (rotation @ base_axis.T).T + translation

        pixels = project_point2image(axes_world, intrinsics, cam2base)

        pixels_origin = pixels[0]
        pixels_x_axis = pixels[1]
        pixels_y_axis = pixels[2]
        pixels_z_axis = pixels[3]

        # image = draw_point(image, pixels)
        image = draw_arrow(image, pixels_origin, pixels_x_axis, axis_type='x')
        image = draw_arrow(image, pixels_origin, pixels_y_axis, axis_type='y')
        image = draw_arrow(image, pixels_origin, pixels_z_axis, axis_type='z')
        
        cv2.imwrite(os.path.join(target_path, image_name), image)