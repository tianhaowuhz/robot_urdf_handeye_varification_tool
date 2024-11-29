from urdfpy import URDF
from math import pi as PI
from termcolor import cprint

if __name__ == "__main__":
    urdf_path = "assets/jakamini_leaphand.urdf"

    robot = URDF.load(urdf_path)
    
    cprint("Robot Links:", "green")
    link_names = []
    for link in robot.links:
        link_names.append(link.name)
    print(link_names)

    cprint("Robot Joints:", "green")
    config = {}
    joint_names = []
    for joint in robot.actuated_joints:
        joint_names.append(joint.name)
        config[joint.name] = 0.0
    print(joint_names)
    
    # custom joint value here
    config['joint_1'] = -PI/2
    config['joint_2'] = -PI*2.2/180
    config['joint_3'] = -PI*90/180
    config['joint_4'] = 0
    config['joint_5'] = -PI*90/180
    config['joint_6'] = -PI*90.5/180

    cprint("Robot Show:", "green")
    robot.show(config)