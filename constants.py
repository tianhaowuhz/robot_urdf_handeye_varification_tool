import numpy as np

'''
robot parameters
'''
URDF_PATH = "assets/jakamini_leaphand.urdf"

# important!!! 
# must set follow the joint order of your robot joint values from sim or real, they may be different from the urdf
ARM_JOINTS_ORDER = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
HAND_JOINTS_ORDER = ['1', '0', '2', '3', '5', '4', '6', '7', '9', '8', '10', '11', '12', '13', '14', '15']

axis_length = 0.03
BASE_AXIS = np.array([
            [0, 0, 0],         #
            [0, 0, axis_length],  #
            [0, axis_length, 0],  #
            [axis_length, 0, 0],  #
            
        ])
'''
cam parameters
'''
INTRINSICS = {
        "fx": 909.935,
        "fy": 908.769,
        "cx": 649.89550781,
        "cy": 350.27832031,
    }

CAM2BASE = np.array(
    [
    [0.01015681,  0.68000359, -0.73313843,  0.56647628],
    [ 0.9997567,  -0.02126201, -0.00587052, -0.32926053],
    [-0.01957997, -0.73290043, -0.6800541,   0.57872199],
    [ 0.,          0.,          0.,          1.        ],
    ]
)