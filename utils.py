import numpy as np
from enum import Enum

from scipy.spatial.transform import Rotation as R

DT = 1/10
LPF = 0.3

BOLT = 0
NUT = 1

class Objects(Enum):
    BOLT = 0
    NUT = 1

class State(Enum):
    IDLE = 0
    PICK = 1
    PICK_LOCK = 2
    GRIP = 3
    PLACE = 4
    PLACE_LOCK = 5
    UNGRIP = 6
    DONE = 7

# Camera to right base transformation
TRC1 = np.array([
        [1,  0,  0,  0.535],
        [0, -1,  0, -0.219],
        [0,  0, -1,  0.75 ],
        [0,  0,  0,  1.0  ]
    ])

TRC = np.array([
        [1,  0,  0,  0.251],
        [0, -1,  0, -0.211],
        [0,  0, -1,  0.75 ],
        [0,  0,  0,  1.0  ]
    ])

# Right base to left base transformation
TLR = np.array([
        [-1,  0, 0, 0.761],
        [ 0, -1, 0, 0.0  ],
        [ 0,  0, 1, 0.0  ],
        [ 0,  0, 0, 1    ]
    ])

TRL = np.linalg.inv(TLR)

def transform_LC(x, y, z, angle, o_type : int):
    pos_C = np.array([x, y, z, 1])

    # Transform Position
    pos_R = TRC @ pos_C
    pos_L = TLR @ pos_R

    if (o_type == BOLT):
        # Convert angle from radians to degrees
        angle_deg = (angle * 180) / np.pi

        # Map the 0-180 range to gripper-achievable angles
        # This creates a continuous mapping from bolt angle to gripper angle
        mapped_angle = 270 - angle_deg  
        # This will map:
        # 0° bolt -> 270° gripper
        # 90° bolt -> 180° gripper
        # 180° bolt -> 90° gripper

        euler = np.array([0, -180, mapped_angle])

    elif (o_type == NUT):
        # Convert angle from radians to degrees
        angle_deg = (angle * 180) / np.pi

        euler = np.array([0, -180, angle])

    else:
        raise Exception(f"Invalid Object ID {o_type}!")
    
    return np.concatenate([pos_L[:3], euler])

def euler_to_rot(euler_angles, order = 'xyz'):
    """
    coverts euler angles to rotation matrix
    """
    rot = R.from_euler(order, euler_angles, degrees=True)
    return rot

def map_yaw_to_joint_angle(yaw_angle):
    """
    Map desired yaw angle (0-180) to achievable joint angles considering joint limits
    Input yaw_angle is in degrees
    """
    # Normalize yaw angle to 0-180 range
    yaw = yaw_angle % 180
    
    if yaw <= 140:
        # If yaw is within the first range, use it directly
        return yaw
    else:
        # If yaw is in the unreachable zone or beyond,
        # map it to the upper range (211-360)
        # Formula: 360 - (180 - yaw)
        return 350 - (180 - yaw)
    
def calculate_error(current_rot, desired_rot):
    """
    caluclate orientation error in left base frame
    """
    orientation_error = (desired_rot*current_rot.inv()).as_rotvec()
    return np.array(orientation_error)
