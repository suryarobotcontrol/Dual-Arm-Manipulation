import numpy as np
import math

def forward_kinematics(joint_angles_radians):
    if len(joint_angles_radians) != 6:
        raise ValueError("Expected 6 joint angles, but got {}".format(len(joint_angles_radians)))

    q1, q2, q3, q4, q5, q6 = joint_angles_radians

    T01 = np.array([[math.cos(q1), -math.sin(q1), 0, 0],
                    [math.sin(q1), math.cos(q1), 0, 0],
                    [0, 0, 1, 0.1283],
                    [0, 0, 0, 1]])

    T12 = np.array([[math.cos(q2), -math.sin(q2), 0, 0],
                    [0, 0, -1, -0.03],
                    [math.sin(q2), math.cos(q2), 0, 0.1150],
                    [0, 0, 0, 1]])

    T23 = np.array([[math.cos(q3), -math.sin(q3), 0, 0],
                    [-math.sin(q3), -math.cos(q3), 0, 0.280],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]])

    T34 = np.array([[math.cos(q4), -math.sin(q4), 0, 0],
                    [0, 0, -1, -0.140],
                    [math.sin(q4), math.cos(q4), 0, 0.020],
                    [0, 0, 0, 1]])

    T45 = np.array([[0, 0, 1, 0.0285],
                    [math.sin(q5), math.cos(q5), 0, 0],
                    [-math.cos(q5), math.sin(q5), 0, 0.105],
                    [0, 0, 0, 1]])
###Darshan found the mistake
    T56 = np.array([[0, 0, -1, -0.105],
                    [math.sin(q6), math.cos(q6), 0, 0],
                    [math.cos(q6), -math.sin(q6), 0, 0.0285],
                    [0, 0, 0, 1]])
     # naisrag'sss
    T6e = np.array([[0, -1, 0, 0],
                    [1, 0, 0, 0],
                    [0, 0, 1, 0.130],
                    [0, 0, 0, 1]])
    # T6e = np.array([[1, 0, 0, 0],
    #                 [0, 1, 0, 0],
    #                 [0, 0, 1, 0.130],
    #                 [0, 0, 0, 1]])
    matrices = [T01, T12, T23, T34, T45, T56, T6e]

    # end_effector_pose  = np.eye(4)
    T02 = np.eye(4)
    T03 = np.eye(4)
    T04 = np.eye(4)
    T05 = np.eye(4)
    T06 = np.eye(4)
    T0e = np.eye(4)

    T02 = np.dot(T01,T12)
    T03 = np.dot(T02,T23)
    T04 = np.dot(T03,T34)
    T05 = np.dot(T04,T45)
    T06 = np.dot(T05,T56)
    T0e = np.dot(T06,T6e)
    # for matrix in matrices:
    #     end_effector_pose = np.dot(end_effector_pose, matrix)

    return np.stack((T01,T02,T03,T04,T05,T06,T0e))


# joint_angles_degrees = [359.99, 343.931, 75.1387, 359.903, 299.981, 359.99]
# joint_angles_radians = [(angle * math.pi) / 180 for angle in joint_angles_degrees]

# position, orientation, total_transform = forward_kinematics(joint_angles_radians)

# print("End Effector Position:", position)
# print("End Effector Orientation (Rotation Matrix):\n", orientation)
# print("\nTotal Transformation Matrix:\n", total_transform)
