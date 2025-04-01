import sys

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.Exceptions.KServerException import KServerException

from .utils import *
from .fwd_kin_gen3lite import forward_kinematics
from .line_collision import detect_coll
from .apf import *

class APF_Properties:
    c_goal = 0.0
    c_obs = 0.0

    f_att = 0.0
    f_rep = 0.0

    f_final = 0.0

class PID:
    def __init__(self, P, I, D, lim=50):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.I = 0

        self.lim = lim

        self.p_err = None

    def step(self, err : np.ndarray):
        P = self.Kp * (err)
        self.I += self.Ki * (err)
        if(self.p_err):
            D = self.Kd * (err - self.p_err)
        else:
            D = 0

        out = P + self.I + D
        out = np.clip(out, -self.lim, self.lim)

        return out

class Gripper:
    def __init__(self, base, base_cyclic) -> None:
        self.base = base
        self.base_cyclic = base_cyclic

    def open(self):
        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0.5
        finger.finger_identifier = 1
        finger.value = position

        # Send the GripperCommand
        self.base.SendGripperCommand(gripper_command)

    def close(self):
        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        position = 0.9
        finger.finger_identifier = 1
        finger.value = position

        # Send the GripperCommand
        self.base.SendGripperCommand(gripper_command)

class RoboticArm:
    def __init__(self, router, name="A"):
        self.router = router
        self.name = name

        # Initialize Connection
        self.init_connections()
        self.gripper = Gripper(self.base, self.base_cyclic)
        print(f"Robotic Arm {name} Registered!")

        # Create a base message to publish
        self.msg = Base_pb2.TwistCommand()
        self.msg.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        self.msg.duration = 0

        # Initialize Arm States
        self.pos_prev = None #np.zeros(3)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)

        self.t_vel = np.zeros(3)

        self.apf = APF_Properties()

        self.cnt_target = 0
        self.cnt_grip = 0

        self.state = State.IDLE
        self.base_transform = None
        self.z_offset = 0
        self.goal_dist = np.inf

        self.pos_home = np.zeros(6)

        # Set Gripper "OPEN"
        self.gripper.open()

        # Replacement for R00, R01,..., R0e
        self.links = np.zeros((1 + 6 + 1, 4, 4))

        self.prev_error = np.zeros(4)
        self.integral = np.zeros(4)

        self.target_pick = None
        self.target_drop = None

        self.c_target = None

        self.ang_control = PID(150/5, 0, 0.1, lim=50)

    def set_home(self, pos_home):
        self.pos_home = np.array(pos_home)

    def set_target_pick(self, target):
        self.target_pick = np.array(target)

    def reset_target_pick(self):
        self.target_pick = None

    def set_target_drop(self, target):
        self.target_drop = np.array(target)

    def reset_target_drop(self):
        self.target_drop = None

    def init_connections(self):
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router)

    def send_vel_command(self, v, w):
        twist = self.msg.twist
        twist.linear_x = v[0]
        twist.linear_y = v[1]
        twist.linear_z = v[2]
        twist.angular_x = w[0]
        twist.angular_y = w[1]
        twist.angular_z = w[2]

        self.base.SendTwistCommand(self.msg)

    def set_zero_vel(self):
        """"
        Set Target Velocities to "Zero"
        """
        self.send_vel_command(np.zeros(3), np.zeros(3))

    def update_velocities(self, pos : np.ndarray):
        if (self.pos_prev is not None):
            self.vel = (self.vel * LPF) + ((pos - self.pos_prev)*(1 - LPF)/DT)
        else:
            self.vel = np.zeros(3)
        self.pos_prev = pos

    def update_state_fk(self):
        try:
            joint_angles = self.base.GetMeasuredJointAngles().joint_angles
            joint_rad = [(angle.value * np.pi / 180) for angle in joint_angles]
            self.links[0] = np.eye(4)
            self.links[1:] = forward_kinematics(joint_rad)

            if(self.base_transform is not None):
                for i in range(self.links.shape[0]):
                    self.links[i] = np.dot(self.base_transform, self.links[i])

            pos = self.links[-1][:3, 3]
            rot = self.links[-1][:3,:3]   

            return rot, pos 
            
        except KServerException as ex:
            print("Forward kinematics error:", ex)
            return None
        
    def update_target(self):
        self.des_pos = self.c_target[:3]
        des_euler = self.c_target[3:]

        # Convert euler angles to rotation matrices
        self.des_rot = euler_to_rot(des_euler)

        # Get Forward Kinematics result
        ret = self.update_state_fk()
        if (not ret):
            return None
        cur_rot, self.cur_pos = ret

        self.cur_pos[2] += self.z_offset

        # Update velocities
        self.update_velocities(self.cur_pos)

        # Convert to rotation objects
        self.cur_rot = R.from_matrix(cur_rot)

        # Calculate Distance from Goal
        self.goal_dist = np.linalg.norm(self.cur_pos - self.des_pos)

        # Calculate Rot Error
        err_rot = calculate_error(self.cur_rot, self.des_rot)
        self.ang_vel = self.ang_control.step(err_rot)

        return 1 # no error

class DualArms:
    router_R, router_L = None, None

    def __init__(self):
        self.init_connections()

        self.arm_R = RoboticArm(self.router_R, name="R")
        self.arm_L = RoboticArm(self.router_L, name="L")

        self.arm_R.base_transform = TLR
        self.arm_R.z_offset = 0.006
        self.arm_L.z_offset = 0.001

        pose_r_home = [0.621, -0.193, 0.448, 90.653, -0.974, -30. ]
        pose_l_home = [0.439,  0.193, 0.448, 90.653, -0.974,  150.]

        pose_home_L = [0.165, -0.07, 0.2, 0, -180, 90]
        pose_home_R = [0.165, 0.07, 0.2, 0, -180, 90]

        pose_home_R_vec = np.array([*pose_home_R[:3], 1])
        pose_home_R_t = TLR @ pose_home_R_vec
        pose_home_R = [*pose_home_R_t[:3], *pose_home_R[3:]]

        # self.arm_R.set_home(pose_r_home)
        # self.arm_L.set_home(pose_l_home)
        self.arm_R.set_home(pose_home_R)
        self.arm_L.set_home(pose_home_L)

        self.min_dist = np.inf

    def init_connections(self):
        # Starting connections
        sys.path.append('/home/intelnuc/Desktop/api_ws/src/api_pkg')
        from utilities import DeviceConnections

        self.connection = DeviceConnections()
        self.router_R, self.router_L = self.connection.routersTCP()
        print("Routers Initialized")

    def set_vel(self, vR, wR, vL, wL):
        """
        Set Velocities for both arms
        vR, wR: right arm velocities in left base frame
        vL, wL: left arm velocities in left base frame
        """
        #Transformation of right arm velocities from left base to right base frame
        rot_R = TRL[:3, :3]
        vR = rot_R @ vR
        wR = rot_R @ wR

        self.arm_R.send_vel_command(vR, wR)
        self.arm_L.send_vel_command(vL, wL)

    def set_zero_vel(self):
        """
        Set Target Velocities to "Zero"
        (for both arms)
        """
        self.arm_R.set_zero_vel()
        self.arm_L.set_zero_vel()

    def update_targets(self):
        ret1 = self.arm_L.update_target()
        ret2 = self.arm_R.update_target()

        if((not ret1) or (not ret2)):
            return None
        else:
            return 1
        
    def check_collisions(self):
        links_L = self.arm_L.links[:, :3, 3]
        links_R = self.arm_R.links[:, :3, 3]

        coll_matrix = np.eye(4)
        for i in range(7):
            for j in range(7):
                if(i>2 and j>2):
                    collision = detect_coll(links_L[i], links_L[i+1], links_R[j], links_R[j+1])
                    coll_matrix[i-3, j-3] = collision
        self.min_dist = np.min(coll_matrix)

        return self.min_dist

    def calculate_apf_forces(self):
        self.grip_L = (self.arm_L.state == State.GRIP)
        self.grip_R = (self.arm_R.state == State.GRIP)

        self.targeting_L = (self.arm_L.state in (State.PICK, State.PICK_LOCK))
        self.targeting_R = (self.arm_R.state in (State.PICK, State.PICK_LOCK))

        self.gripped_L = (self.arm_L.state in (State.PLACE, State.PLACE_LOCK))
        self.gripped_R = (self.arm_R.state in (State.PLACE, State.PLACE_LOCK))

        L_force, L_dist_error, L_forces = apf(self.arm_L.cur_pos, self.arm_L.des_pos, 
                                    self.arm_R.cur_pos, 
                                    self.arm_L.pos_home[:3],
                                    self.min_dist, 
                                    self.arm_L.vel, 
                                    charge_goal=self.arm_L.apf.c_goal,
                                    charge_ob=self.arm_L.apf.c_obs,
                                    is_gripping=self.grip_L,
                                    is_targeting=self.targeting_L,
                                    has_gripped=self.gripped_L)
        
        R_force, R_dist_error, R_forces = apf(self.arm_R.cur_pos, self.arm_R.des_pos, 
                                    self.arm_L.cur_pos, 
                                    self.arm_R.pos_home[:3],
                                    self.min_dist, 
                                    self.arm_R.vel, 
                                    charge_goal=self.arm_R.apf.c_goal,
                                    charge_ob=self.arm_R.apf.c_obs,
                                    is_gripping=self.grip_R,
                                    is_targeting=self.targeting_R,
                                    has_gripped=self.gripped_R)       

        L_vel = L_force / 1000
        R_vel = R_force / 1000

        self.arm_L.apf.f_final = L_force
        self.arm_L.apf.f_att = L_forces[0]
        self.arm_L.apf.f_rep = L_forces[1]

        self.arm_R.apf.f_final = R_force
        self.arm_R.apf.f_att = R_forces[0]
        self.arm_R.apf.f_rep = R_forces[1]

        # self.arm_L.t_vel += L_vel
        # self.arm_R.t_vel += R_vel

        self.set_vel(R_vel, self.arm_R.ang_vel, L_vel, self.arm_L.ang_vel)
        #self.set_vel(self.arm_R.t_vel, self.arm_R.ang_vel, self.arm_L.t_vel, self.arm_L.ang_vel)

    def nut_target(self, pose, drop = None):
        if(self.arm_L.state == State.IDLE):
            self.arm_L.target_pick = pose
            if(drop is None):
                self.arm_L.target_drop = self.arm_L.pos_home
            else:
                self.arm_L.target_drop = drop

    def bolt_target(self, pose, drop = None):
        if(self.arm_R.state == State.IDLE):
            self.arm_R.target_pick = pose
            if(drop is None):
                self.arm_R.target_drop = self.arm_R.pos_home
            else:
                self.arm_R.target_drop = drop
