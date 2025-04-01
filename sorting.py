import numpy as np

from .arm import DualArms, RoboticArm
from .utils import State

C_GOAL = 0.1
C_OBS = 0.01

PICK_GOAL_THRES = 0.125
PICK_GOAL_CNT = 15

GRIP_GOAL_THRES = 0.01
GRIP_GOAL_CNT_OPEN = 10
GRIP_GOAL_CNT_CLOSE = 30

class Algorithm:
    def __init__(self, arms : DualArms):
        self.arms = arms

        self.arm_L = self.arms.arm_L
        self.arm_R = self.arms.arm_R

    def upgrade_lock(self):
        # First upgrade the "Pick"
        if((self.arm_L.state == State.PICK) and (self.arm_R.state == State.PICK)):
            if(self.arm_L.goal_dist < self.arm_R.goal_dist):
                self.arm_L.state = State.PICK_LOCK
            else:
                self.arm_R.state = State.PICK_LOCK

        elif((self.arm_L.state == State.PICK) and (self.arm_R.state not in (State.PICK_LOCK, State.GRIP, State.PLACE_LOCK, State.UNGRIP))):
            self.arm_L.state = State.PICK_LOCK
    
        elif((self.arm_R.state == State.PICK) and (self.arm_L.state not in (State.PICK_LOCK, State.GRIP, State.PLACE_LOCK, State.UNGRIP))):
            self.arm_R.state = State.PICK_LOCK

        # Then upgrade the "Place"
        if((self.arm_L.state == State.PLACE) and (self.arm_R.state == State.PLACE)):
            if(self.arm_L.goal_dist < self.arm_R.goal_dist):
                self.arm_L.state = State.PLACE_LOCK
            else:
                self.arm_R.state = State.PLACE
        elif((self.arm_L.state == State.PLACE) and (self.arm_R.state not in (State.PICK_LOCK, State.GRIP, State.PLACE_LOCK, State.UNGRIP))):
            self.arm_L.state = State.PLACE_LOCK
    
        elif((self.arm_R.state == State.PLACE) and (self.arm_L.state not in (State.PICK_LOCK, State.GRIP, State.PLACE_LOCK, State.UNGRIP))):
            self.arm_R.state = State.PLACE_LOCK

    def check_arm_targets(self, arm : RoboticArm):
        # If ARM is IDLE, set the "pick" location as target
        if(arm.state == State.IDLE):
            # Check if we have a target available for "arm"
            if(arm.target_pick is not None):
                arm.c_target = arm.target_pick
                arm.state = State.PICK
            else:
                # We set "home" as target
                arm.c_target = arm.pos_home

            # Set APF Charges
            arm.apf.c_goal = C_GOAL
            arm.apf.c_obs = C_OBS

        elif(arm.state == State.PICK):
            arm.apf.c_goal = C_GOAL
            arm.apf.c_obs = C_OBS

        elif(arm.state == State.PICK_LOCK):
            arm.apf.c_goal = C_GOAL
            arm.apf.c_obs = 0

        elif(arm.state == State.GRIP):
            # Check if we can grip
            if(arm.goal_dist < GRIP_GOAL_THRES):
                arm.cnt_grip += 1

            if(arm.cnt_grip > GRIP_GOAL_CNT_OPEN):
                arm.gripper.close()
            else:
                arm.gripper.open()

            if(arm.cnt_grip > GRIP_GOAL_CNT_CLOSE):
                arm.cnt_grip = 0
                arm.state = State.PLACE
                arm.c_target = arm.target_drop

            arm.apf.c_goal = C_GOAL
            arm.apf.c_obs = 0

        elif(arm.state == State.PLACE):
            arm.apf.c_goal = C_GOAL
            arm.apf.c_obs = C_OBS

        elif(arm.state == State.PLACE_LOCK):
            arm.apf.c_goal = C_GOAL
            arm.apf.c_obs = 0

        elif(arm.state == State.UNGRIP):
            arm.apf.c_goal = C_GOAL
            arm.apf.c_obs = 0

            arm.cnt_grip += 1

            if(arm.cnt_grip > 10):
                arm.gripper.open()

            if(arm.cnt_grip > 30):
                arm.cnt_grip = 0
                arm.state = State.DONE


        elif(arm.state == State.DONE):
            arm.apf.c_goal = C_GOAL
            arm.apf.c_obs = C_OBS

            arm.c_target = arm.pos_home
            arm.target_drop = None
            arm.target_pick = None
            arm.state = State.IDLE

        else:
            raise KeyboardInterrupt("Unhandled State")
        

        if(arm.state in (State.PICK, State.PICK_LOCK)):
            if(arm.goal_dist < PICK_GOAL_THRES):
                arm.cnt_target += 1

            if(arm.cnt_target > PICK_GOAL_CNT):
                arm.state = State.GRIP
                arm.cnt_target = 0
    
        if(arm.state in (State.PLACE, State.PLACE_LOCK)):
            if(arm.goal_dist < PICK_GOAL_THRES):
                arm.cnt_target += 1

            if(arm.cnt_target > PICK_GOAL_CNT):
                arm.state = State.UNGRIP
                arm.cnt_target = 0

    def set_targets(self):
        self.check_arm_targets(self.arm_L)
        self.check_arm_targets(self.arm_R)

        self.upgrade_lock()
