from apf_sorting import *

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

from apf_sorting.utils import TLR, TRL


pose_drop_L = [0.218, -0.15, 0.2, 0, -180, 90]
pose_drop_R = [0.218, 0.15, 0.2, 0, -180, 270]

pose_drop_R_vec = np.array([*pose_drop_R[:3], 1])
pose_drop_R_t = TLR @ pose_drop_R_vec
pose_drop_R = [*pose_drop_R_t[:3], *pose_drop_R[3:]]

print(pose_drop_L, pose_drop_R)

pt = None
pt2 = None

def bolt_callback(msg : Float32MultiArray):
    global pt2
    n_objects = msg.layout.dim[0].size
    bolt_data = np.array(msg.data).reshape(n_objects, 6)
    
    if n_objects > 0:
        bolt = bolt_data[0]  # Using first bolt
        bolt_id = bolt[0]
        angle = bolt[1]
        size = bolt[2]
        x = bolt[3]
        y = bolt[4]
        z = bolt[5]
        
        # Transform to left base frame
        pose = transform_LC(x+0.03, y+0.03, z, angle, BOLT)
        #pose = transform_LC(x, y, z, angle, BOLT)
        pose[2] = -0.010
        if(pt is not None):
            if(np.linalg.norm(pt - pose) < 0.005):
                return None

        arms.bolt_target(pose, pose_drop_R)
        if(arms.arm_R.target_pick is not None):
            pt2 = arms.arm_R.target_pick


def nut_callback(msg : Float32MultiArray):
    global pt
    n_objects = msg.layout.dim[0].size
    nut_data = np.array(msg.data).reshape(n_objects, 6)

    if(n_objects > 0):
        nut = nut_data[-1]  # Using first nut
        nut_id = nut[0]
        angle = nut[1]
        size = nut[2]
        x = nut[3]
        y = nut[4]
        z = nut[5]

        # Transform to left base frame
        pose = transform_LC(x+0.02, y+0.03, z, angle, NUT)
        #pose = transform_LC(x, y, z, angle, NUT)
        pose[2] = -0.010

        if(pt is not None):
            if(np.linalg.norm(pt - pose) < 0.002):
                return None

        arms.nut_target(pose, pose_drop_L)
        if(arms.arm_L.target_pick is not None):
            pt = arms.arm_L.target_pick
        #print(pose, arms.arm_L.target_pick)

import csv


def record(arms : DualArms):
    end_eff_L_pos = arms.arm_L.cur_pos
    end_eff_L_rot = arms.arm_L.cur_rot.as_euler("XYZ")

    end_eff_R_pos = arms.arm_R.cur_pos
    end_eff_R_rot = arms.arm_R.cur_rot.as_euler("XYZ")

    force_att_L = arms.arm_L.apf.f_att
    force_rep_L = arms.arm_L.apf.f_rep
    force_tot_L = arms.arm_L.apf.f_final

    force_att_R = arms.arm_R.apf.f_att
    force_rep_R = arms.arm_R.apf.f_rep
    force_tot_R = arms.arm_R.apf.f_final

    goal_err_L = arms.arm_L.goal_dist
    goal_err_R = arms.arm_R.goal_dist

    cur_target_L = arms.arm_L.c_target # 6 tuple
    cur_target_R = arms.arm_R.c_target


    #print(cur_target_R, goal_err_R)

    data_c = [end_eff_L_pos, end_eff_L_rot, end_eff_R_pos, end_eff_R_rot, force_att_L, force_rep_L, force_tot_L, force_att_R, force_rep_R, force_tot_R, goal_err_L, goal_err_R, cur_target_L, cur_target_R]
    data_r = []

    for item in data_c:
        try:
            data_r += [*item]
        except:
            data_r += [item]

    csvw.writerow(data_r)



def main(arms : DualArms):
    # Initialize ROS node before the main loop
    rospy.init_node('dual_arm_controller', anonymous=True)
    
    # Create ROS subscribers
    sub = rospy.Subscriber('bolt_detections', Float32MultiArray, bolt_callback, queue_size=1, tcp_nodelay=True)
    sub2 = rospy.Subscriber('nut_detections', Float32MultiArray, nut_callback, queue_size=1, tcp_nodelay=True)
    
    rate = rospy.Rate(10)  # 10 Hz control loop
    print("Waiting for camera detections...")

    algorithm = Algorithm(arms)

    while not rospy.is_shutdown():
        # Setting Target according to Arm "States"
        algorithm.set_targets()
        
        # Updating Targets and Arm Positions
        ret = arms.update_targets()
        if(not ret):
            print("Failed to get forward kinematics, retrying...")
            rate.sleep()
            continue

        #print(arms.arm_R.cur_pos)

        # Check collisions and calculate min_dist
        min_dist = arms.check_collisions()

        # Calculate APF forces
        arms.calculate_apf_forces()

        print(arms.arm_L.state, arms.arm_R.state)

        # Recording current states and forces of arms
        record(arms)




if __name__ == "__main__":
    arms = DualArms()


    f_csv = open("/home/intelnuc/Desktop/collision_avoidance/Useful/saftey_lock/data.csv", "w")
    csvw = csv.writer(f_csv)

    header = "end_eff_L_pos, end_eff_L_rot, end_eff_R_pos, end_eff_R_rot, force_att_L, force_rep_L, force_tot_L, force_att_R, force_rep_R, force_tot_R, goal_err_L, goal_err_R, cur_target_L, cur_target_R"
    header = header.replace(",", "").split()

    header_n = []
    for item in header:
        if "goal" not in item:
            header_n += [item+"X", item+"Y", item+"Z"]
        else:
            header_n += [item]

    csvw.writerow(header_n)

    try:
        main(arms)
    except KeyboardInterrupt:
        print("\nStopping the robot.....")
    except rospy.ROSInterruptException:
        print("\nROS Node interrupted.....")

    f_csv.flush()
    f_csv.close()

    # Stop both arms
    arms.set_zero_vel()
    arms.connection.close()
    rospy.signal_shutdown("Program ended")
    print("Connection Closed!")
