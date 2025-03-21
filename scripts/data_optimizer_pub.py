#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray
from fetch_joint_controls.msg import traj_opt
import numpy as np
import time

def load_data(file_path):
    """
    Load the dataset from a file.
    Modify this function based on your file format.
    """
    # Assuming a NumPy file (can be adapted for CSV, JSON, etc.)
    data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_opt_info.npy', allow_pickle=True)    
    ka = data.item().get('ka')           # 1D array: time steps
    fail_flag = data.item().get('fail_flag')       # 2D array: joint accelerations (NxM)
    t = data.item().get('plan_time_curr')
    return ka, fail_flag, t


def publish_joint_data(file_path):
    """
    Publishes joint data to ROS topics every 2 seconds.
    """
    rospy.init_node('optimizer_based_joint_data_publisher', anonymous=True)

    # Define publishers for joint data
    pub = rospy.Publisher('/traj_opt_data', traj_opt, queue_size=1)

    # Load data from the file
    ka, fail_flag, t = load_data(file_path)


    # rate = rospy.Rate(0.1)  # Publish every 2 seconds (0.5 Hz)
    msg = traj_opt()
    for i in range(len(t)):
        
        msg.t = Float32(data=t[i])
        msg.ka = Float32MultiArray(data=ka[i,:].tolist())
        msg.fail_flag = Float32(data=fail_flag[i])
        # rate.sleep()
        print(msg.t)
        pub.publish(msg)
        time.sleep(4)

if __name__ == "__main__":
    file_path =  np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_opt_info.npy', allow_pickle=True)

    try:
        publish_joint_data(file_path)
    except rospy.ROSInterruptException:
        pass
