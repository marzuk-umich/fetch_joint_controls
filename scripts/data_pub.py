#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Float32
from fetch_joint_controls.msg import optimizerMessage
import numpy as np
import time

def load_data(file_path):
    """
    Load the dataset from a file.
    Modify this function based on your file format.
    """

    # Assuming a NumPy file (can be adapted for CSV, JSON, etc.)
    data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_detailed_traj.npy', allow_pickle=True)    
    t = data.item().get('t')           # 1D array: time steps
    fail_flag = data.item().get('fail_flag')
    q = data.item().get('q')        # 2D array: joint positions (NxM)
    qd = data.item().get('qd')         # 2D array: joint velocities (NxM)
    qdd = data.item().get('qdd')       # 2D array: joint accelerations (NxM)
    return t,fail_flag, q, qd, qdd

def publish_joint_data(file_path):
    """
    Publishes joint data to ROS topics every 2 seconds.
    """
    rospy.init_node('optimizer_based_joint_data_publisher', anonymous=True)

    # Define publishers for joint data
    pub = rospy.Publisher('/optimizer_data', optimizerMessage, queue_size=10)

    # Load data from the file
    t,fail_flag, q, qd, qdd = load_data(file_path)


    rate = rospy.Rate(50)  # Publish every 2 seconds (0.5 Hz)
    msg = optimizerMessage()
    for i in range(len(t)):
        print(i)
        # if rospy.is_shutdown():
        #     break

        # # Prepare messages
        msg.time = Float32(data=t[i])
        # msg.fail_flag = Float32(data=fail_flag[i])
        msg.position = Float32MultiArray(data=q[i,:].tolist())
        msg.velocity = Float32MultiArray(data=qd[i,:].tolist())
        msg.acceleration = Float32MultiArray(data=qdd[i,:].tolist())
        
        # Publish data
        pub.publish(msg)
 

        # rospy.loginfo(f"Published data for time {time_step:.2f}s")
        
        rate.sleep()

if __name__ == "__main__":
    file_path =  np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_detailed_traj.npy', allow_pickle=True)

    try:
        publish_joint_data(file_path)
    except rospy.ROSInterruptException:
        pass
