#!/bin/env python3


import rospy
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Point
import numpy as np
# Initialize the ROS node
rospy.init_node('trajectory_data_publisher')

# Create publishers for different topics
ka_pub = rospy.Publisher('joint_accelerations', Float64MultiArray, queue_size=10)
fail_flag_pub = rospy.Publisher('fail_flag', Float32, queue_size=10)
time_pub = rospy.Publisher('plan_time_curr', Float32, queue_size=10)
obs_pub = rospy.Publisher('obstacle_position', Point, queue_size=10)
obs_size_pub = rospy.Publisher('obstacle_size', Point, queue_size=10)

# Your data
data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_opt_info.npy', allow_pickle=True)
rate = rospy.Rate(2)  # Set rate to publish data (e.g., 2 Hz)

# Publish data for each time step
# data.item().get('ka')
for i in range(len(data.item().get('plan_time_curr'))):
    # Prepare joint accelerations data
    ka_msg = Float64MultiArray()
    ka_msg.data = data.item().get('ka')[i]
    ka_pub.publish(ka_msg)

    # Publish fail_flag for the current time step
    fail_flag_msg = Float32()
    fail_flag_msg.data = data.item().get('fail_flag')[i]
    fail_flag_pub.publish(fail_flag_msg)

    # Publish the current time
    time_msg = Float32()
    time_msg.data = data.item().get('plan_time_curr')[i]
    time_pub.publish(time_msg)

    # Publish obstacle data (if it doesn't change, publish once)
    if i == 0:  # Only publish once
        obs_msg = Point(*data.item().get('obs_pos')[0])
        obs_pub.publish(obs_msg)

        obs_size_msg = Point(*data.item().get('obs_size')[0])
        obs_size_pub.publish(obs_size_msg)

    rate.sleep()
