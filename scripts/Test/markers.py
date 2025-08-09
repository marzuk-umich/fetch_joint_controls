#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
import tf
import os
import numpy as np

def publish_grasp_markers():
    rospy.init_node('grasp_markers_publisher')

    marker_pub = rospy.Publisher('/grasp_markers', MarkerArray, queue_size=10)
    rospy.sleep(1.0)  # wait for publisher to connect

    listener = tf.TransformListener()
    rospy.sleep(1.0)  # let tf listener initialize

    # Read grasp poses from file
    grasp_file_path = os.path.expanduser("/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test/selected_grasps.txt")
    grasp_poses = []
    with open(grasp_file_path, 'r') as f:
        for line in f:
            values = list(map(float, line.strip().split()))
            if len(values) == 7:
                grasp_poses.append(values)

    marker_array = MarkerArray()
    transformed_tips = []

    for i, pose_vals in enumerate(grasp_poses):
        x, y, z, qx, qy, qz, qw = pose_vals

        input_pose = PoseStamped()
        input_pose.header.frame_id = 'head_camera_rgb_optical_frame'
        input_pose.pose.position.x = x
        input_pose.pose.position.y = y
        input_pose.pose.position.z = z
        input_pose.pose.orientation.x = qx
        input_pose.pose.orientation.y = qy
        input_pose.pose.orientation.z = qz
        input_pose.pose.orientation.w = qw

        try:
            listener.waitForTransform('base_link', 'head_camera_rgb_optical_frame', rospy.Time(), rospy.Duration(4.0))
            transformed_pose = listener.transformPose('base_link', input_pose)
            
            # Calculate tip position (move along +X by 0.1m)
            quat = transformed_pose.pose.orientation
            translation = np.array([0.1, 0.0, 0.0])
            rotation = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
            tip_in_base = np.dot(rotation, np.append(translation, 1.0))
            tip_position = transformed_pose.pose.position
            tip_x = tip_position.x + tip_in_base[0]
            tip_y = tip_position.y + tip_in_base[1]
            tip_z = tip_position.z + tip_in_base[2]
            
            # Save tip position + orientation
            transformed_tips.append([tip_x, tip_y, tip_z, quat.x, quat.y, quat.z, quat.w])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr(f"TF transform failed for grasp {i}")
            continue

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "grasp_arrows"
        marker.id = i
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = transformed_pose.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker_array.markers.append(marker)

        text_marker = Marker()
        text_marker.header.frame_id = "base_link"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "grasp_labels"
        text_marker.id = 1000 + i
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = transformed_pose.pose.position.x
        text_marker.pose.position.y = transformed_pose.pose.position.y
        text_marker.pose.position.z = transformed_pose.pose.position.z + 0.05
        text_marker.scale.z = 0.05
        text_marker.color.r = 1.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0
        text_marker.text = f"Grasp {i}"
        marker_array.markers.append(text_marker)

    # Save all transformed tip grasps to a text file
    output_path = os.path.expanduser("~/catkin_ws/src/fetch_joint_controls/scripts/Test/transformed_grasp_tips.txt")
    with open(output_path, 'w') as f:
        for g in transformed_tips:
            f.write(' '.join([f"{num:.8f}" for num in g]) + '\n')
    rospy.loginfo(f"Saved all transformed grasp tips to {output_path}")

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == "__main__":
    publish_grasp_markers()
