#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
import tf
from visualization_msgs.msg import Marker
import copy

def main():
    rospy.init_node('fetch_pick_pose_marker', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    listener = tf.TransformListener()
    rospy.sleep(1.0)  # Allow TF listener to get started

    # --- Read grasp pose from file ---
    grasp_data = []
    with open('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/fetch_grasps.txt', 'r') as f:
        grasp_data = [float(s) for s in f.readline().split()]

    test_latest_pose = Pose(
        Point(grasp_data[0], grasp_data[1], grasp_data[2]),
        Quaternion(grasp_data[3], grasp_data[4], grasp_data[5], grasp_data[6])
    )

    listener.waitForTransform('head_camera_rgb_optical_frame', 'base_link', rospy.Time(), rospy.Duration(1.0))
    
    test_pose_stamped = PoseStamped()
    test_pose_stamped.header.frame_id = 'head_camera_rgb_optical_frame'
    test_pose_stamped.pose = test_latest_pose
    transformed_pose = listener.transformPose('base_link', test_pose_stamped)

    # --- Pre-pick and Pick Poses ---
    pre_pick_pose = copy.deepcopy(transformed_pose)
    pre_pick_pose.pose.position.x -= 0.2

    pick_pose = copy.deepcopy(pre_pick_pose)
    pick_pose.pose.position.x += 0.2

    print("Computed Pick Pose:\n", pick_pose)

    # --- Prepare Marker ---
    marker = Marker()
    marker.header.frame_id = pick_pose.header.frame_id
    marker.ns = "pick_pose_marker"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.2  # Arrow shaft length
    marker.scale.y = 0.05  # Shaft thickness
    marker.scale.z = 0.05  # Arrow head size
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # --- Publish marker continuously ---
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()

        # Update marker pose live (if pick_pose moves dynamically, this will track it)
        marker.pose = pick_pose.pose

        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    main()
