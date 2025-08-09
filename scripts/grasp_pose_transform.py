#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import argparse

def transform_grasp_pose(position, quaternion, source_frame, target_frame):
    rospy.init_node('grasp_pose_transform', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0)

    pose = PoseStamped()
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = source_frame
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    try:
        transformed = tf_buffer.transform(pose, target_frame, rospy.Duration(2.0))
        new_position = [
            transformed.pose.position.x,
            transformed.pose.position.y,
            transformed.pose.position.z
        ]
        new_quaternion = [
            transformed.pose.orientation.x,
            transformed.pose.orientation.y,
            transformed.pose.orientation.z,
            transformed.pose.orientation.w
        ]
        return new_position, new_quaternion
    except Exception as e:
        rospy.logerr(f"[TF ERROR] Failed to transform pose: {e}")
        return None, None


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--position', nargs=3, type=float, required=True, help="x y z in source frame")
    parser.add_argument('--quaternion', nargs=4, type=float, required=True, help="x y z w quaternion in source frame")
    parser.add_argument('--source_frame', type=str, default="head_camera_rgb_optical_frame", help="TF source frame")
    parser.add_argument('--target_frame', type=str, default="base_link", help="TF target frame")
    args = parser.parse_args()

    pos = args.position
    quat = args.quaternion

    transformed_pos, transformed_quat = transform_grasp_pose(pos, quat, args.source_frame, args.target_frame)

    if transformed_pos:
        print("\n✅ Transformed Grasp Pose:")
        print("Position   :", transformed_pos)
        print("Quaternion :", transformed_quat)
    else:
        print("❌ Failed to transform pose.")
