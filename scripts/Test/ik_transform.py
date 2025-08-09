#!/usr/bin/env python
import rospy
import tf
import csv
from geometry_msgs.msg import PoseStamped

def read_pose_from_txt(path):
    with open(path, 'r') as f:
        line = f.readline().strip()
        values = list(map(float, line.replace(',', ' ').split()))
        assert len(values) == 7, "Pose must have 7 elements (x y z qx qy qz qw)"
        return values[:3], values[3:]

def transform_pose(xyz, quat, source_frame='head_camera_rgb_optical_frame', target_frame='base_link'):
    listener = tf.TransformListener()
    rospy.sleep(1.0)  # Give TF buffer time

    pose = PoseStamped()
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = source_frame
    pose.pose.position.x = xyz[0]
    pose.pose.position.y = xyz[1]
    pose.pose.position.z = xyz[2]
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
    transformed = listener.transformPose(target_frame, pose)
    return transformed.pose


def write_pose_to_csv(pose, path):
    data = [
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    ]
    with open(path, 'w') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writerow(data)
    print(f" Transformed pose saved to {path}")

if __name__ == "__main__":
    rospy.init_node("transform_grasp_pose", anonymous=True)
    input_txt = "/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/fetch_grasps.txt"
    output_csv = "/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/Test/transformed_grasp_pose.csv"

    xyz, quat = read_pose_from_txt(input_txt)
    transformed_pose = transform_pose(xyz, quat)
    write_pose_to_csv(transformed_pose, output_csv)
