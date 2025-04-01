#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("fetch_wave_motion", anonymous=True)

    arm = moveit_commander.MoveGroupCommander("arm")

    # Move to a "wave-ready" joint pose
    wave_pose = [0.0, 0.7, 0.0, 1.4, 0.0, 1.2, 0.0]  # elbow bent, wrist up
    arm.go(wave_pose, wait=True)
    arm.stop()

    rospy.sleep(1.0)

    # Create waving motion (wrist_flex_joint index = 5)
    traj = arm.plan()
    traj_points = []

    wrist_angles = [1.2, 0.8, 1.2, 0.8, 1.2]  # oscillate wrist
    time_per_point = 2.0  # seconds

    for i, angle in enumerate(wrist_angles):
        pt = JointTrajectoryPoint()
        pt.positions = wave_pose.copy()
        pt.positions[5] = angle
        pt.time_from_start = rospy.Duration.from_sec((i + 1) * time_per_point)
        traj_points.append(pt)

    # Assign the custom waypoints to the planned trajectory
    traj.joint_trajectory.points = traj_points
    traj.joint_trajectory.joint_names = arm.get_joints()[1:-1]  # skip virtual joints

    # Execute the trajectory
    arm.execute(traj, wait=True)
    arm.stop()

    rospy.loginfo("Wave complete!")

if __name__ == '__main__':
    main()
