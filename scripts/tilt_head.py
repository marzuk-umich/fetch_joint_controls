#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def move_head_to_exact_position():
    rospy.init_node('move_head_to_exact_position')

    client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']

    point = JointTrajectoryPoint()
    # Set exact positions you measured
    point.positions = [0.0012, 0.69]  # pan, tilt
    point.time_from_start = rospy.Duration(2.0)  # Move in 2 seconds

    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.Time.now()

    client.send_goal(goal)
    client.wait_for_result()

if __name__ == '__main__':
    move_head_to_exact_position()
