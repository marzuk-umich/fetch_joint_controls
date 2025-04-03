#!/usr/bin/env python3

import rospy
import threading
import numpy as np
import csv
import matplotlib.pyplot as plt
import pandas as pd
from collections import deque
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from fetch_joint_controls.msg import traj_opt
import actionlib
from test_joints import generate_smoothened_trajectory

class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names, config):
        self.client = actionlib.SimpleActionClient(name, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names
        self.config = config
        self.timestamp = None
        self.state = np.zeros(((len(joint_names) * 2), 1))
        self.ready = False
        self.actual_positions_log = []
        rospy.Subscriber('/joint_states', JointState, callback=self._joint_callback, queue_size=10)

    def move_to(self, waypoints, cb=False):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        for i in range(waypoints.shape[0]):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(waypoints[i, 0, 0])
            point.positions = waypoints[i, 1, :].tolist()
            point.velocities = waypoints[i, 2, :].tolist()
            point.accelerations = waypoints[i, 3, :].tolist()
            trajectory.points.append(point)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory
        follow_goal.goal_time_tolerance = rospy.Duration(0.0)
        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

    def _joint_callback(self, msg):
        if self.joint_names[0] in msg.name:
            self.timestamp = msg.header.stamp
            self.state[:len(self.joint_names), 0] = msg.position[self.config['data_start']:self.config['data_end']]
            self.state[len(self.joint_names):, 0] = msg.velocity[self.config['data_start']:self.config['data_end']]
            self.ready = True
            t = self.timestamp.secs + self.timestamp.nsecs * 1e-9
            self.actual_positions_log.append((t, self.state[:len(self.joint_names), 0].tolist()))

    def create_zero_waypoint(self, n=1):
        return np.zeros((n, 4, self.config['joints_num']))

    def current_waypoint(self, n=1):
        time = np.ones((1, 1, self.config['joints_num']))
        pos = self.state[:self.config['joints_num'], 0][np.newaxis, np.newaxis, :]
        vel = np.zeros((1, 1, self.config['joints_num']))
        acc = np.zeros((1, 1, self.config['joints_num']))
        return np.concatenate([time, pos, vel, acc], axis=1)

optimizer_queue = deque()
position = np.zeros((1, 7))
velocity = np.zeros((1, 7))
acceleration = np.zeros((1, 7))
current_time = 0.0
dof = 7
commanded_positions_log = []

def optimizer_callback(msg):
    optimizer_queue.append({
        'ka': np.array(msg.ka.data),
        'fail_flag': msg.fail_flag.data,
        'timestamp': msg.t.data
    })

def trajectory_execution_loop(arm_client):
    global position, velocity, acceleration, current_time
    dt = 0.5
    while not rospy.is_shutdown():
        if not optimizer_queue:
            rospy.sleep(0.01)
            continue

        msg = optimizer_queue.popleft()

        if msg['fail_flag'] == 0:
            acceleration[0, :6] = msg['ka']
            position += velocity * dt + 0.5 * acceleration * dt ** 2
            velocity += acceleration * dt

        else:
            acceleration = -velocity / dt
            position += velocity * dt + 0.5 * acceleration * dt ** 2
            velocity = np.zeros((1, 7))

        current_time += np.ones((1,7))*dt
        generate_smoothened_trajectory(current_time,position,velocity, acceleration)

        traj = JointTrajectory()
        traj.joint_names = arm_client.joint_names
        pt = JointTrajectoryPoint()
        pt.time_from_start = rospy.Duration.from_sec(dt)
        pt.positions = position.flatten().tolist()
        pt.velocities = velocity.flatten().tolist()
        pt.accelerations = acceleration.flatten().tolist()
        traj.points.append(pt)

        t_now = rospy.get_time()
        commanded_positions_log.append((t_now + dt, pt.positions[:]))

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        goal.goal_time_tolerance = rospy.Duration(0.0)
        rospy.loginfo("Sending trajectory...")
        arm_client.client.send_goal(goal)
        arm_client.client.wait_for_result()
        rospy.loginfo("Trajectory done.")

        current_time += dt

def export_to_csv():
    with open("commanded_all_joints.csv", "w", newline="") as f_cmd:
        writer = csv.writer(f_cmd)
        writer.writerow(["Time"] + [f"Joint_{i}" for i in range(7)])
        for t, pos_list in commanded_positions_log:
            writer.writerow([t] + pos_list)

    with open("actual_all_joints.csv", "w", newline="") as f_act:
        writer = csv.writer(f_act)
        writer.writerow(["Time"] + [f"Joint_{i}" for i in range(7)])
        for t, pos_list in arm_client.actual_positions_log:
            writer.writerow([t] + pos_list)

    plot_joint_logs("commanded_all_joints.csv", "actual_all_joints.csv")

def plot_joint_logs(commanded_csv, actual_csv):
    cmd_df = pd.read_csv(commanded_csv)
    act_df = pd.read_csv(actual_csv)

    for joint_idx in range(7):
        plt.figure(figsize=(10, 4))
        plt.plot(cmd_df["Time"], cmd_df[f"Joint_{joint_idx}"], label="Commanded", linewidth=2)
        plt.plot(act_df["Time"], act_df[f"Joint_{joint_idx}"], label="Actual", linestyle="--")
        plt.title(f"Joint {joint_idx} Position Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (rad)")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(f"joint_{joint_idx}_plot.png")
        plt.close()

def on_shutdown():
    rospy.loginfo("Shutting down, saving logs to CSV and generating plots...")
    export_to_csv()

if __name__ == '__main__':
    rospy.init_node("queued_trajectory_runner")

    torso_joint_names = ["torso_lift_joint"]
    torso_config = {'joints_num': len(torso_joint_names), 'data_start': 2, 'data_end': 3}
    torso_client = FollowTrajectoryClient("torso_controller/follow_joint_trajectory", torso_joint_names, torso_config)

    arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                       "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    arm_config = {'joints_num': len(arm_joint_names), 'data_start': 6, 'data_end': 13}
    arm_client = FollowTrajectoryClient("arm_controller/follow_joint_trajectory", arm_joint_names, arm_config)

    # Move ARM to save position
    arm_waypoints = arm_client.create_zero_waypoint()
    arm_waypoints[:, 0, :] = 5.
    arm_waypoints[:, 1, :] = np.array([1.32, 0, -1.57, 1.72, 0.0, 1.66, 0.0])
    arm_waypoints[:, 2, :] = 0.
    rospy.loginfo("Moving arm to safe position...")
    arm_waypoints = np.concatenate([arm_client.current_waypoint(), arm_waypoints], axis=0)
    arm_client.move_to(arm_waypoints, False)

    arm_waypoints = arm_client.create_zero_waypoint()
    arm_waypoints[:, 0, :] = 4.
    arm_waypoints[:, 1, :] = np.array([0, 0, -1.57, 1.72, 0.0, 0.0, 0.0])
    arm_waypoints[:, 2, :] = 0.
    arm_waypoints = np.concatenate([arm_client.current_waypoint(), arm_waypoints], axis=0)
    arm_client.move_to(arm_waypoints, False)

    while not arm_client.ready and not rospy.is_shutdown():
        rospy.sleep(0.1)

    rospy.Subscriber('/traj_opt_data', traj_opt, optimizer_callback)
    exec_thread = threading.Thread(target=trajectory_execution_loop, args=(arm_client,))
    exec_thread.start()

    rospy.on_shutdown(on_shutdown)
    rospy.spin()

    