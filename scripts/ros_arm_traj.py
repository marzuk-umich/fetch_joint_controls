#!/usr/bin/env python3

import rospy
import threading
import numpy as np
import csv
from collections import deque
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from fetch_joint_controls.msg import traj_opt
import actionlib

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
        self.actual_velocity_log = []
        self.actual_acceleration_log = []
        self.prev_velocity = None
        self.prev_timestamp = None
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
        if not cb:
            self.client.wait_for_result()

    def _joint_callback(self, msg):
        if self.joint_names[0] in msg.name:
            self.timestamp = msg.header.stamp
            self.state[:len(self.joint_names), 0] = msg.position[self.config['data_start']:self.config['data_end']]
            self.state[len(self.joint_names):, 0] = msg.velocity[self.config['data_start']:self.config['data_end']]
            self.ready = True
            t = self.timestamp.secs + self.timestamp.nsecs * 1e-9
            self.actual_positions_log.append((t, self.state[:len(self.joint_names), 0].tolist()))
            self.actual_velocity_log.append((t, self.state[len(self.joint_names):, 0].tolist()))
            if self.prev_velocity is not None and self.prev_timestamp is not None:
                dt = t - self.prev_timestamp
                acc_est = (self.state[len(self.joint_names):, 0] - self.prev_velocity) / dt
                self.actual_acceleration_log.append((t, acc_est.tolist()))
            self.prev_velocity = self.state[len(self.joint_names):, 0].copy()
            self.prev_timestamp = t

    def create_zero_waypoint(self, n=1):
        return np.zeros((n, 4, self.config['joints_num']))

    def current_waypoint(self, n=1):
        time = np.ones((1, 1, self.config['joints_num']))
        pos = self.state[:self.config['joints_num'], 0][np.newaxis, np.newaxis, :]
        vel = np.zeros((1, 1, self.config['joints_num']))
        acc = np.zeros((1, 1, self.config['joints_num']))
        return np.concatenate([time, pos, vel, acc], axis=1)

def export_to_csv():
    with open("/home/marzuk/catkin_ws/src/fetch_joint_controls/test/commanded_all_joints.csv", "w", newline="") as f_cmd:
        writer = csv.writer(f_cmd)
        writer.writerow(["Time"] + [f"Joint_{i}" for i in range(7)])
        for t, pos_list in commanded_positions_log:
            writer.writerow([t] + pos_list)

    with open("/home/marzuk/catkin_ws/src/fetch_joint_controls/test/commanded_all_joints_velocity.csv", "w", newline="") as f_vel:
        writer = csv.writer(f_vel)
        writer.writerow(["Time"] + [f"Joint_{i}" for i in range(7)])
        for t, vel_list in commanded_velocity_log:
            writer.writerow([t] + vel_list)

    with open("/home/marzuk/catkin_ws/src/fetch_joint_controls/test/commanded_all_joints_accln.csv", "w", newline="") as f_cmd:
        writer = csv.writer(f_cmd)
        writer.writerow(["Time"] + [f"Joint_{i}" for i in range(7)])
        for t, accln_list in commanded_acceleration_log:
            writer.writerow([t] + accln_list)

    with open("/home/marzuk/catkin_ws/src/fetch_joint_controls/test/actual_all_joints.csv", "w", newline="") as f_act:
        writer = csv.writer(f_act)
        writer.writerow(["Time"] + [f"Joint_{i}" for i in range(7)])
        for t, pos_list in arm_client.actual_positions_log:
            writer.writerow([t] + pos_list)

    with open("/home/marzuk/catkin_ws/src/fetch_joint_controls/test/actual_all_joints_velocity.csv", "w", newline="") as f_vel:
        writer = csv.writer(f_vel)
        writer.writerow(["Time"] + [f"Joint_{i}" for i in range(7)])
        for t, vel_list in arm_client.actual_velocity_log:
            writer.writerow([t] + vel_list)

    with open("/home/marzuk/catkin_ws/src/fetch_joint_controls/test/actual_all_joints_accln.csv", "w", newline="") as f_act:
        writer = csv.writer(f_act)
        writer.writerow(["Time"] + [f"Joint_{i}" for i in range(7)])
        for t, accln_list in arm_client.actual_acceleration_log:
            writer.writerow([t] + accln_list)

    with open("/home/marzuk/catkin_ws/src/fetch_joint_controls/test/fail_flags.csv", "w", newline="") as f_fail:
        writer = csv.writer(f_fail)
        writer.writerow(["Time", "FailFlag"])
        for t, fail_flag in fail_flags_log:
            writer.writerow([t, fail_flag])

optimizer_queue = deque()
position = np.zeros((1, 7))
velocity = np.zeros((1, 7))
acceleration = np.zeros((1, 7))
current_time = 0.0
dof = 7
commanded_positions_log = []
commanded_velocity_log = []
commanded_acceleration_log = []
fail_flags_log = []

def optimizer_callback(msg):
    optimizer_queue.append({
        'ka': np.array(msg.ka.data),
        'fail_flag': msg.fail_flag.data,
        'timestamp': msg.t.data
    })

def trajectory_execution_loop(arm_client):
    global position, velocity, acceleration, current_time
    dt = 0.5
    steps = 500
    dt_step = dt / steps
    flag = 0
    while not rospy.is_shutdown():
        if not optimizer_queue:
            rospy.sleep(0.01)
            continue

        msg = optimizer_queue.popleft()
        fail_flags_log.append((rospy.get_time(), msg['fail_flag']))
        
        if msg['fail_flag'] == 0:
            acceleration = np.zeros((1, 7))
            acceleration[0] = np.append(msg['ka'], 0.0)
            # print(acceleration[0,1])

        elif msg['fail_flag'] !=0 :
            acceleration[0] = -velocity[0] / dt
            position += velocity * dt + 0.5 * acceleration * dt**2
            velocity[:] = 0


        qdd = np.tile(acceleration, (steps, 1))
        q = np.zeros((steps, dof))
        qd = np.zeros((steps, dof))
        q[0] = position.copy()
        qd[0] = velocity.copy()

        print(position[0,5])

        for i in range(1, steps):
            if msg['fail_flag'] == 0:
                qd[i] = qd[i - 1] + qdd[i - 1] * dt_step
                q[i] = q[i - 1] + qd[i - 1] * dt_step + 0.5 * qdd[i - 1] * dt_step**2

            if msg['fail_flag'] != 0:
                q[i] = q[i - 1] + qd[i - 1] * dt_step + 0.5 * qdd[i - 1] * dt_step**2
                qd[i] = np.zeros((1,7))


        traj = JointTrajectory()
        traj.joint_names = arm_client.joint_names
        t_now = rospy.get_time()

        v_traj = _interpolate_q(velocity, steps)
        a_traj = _interpolate_q(acceleration[0], steps)

        for i in range(steps):
            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration((i + 1) * dt_step)
            pt.positions = q[i].tolist()
            pt.velocities = qd[i].tolist()
            pt.accelerations = qdd[i].tolist()
            traj.points.append(pt)

            commanded_positions_log.append((t_now + (i + 1) * dt_step, pt.positions[:]))
            commanded_velocity_log.append((t_now + (i + 1) * dt_step, pt.velocities[:]))
            commanded_acceleration_log.append((t_now + (i + 1) * dt_step, pt.accelerations[:]))

        # Update state for continuity in next loop
        position[:] = q[-1]
        velocity[:] = qd[-1]

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        goal.goal_time_tolerance = rospy.Duration(0.0)
        arm_client.client.send_goal(goal)
        arm_client.client.wait_for_result()

def _interpolate_q(q, out_steps, match_end=True, match_start=False):
    if not (match_start or match_end):
        raise ValueError
    if len(q.shape) == 1:
        q = np.expand_dims(q, 0)
    in_times = np.linspace(0, 0.5, num=len(q) + int(not match_start), endpoint=match_end)
    coll_times = np.linspace(0, 0.5, num=out_steps + int(not match_start), endpoint=match_end)
    if not match_start:
        in_times = in_times[1:]
        coll_times = coll_times[1:]
    q_interp = [np.interp(coll_times, in_times, q[:, i]) for i in range(dof)]
    return np.array(q_interp, order='F').T

def on_shutdown():
    rospy.loginfo("Shutting down, saving logs to CSV...")
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

    torso_time = np.ones((1, 1, torso_client.config['joints_num'])) * 3
    torso_goal_pos = np.ones((1, 1, torso_client.config['joints_num'])) * 0.3
    torso_goal_vel = np.ones((1, 1, torso_client.config['joints_num'])) * 0.0
    torso_goal_accln = np.ones((1, 1, torso_client.config['joints_num'])) * 0.0
    torso_waypoints = np.concatenate([torso_time, torso_goal_pos, torso_goal_vel, torso_goal_accln], axis=1)
    rospy.loginfo("Raising torso...")
    torso_waypoints = np.concatenate([torso_client.current_waypoint(), torso_waypoints], axis=0)
    torso_client.move_to(torso_waypoints, False)

    detailed_trajectory = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/New/center_box_avoidance/center_box_avoidance/center_box_avoidance_detailed_traj.npy', allow_pickle=True)
    position[0, :6] = detailed_trajectory.item().get('q')[0, :]

    arm_waypoints = arm_client.create_zero_waypoint()
    arm_waypoints[:, 0, :] = 4.
    arm_waypoints[:, 1, :] = np.array([1.32, 0, -1.57, 1.72, 0.0, 1.66, 0.0])
    arm_waypoints[:, 2, :] = 0.
    rospy.loginfo("Moving arm to safe position...")
    arm_waypoints = np.concatenate([arm_client.current_waypoint(), arm_waypoints], axis=0)
    arm_client.move_to(arm_waypoints, False)

    arm_waypoints = arm_client.create_zero_waypoint()
    arm_waypoints[:, 0, :] = 4.
    arm_waypoints[:, 1, :] = position
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
