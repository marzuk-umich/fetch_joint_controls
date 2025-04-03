 #!/usr/bin/env python3

 import rospy
 import threading
 import numpy as np
 import csv
 from collections import deque
 import time
 from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
 from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
 from sensor_msgs.msg import JointState
 from fetch_joint_controls.msg import traj_opt
 import actionlib
 
 from control_msgs.msg import FollowJointTrajectoryFeedback
 
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
         self.actual_positions_log = []  # (time, [positions])
         rospy.Subscriber('/joint_states', JointState, callback=self._joint_callback, queue_size=10)
 
     def move_to(self, waypoints, cb=False):
         trajectory = JointTrajectory()
         trajectory.joint_names = self.joint_names
         for i in range(waypoints.shape[0]):
             trajectory.points.append(JointTrajectoryPoint())
             trajectory.points[i].time_from_start = rospy.Duration(waypoints[i, 0, 0])
             trajectory.points[i].positions = waypoints[i, 1, :]
             trajectory.points[i].velocities = waypoints[i, 2, :]
             trajectory.points[i].accelerations =  waypoints[i, 3, :]
         follow_goal = FollowJointTrajectoryGoal()
         follow_goal.trajectory = trajectory
         follow_goal.goal_time_tolerance = rospy.Duration(0.0)
         if cb:
             self.client.send_goal(follow_goal, feedback_cb=self.feedback_cb)
         else:
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
 
     def feedback_cb(self, feedback):
         print(f"Goal feedback callback output:\n{feedback}")
 
     def cancel_all_goals(self):
         self._joint_client.cancel_all_goals()
 
     def current_waypoint(self, n=1):
         time = np.ones((1, 1, self.config['joints_num'])) * 1
         goal_pos = self.state[:self.config['joints_num'], 0][np.newaxis, np.newaxis, :]
         goal_vel = np.zeros((1, 1, self.config['joints_num']))
         goal_accln = np.zeros((1, 1, self.config['joints_num']))
 
         waypoint = np.concatenate([time, goal_pos, goal_vel, goal_accln], axis=1)
         waypoints = np.ones((n, *waypoint.shape[1:]), dtype=waypoint.dtype) * waypoint
         return waypoints
 
     def create_zero_waypoint(self, n=1):
         return np.zeros((n, 4, self.config['joints_num']))
 
 optimizer_queue = deque()
 position = np.zeros((1, 7))
 velocity = np.zeros((1, 7))
 acceleration = np.zeros((1, 7))
 current_time = 0.0
 qvel = np.zeros((1,7))
 dof = 3
 
 commanded_positions_log = []  # (time, [positions])
 
 def optimizer_callback(msg):
     optimizer_queue.append({
         'ka': np.array(msg.ka.data),
         'fail_flag': msg.fail_flag.data,
         'timestamp': msg.t.data
     })
 
 def trajectory_execution_loop(arm_client):
     # TODO: Lets try doing something like descretizing the control input and then finding the vel and pos. 
     global position, velocity, acceleration, current_time
     dt = 0.5
     steps = 100
     dt_step = dt / steps
     while not rospy.is_shutdown():
         if not optimizer_queue:
             rospy.sleep(0.01)
             continue
 
         msg = optimizer_queue.popleft()
 
         if msg['fail_flag'] == 0:
             acceleration[0, :6] = msg['ka']
 
 
         else:
             acceleration = -velocity / dt # Since velocity is set to 0, this works to decelerate to 0
             velocity = np.zeros([1,7])
 
 
         qdd = _interpolate_q(acceleration, steps)  #id = 100
         t = dt_step / steps # t = 0.5 /100
         qd_delt = np.cumsum(qdd * t, axis=0) #col addition
         qd = velocity + qd_delt
         q = position + np.cumsum(0.5 * qdd * t * t, axis=0)
         q[1:] += np.cumsum(qd[:-1] * t, axis=0)
         print(q.shape)
 
 
         traj = JointTrajectory()
         traj.joint_names = arm_client.joint_names
 
         v = velocity.copy()
         p = position.copy()
         a = acceleration.copy()
         t_now = rospy.get_time()
 
         for i in range(steps):
             p += v * dt_step + 0.5 * a * dt_step ** 2
             v += a * dt_step
 
             pt = JointTrajectoryPoint()
             pt.time_from_start = rospy.Duration.from_sec((i + 1) * dt_step)
             pt.positions = p.flatten().tolist()
             pt.velocities = v.flatten().tolist()
             pt.accelerations = a.flatten().tolist()
             traj.points.append(pt)
 
             commanded_positions_log.append((t_now + (i + 1) * dt_step, pt.positions[:]))
 
         velocity[:] = v
         print(velocity.shape)
         position[:] = p
         current_time += dt
 
         goal = FollowJointTrajectoryGoal()
         goal.trajectory = traj
         goal.goal_time_tolerance = rospy.Duration(0.0)
         rospy.loginfo("Sending trajectory...")
         arm_client.client.send_goal(goal)
         arm_client.client.wait_for_result()
         rospy.loginfo("Trajectory done.")
         # time.sleep(2)
 
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
 
 
 def _interpolate_q(q: np.ndarray, out_steps, match_end=True, match_start=False):
     '''Interpolate the joint positions to a desired number of steps.
 
     Args:
         q: np.ndarray, shape (timestep_discretization, dof)
             The joint positions to interpolate.
         out_steps: int
             The number of steps to interpolate to.
         match_end: bool
             Whether to match the end of the trajectory time.
         match_start: bool
             Whether to match the start of the trajectory time.
     
     Returns:
         np.ndarray, shape (out_steps, dof)
             The interpolated joint positions.
     
     Raises:
         ValueError: If neither match_start nor match_end is True.
     '''
     if not (match_start or match_end):
         raise ValueError
     if len(q.shape) == 1:
         q = np.expand_dims(q,0)
     in_times = np.linspace(0, 0.5, num=len(q) + int(not match_start), endpoint=match_end)
     coll_times = np.linspace(0, 0.5, num=out_steps + int(not match_start), endpoint=match_end)
     if not match_start:
         in_times = in_times[1:]
         coll_times = coll_times[1:]
     q_interpolated = [None]*dof
     for i in range(dof):
         q_interpolated[i] = np.interp(coll_times, in_times, q[:,i])
     q_interpolated = np.array(q_interpolated, order='F').T # Use fortran order so it's C order when transposed.
     return q_interpolated
 
 
 
 def on_shutdown():
     rospy.loginfo("Shutting down, saving logs to CSV...")
     export_to_csv()
 
 if __name__ == '__main__':
     rospy.init_node("queued_trajectory_runner")
 
     torso_joint_names = ["torso_lift_joint"]
     config = {}
     config['joints_num'] = len(torso_joint_names)
     config['data_start'] = 2
     config['data_end'] = config['data_start'] + config['joints_num']
     torso_client = FollowTrajectoryClient("torso_controller/follow_joint_trajectory", torso_joint_names, config)
 
     arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                        "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
     config = {
         'joints_num': len(arm_joint_names),
         'data_start': 6,
         'data_end': 6 + len(arm_joint_names)
     }
     arm_client = FollowTrajectoryClient('arm_controller/follow_joint_trajectory', arm_joint_names, config)
 
     while not arm_client.ready and not rospy.is_shutdown():
         rospy.sleep(0.1)
 
     torso_time = np.ones((1, 1, torso_client.config['joints_num'])) * 3
     torso_goal_pos = np.ones((1, 1, torso_client.config['joints_num'])) * 0.3
     torso_goal_vel = np.ones((1, 1, torso_client.config['joints_num'])) * 0.0
     torso_goal_accln = np.ones((1, 1, torso_client.config['joints_num'])) * 0.0
     torso_waypoints = np.concatenate([torso_time, torso_goal_pos, torso_goal_vel,torso_goal_accln], axis=1)
     rospy.loginfo("Raising torso...")
     # torso_client.move_to(torso_waypoints, False)
 
     arm_waypoints = arm_client.create_zero_waypoint()
     arm_waypoints[:, 0, :] = 5.
     arm_waypoints[:, 1, :] = np.array([1.32, 0, -1.57, 1.72, 0.0, 1.66, 0.0])
     arm_waypoints[:, 2, :] = 0.
     rospy.loginfo("Moving arm to safe position...")
     arm_waypoints = np.concatenate([arm_client.current_waypoint(), arm_waypoints], axis=0)
     # arm_client.move_to(arm_waypoints, False)
 
     arm_waypoints = arm_client.create_zero_waypoint()
     arm_waypoints[:, 0, :] = 4.
     arm_waypoints[:, 1, :] = np.array([0, 0, -1.57, 1.72, 0.0, 0.0, 0.0])
     arm_waypoints[:, 2, :] = 0.
     arm_waypoints = np.concatenate([arm_client.current_waypoint(), arm_waypoints], axis=0)
     # arm_client.move_to(arm_waypoints, False)
 
     rospy.Subscriber('/traj_opt_data', traj_opt, optimizer_callback)
 
     exec_thread = threading.Thread(target=trajectory_execution_loop, args=(arm_client,))
     exec_thread.start()
 
     rospy.on_shutdown(on_shutdown)
     rospy.spin()