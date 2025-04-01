#!/bin/env python3
import sys
import actionlib
import rospy
import numpy as np
import threading
import matplotlib.pyplot as plt

from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading 
import time 
from fetch_joint_controls.msg import traj_opt
from scipy.interpolate import interp1d
from test_joints import plot_trajectories_with_optimizer, generate_smoothened_trajectory

class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names, config):
        self.client = actionlib.SimpleActionClient(name, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names
        # joints config and state
        self.config = config
        self.timestamp = None
        self.state = np.zeros(((len(joint_names) * 2), 1))
        # joint subscription
        self.joint_sub = rospy.Subscriber('joint_states', JointState, callback=self._joint_callback, queue_size=10)
        self.ready = False
        # thread recording
        self.state_recordings = None
        self.time_recordings = None

    def move_to(self, waypoints, cb=False):
        # if len(self.joint_names) != len(waypoints):
        #     print("Invalid trajectory position")
        #     return False
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
    
    def feedback_cb(self, feedback):
        print(f"Goal feedback callback output:\n{feedback}")

    def cancel_all_goals(self):
        self._joint_client.cancel_all_goals()

    def check_waypoints(self, waypoints):
        pass

    def check_time_from_start(self, time):
        pass

    def check_positions(self, positions):
        pass

    def check_velocities(self, velocities):
        pass

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
    
    def record_state(self):
        self.state_recordings = self.state.copy()
        self.time_recordings = [self.timestamp.secs + self.timestamp.nsecs * 1e-9]
        while self.keep_running:
            rospy.sleep(0.1)
            self.time_recordings.append(self.timestamp.secs + self.timestamp.nsecs * 1e-9)
            self.state_recordings = np.concatenate([self.state_recordings, self.state], axis=1)

    def create_recording(self):
        self.thread = threading.Thread(target=self.record_state)
        self.thread.daemon = True
        self.keep_running = True
            
    def start_thread(self):
        self.thread.start()

    def stop_thread(self):
        self.keep_running = False
        self.thread.join()
        self.time_recordings = np.array(self.time_recordings)
        self.time_recordings -= arm_client.time_recordings[0]


####### Smoothened Trajectory Plots
def plot_smoothened_joint_trajectory():
    global initial_position, initial_velocity

    detailed_trajectory     = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_detailed_traj.npy', allow_pickle=True)
    optimizer_raw_data      = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_opt_info.npy', allow_pickle=True)
    NUMBER_OF_JOINTS        = 7
    dt                      = 0.5
    current_time            = np.zeros((1,NUMBER_OF_JOINTS))
    acceleration            = np.zeros((1,7)) 
    velocity                = np.zeros((1,7))
    position                = np.zeros((1,7))    
    t_detailed              = detailed_trajectory.item().get('t')
    position_detailed       = detailed_trajectory.item().get('q')
    velocity_detailed       = detailed_trajectory.item().get('qd')
    acceleration_detailed   = detailed_trajectory.item().get('qdd')
    acceleration_opt        = optimizer_raw_data.item().get('ka')
    position[0,:6]          = detailed_trajectory.item().get('q')[0,:]
    velocity[0,:6]          = detailed_trajectory.item().get('qd')[0,:]
    fail_flag_opt           = optimizer_raw_data.item().get('fail_flag')
    t_opt                   = optimizer_raw_data.item().get('plan_time_curr')
    waypoint                = arm_client.create_zero_waypoint()
    waypoint                = np.zeros((len(t_opt), 4, NUMBER_OF_JOINTS))


    for i in (range(len(t_opt))):
        if i < 39:
            if fail_flag_opt[i] == 0:
                acceleration[0,:6] = acceleration_opt[i,:]
                position += velocity * dt + (0.5*acceleration*dt*dt)  # s = s + v*t
                velocity += dt * acceleration
            else: 
                acceleration = -velocity / dt  
                velocity = np.zeros([1,NUMBER_OF_JOINTS])
                position += velocity * dt + (0.5*acceleration*dt*dt)  # s = st + 0.5*v*t*t

        current_time += np.ones((1,NUMBER_OF_JOINTS))*dt
        waypoint[i, 0, :] = current_time
        waypoint[i, 1, :] = position
        waypoint[i, 2, :] = velocity
        waypoint[i, 3, :] = acceleration
    # arm_client.move_to(waypoint)    
    fine_tuned_time_data ,fine_tuned_position_data, fine_tuned_velocity_data, fine_tuned_acceleration_data = generate_smoothened_trajectory(waypoint[:, 0, :], waypoint[:, 1, :], waypoint[:, 2, :], waypoint[:, 3, :])
    waypoint = np.zeros((1000, 4, NUMBER_OF_JOINTS))

    print(np.shape(waypoint[:, 0, :]))
    waypoint[:, 0, :] = fine_tuned_time_data
    waypoint[:, 1, :] = fine_tuned_position_data
    waypoint[:, 2, :] = fine_tuned_velocity_data
    waypoint[:, 3, :] = fine_tuned_acceleration_data
    for i in range(np.shape(waypoint)[0]):
        print(waypoint[i, :, :].shape)
        print(rospy.Duration(waypoint[i, 0, 0]))

    
    arm_client.move_to(waypoint)
    # index = 1 -> position
    # index = 2 -> velocity
    # index = 3 -> acceleration
    plot_trajectories_with_optimizer(waypoint, t_detailed, detailed_data=position_detailed, index=1 ) 
        


##### plotting trajectories without Smoothening
def plot_trajectory():
    from test_joints import plot_trajectories_with_optimizer
    global initial_position, initial_velocity
    detailed_trajectory = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_detailed_traj.npy', allow_pickle=True)
    position = np.zeros((1,7))
    NUMBER_OF_JOINTS = 7
    position[0,:6] = detailed_trajectory.item().get('q')[0,:]
    velocity = np.zeros((1,7))
    velocity[0,:6] = detailed_trajectory.item().get('qd')[0,:]
    current_time = np.zeros((1,NUMBER_OF_JOINTS))
    dt = 0.5
    acceleration = np.zeros((1,7)) 
    
    position_detailed = detailed_trajectory.item().get('q')
    velocity_detailed = detailed_trajectory.item().get('qd')
    acceleration_detailed = detailed_trajectory.item().get('qdd')


    t_detailed = detailed_trajectory.item().get('t')

    waypoint = arm_client.create_zero_waypoint()

    # Plot position

    optimizer_raw_data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_opt_info.npy', allow_pickle=True)
    acceleration_opt = optimizer_raw_data.item().get('ka')
    fail_flag_opt = optimizer_raw_data.item().get('fail_flag')
    # print(optimizer_raw_data)
    t_opt = optimizer_raw_data.item().get('plan_time_curr')
    waypoint = np.zeros((len(t_opt), 4, NUMBER_OF_JOINTS))


    for i in (range(len(t_opt))):
        if i < 39:
            if fail_flag_opt[i] == 0:
                acceleration[0,:6] = acceleration_opt[i,:]
                position += velocity * dt + (0.5*acceleration*dt*dt)  # s = s + v*t
                velocity += dt * acceleration
            else: 
                acceleration = -velocity / dt  # Since velocity is set to 0, this works to decelerate to 0
                velocity = np.zeros([1,NUMBER_OF_JOINTS])
                position += velocity * dt + (0.5*acceleration*dt*dt)  # s = s + v*t

        current_time += np.ones((1,NUMBER_OF_JOINTS))*dt
        waypoint[i, 0, :] = current_time
        waypoint[i, 1, :] = position
        waypoint[i, 2, :] = velocity
        waypoint[i, 3, :] = acceleration
    # print(waypoint)
    arm_client.move_to(waypoint)    
    
    
    #index = 1 -> position
    #index = 2 -> velocity
    #index = 3 -> acceleration
    plot_trajectories_with_optimizer(waypoint, t_detailed, detailed_data=position_detailed, index=1 ) 




detailed_trajectory = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_detailed_traj.npy', allow_pickle=True)
initial_position = np.array([-7.85391618e-01,  1.00065447e-02, -1.22795596e-09, -1.16367514e-07, 0.00000000e+00,  5.00065426e-02])
initial_velocity = np.array([ 1.30899533e-03,  1.30893494e-03, -2.45591220e-07, -2.32735029e-05, 0.00000000e+00,  1.30852094e-03])
dt = 0.5 
num_joints = 7
current_time = np.zeros((1,num_joints))
position = np.zeros((1, num_joints))   
position[0,:6] = initial_position
velocity = np.zeros((1, num_joints))  
velocity[0,:6] = initial_velocity
acceleration = np.zeros((1, num_joints)) 
current_time = np.ones((1,num_joints))


from collections import deque
optimizer_queue = deque()

def optimizer_callback(msg):
    optimizer_queue.append({
        'ka': np.array(msg.ka.data),
        'fail_flag': msg.fail_flag.data,
        'timestamp': msg.t.data
    })

def trajectory_execution_loop():
    global position, velocity, acceleration
    dt = 0.5
    steps = 100
    dt_step = dt / steps

    while not rospy.is_shutdown():
        if not optimizer_queue:
            rospy.sleep(0.01)
            continue

        # Get next acceleration
        msg = optimizer_queue.popleft()

        if msg['fail_flag'] == 0:
            acceleration[0, :6] = msg['ka']
        else:
            acceleration = -velocity / dt
            velocity = np.zeros_like(velocity)

        # Generate trajectory points
        traj = JointTrajectory()
        traj.joint_names = arm_client.joint_names

        v = velocity.copy()
        p = position.copy()
        a = acceleration.copy()

        for i in range(steps):
            v += a * dt_step
            p += v * dt_step + 0.5 * a * dt_step ** 2

            pt = JointTrajectoryPoint()
            pt.time_from_start = rospy.Duration.from_sec((i + 1) * dt_step)
            pt.positions = p.flatten().tolist()
            pt.velocities = v.flatten().tolist()
            pt.accelerations = a.flatten().tolist()
            traj.points.append(pt)

        # Update global state
        velocity[:] = v
        position[:] = p

        # Send goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        goal.goal_time_tolerance = rospy.Duration(0.0)

        arm_client.client.send_goal(goal)
        arm_client.client.wait_for_result()




if __name__ == '__main__':

    from test_joints import generate_trajectory, plot_trajectories, test_joints_independently


    rospy.init_node('arm_to_pos', anonymous=True)
    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Check robot serial number, uncomment this line when running on fetch!
    # if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX": 
    #     rospy.logerr("This script should not be run on a real robot")
    #     sys.exit(-1)
    
    torso_joint_names = ["torso_lift_joint"]
    config = {}
    config['joints_num'] = len(torso_joint_names)
    config['data_start'] = 2
    config['data_end'] = config['data_start'] + config['joints_num']
    torso_client = FollowTrajectoryClient("torso_controller/follow_joint_trajectory", torso_joint_names, config)

    arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
    config = {}
    config['joints_num'] = len(arm_joint_names)
    config['data_start'] = 6
    config['data_end'] = config['data_start'] + config['joints_num']
    print(config['joints_num'])
    arm_client = FollowTrajectoryClient('arm_controller/follow_joint_trajectory', arm_joint_names, config)

    while not (arm_client.ready and torso_client.ready):
        rospy.sleep(0.1)

    # Move ARM to save position

    # # Move torso up!
    torso_time = np.ones((1, 1, torso_client.config['joints_num'])) * 3
    torso_goal_pos = np.ones((1, 1, torso_client.config['joints_num'])) * 0.3
    torso_goal_vel = np.ones((1, 1, torso_client.config['joints_num'])) * 0.0
    torso_goal_accln = np.ones((1, 1, torso_client.config['joints_num'])) * 0.0
    torso_waypoints = np.concatenate([torso_time, torso_goal_pos, torso_goal_vel,torso_goal_accln], axis=1)
    rospy.loginfo("Raising torso...")
    print(np.shape(torso_client.current_waypoint()))
    torso_waypoints = np.concatenate([torso_client.current_waypoint(), torso_waypoints], axis=0)
    torso_client.move_to(torso_waypoints, False)

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

    # update_trajectory_with_optimizer()
    # Uncomment the line below to use the optimizer without ROS
    current_time = 0.0
    velocity = np.zeros((1, 7))
    position = np.zeros((1, 7))
    acceleration = np.zeros((1, 7))

    
    rospy.Subscriber('/traj_opt_data', traj_opt, optimizer_callback)

    # Start the consumer loop
    exec_thread = threading.Thread(target=trajectory_execution_loop)
    exec_thread.start()

    rospy.spin()


    # Uncomment the line below to use the optimizer without ROS
    # plot_smoothened_joint_trajectory()
    
    