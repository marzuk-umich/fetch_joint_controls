#!/usr/bin/env python
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
            trajectory.points[i].accelerations = [0.0] * len(waypoints[i, 1, :]) # Because there is no acceleration data
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
        waypoint = np.concatenate([time, goal_pos, goal_vel], axis=1)
        waypoints = np.ones((n, *waypoint.shape[1:]), dtype=waypoint.dtype) * waypoint
        return waypoints
    
    def create_zero_waypoint(self, n=1):
        return np.zeros((n, 3, self.config['joints_num']))
    
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

def get_optimizer_data():
    # Simulated function to return acceleration values for 7 joints
    return np.array([1,1,1,1,1,1,1])  # Replace with actual optimizer logic

def update_trajectory_with_optimizer(desired_position, initial_position, max_vel, total_time):
    # Initialize trajectory variables
    dt = 1  # Time interval in seconds
    num_joints = 7
    current_time = np.zeros((1,num_joints))
    position = np.zeros((1, num_joints))  # Initialize position
    velocity = np.zeros((1, num_joints))  # Initialize velocity
    acceleration = np.zeros((1, num_joints))  # Initialize acceleration

    # Start loop to periodically get optimizer data and update trajectory
    while current_time[0,0] <= total_time:
        optimizer_data = get_optimizer_data()  # Get acceleration data for 7 joints
        if optimizer_data is not None:
            acceleration = optimizer_data  # Update acceleration from optimizer
        else:
            velocity = np.zeros((0,num_joints))  # Stop robot if no data - breaking

        # Update position and velocity using piecewise linear acceleration
        velocity += dt * acceleration   # v = u + at
        position += velocity * dt  # s = s + v*t
    
        # Wait for the next interval
        time.sleep(dt)
        current_time += np.ones((1,num_joints))*dt
        arm_waypoints = arm_client.create_zero_waypoint()        
        arm_waypoints[:, 0, :] = current_time
        # print(arm_waypoints[:,0,:])
        arm_waypoints[:, 1, :] = position
        # print(arm_waypoints[:,1,:])
        arm_waypoints[:, 2, :] = velocity
        # print(arm_waypoints)
        arm_waypoints = np.concatenate([arm_client.current_waypoint(), arm_waypoints], axis=0)
        # print(np.shape(arm_waypoints))
        arm_client.move_to(arm_waypoints, False)


    """
    TODO : 
    - concatenate these objects and make it similiar to the waypoints matrix 
    - implement move to at every second
    """

    print(current_time)
    print(position)
    print(velocity)
    print(acceleration)
    print("Trajectory update completed.")

    # Position
    plt.subplot(3, 1, 1)
    plt.plot(current_time, position, label="Position")
    plt.ylabel("Position")
    plt.legend()


    




if __name__ == '__main__':

    from test_joints import generate_trajectory, plot_trajectories, test_joints_independently


    rospy.init_node('arm_to_pos', anonymous=True)
    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Check robot serial number, uncomment this line when running on fetch!
    if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX": 
        rospy.logerr("This script should not be run on a real robot")
        sys.exit(-1)
    
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
    arm_client = FollowTrajectoryClient('arm_controller/follow_joint_trajectory', arm_joint_names, config)



    data = np.load('fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_output_traj.npy')

    arm_client.move_to(data, False)
