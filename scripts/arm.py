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


def update_trajectory_with_optimizer(desired_position, initial_position, max_vel, total_time):
    
    
    detailed_trajectory = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_detailed_traj.npy', allow_pickle=True)
    initial_position = detailed_trajectory.item().get('q')[0]
    initial_velocity = detailed_trajectory.item().get('qd')[0]

    dt = 0.5
    num_joints = 7
    current_time = np.zeros((1,num_joints))
    position = np.zeros((1, num_joints))   
    position[0,:6] = initial_position
    print(position)
    velocity = np.zeros((1, num_joints))  
    velocity[0,:6] = initial_velocity
    print(velocity)
    acceleration = np.zeros((1, num_joints)) 
    print(acceleration)
    # Start loop to periodically get optimizer data and update trajectory
    while True:
        """
        TODO: 
        - make a rosnode to publish the data from the optimizer
        - data to be published for each discrete time interval and not just dump the data 
        - subscribe to the node at 0.5 sec
        - record video of the points
        - compare with the original trajectory 
        - code to match with the repo and make it pretty.
        """
        optimizer_data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_opt_info.npy', allow_pickle=True)
        print(optimizer_data)
        fail_flag = optimizer_data.item().get('fail_flag')

        if fail_flag[-1] != 0:
            acceleration[:6] = optimizer_data.item().get('ka')  # Update acceleration from optimizer
        # else:
        #     velocity = np.zeros((1,num_joints))  # Stop robot if no data - breaking
        print(np.shape(position))
        print(np.shape(velocity))
        print(np.shape(acceleration))
        print(velocity)
        print(position)

        # Update position and velocity using piecewise linear acceleration
        velocity += dt * acceleration   # v = u + at
        position += velocity * dt  # s = s + v*t

        """
        TODO : DONE [11/21/24]
        - concatenate these objects and make it similiar to the waypoints matrix 
        - implement move to at every second
        """

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

    while not (arm_client.ready and torso_client.ready):
        rospy.sleep(0.1)

    # # Move torso up!
    # torso_time = np.ones((1, 1, torso_client.config['joints_num'])) * 4
    # torso_goal_pos = np.ones((1, 1, torso_client.config['joints_num'])) * 0.3
    # torso_goal_vel = np.ones((1, 1, torso_client.config['joints_num'])) * 0.0
    # torso_waypoints = np.concatenate([torso_time, torso_goal_pos, torso_goal_vel], axis=1)
    # rospy.loginfo("Raising torso...")
    # torso_waypoints = np.concatenate([torso_client.current_waypoint(), torso_waypoints], axis=0)
    # #torso_client.move_to(torso_waypoints, False)

    # # Move ARM to save position
    # arm_waypoints = arm_client.create_zero_waypoint()
    # arm_waypoints[:, 0, :] = 5.
    # arm_waypoints[:, 1, :] = np.array([1.32, 0, -1.57, 1.72, 0.0, 1.66, 0.0])
    # arm_waypoints[:, 2, :] = 0.
    # rospy.loginfo("Moving arm to safe position...")
    # arm_waypoints = np.concatenate([arm_client.current_waypoint(), arm_waypoints], axis=0)
    # #arm_client.move_to(arm_waypoints, False)

    arm_waypoints = arm_client.create_zero_waypoint()
    arm_waypoints[:, 0, :] = 4.
    # print(arm_waypoints[:,0,:])
    arm_waypoints[:, 1, :] = np.array([0, 0, -1.57, 1.72, 0.0, 0.0, 0.0])
    # print(arm_waypoints[:,1,:])
    arm_waypoints[:, 2, :] = 0.
    # print(arm_waypoints)
    arm_waypoints = np.concatenate([arm_client.current_waypoint(), arm_waypoints], axis=0)
    # print(np.shape(arm_waypoints))

    arm_client.move_to(arm_waypoints, False)

    # # Move joints independently using simulation!!
    # rospy.loginfo("Moving joints independently...")
    # waypoints = test_joints_independently(arm_client)
    # arm_client.move_to(waypoints, False)

    # print("ARM STATE: ", arm_client.state)
    # # input("Press any key")

    xi = 0.
    xf = 1.48353
    JOINT_IDX = 6
    TOTAL_TIME = 8
    rospy.loginfo(f"Moving joint {JOINT_IDX} following generated trajectory")
    arm_client.create_recording()
    arm_client.start_thread()
    t, pos, vel, accln = generate_trajectory(desired_position=xf, initial_position=xi, max_vel=1., total_time=TOTAL_TIME)
    # print(t,pos,vel, accln)
    # waypoints = arm_client.current_waypoint(n=t.shape[0])
    # waypoints[:, 0, :] = t[:, np.newaxis]
    # waypoints[:, 1, JOINT_IDX] = pos
    # waypoints[:, 2, JOINT_IDX] = vel
    # # print(np.shape(waypoints))
    # arm_client.move_to(waypoints, False)
    # arm_client.stop_thread()

    # plot_trajectories(t, pos, vel, arm_client.time_recordings, arm_client.state_recordings[JOINT_IDX, :], arm_client.state_recordings[len(arm_client.joint_names) + JOINT_IDX, :])

    arm_client.create_recording()
    arm_client.start_thread()
    t, pos, vel, accln = update_trajectory_with_optimizer(desired_position=xf, initial_position=xi, max_vel=1., total_time=TOTAL_TIME / 2)
    # waypoints = arm_client.current_waypoint(n=t.shape[0])
    # waypoints[:, 0, :] = t[:, np.newaxis]
    # waypoints[:, 1, JOINT_IDX] = pos
    # waypoints[:, 2, JOINT_IDX] = vel
    # waypoints[:, 3, JOINT_IDX] = accln
    # arm_client.move_to(waypoints, False)
    # arm_client.stop_thread()

    # plot_trajectories(t, pos, vel, arm_client.time_recordings, arm_client.state_recordings[JOINT_IDX, :], arm_client.state_recordings[len(arm_client.joint_names) + JOINT_IDX, :])
    

    