#!/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv
import math
import time

class WaypointFollower:
    def __init__(self):
        rospy.init_node('waypoint_follower', anonymous=True)

        # Load waypoints from CSV file
        self.waypoints = self.load_waypoints("/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/trajectory_plots/waypoints.csv")
        self.current_waypoint_idx = 0

        # Tuning parameters
        self.kp_linear = 0.5  # Proportional gain for position
        self.kp_angular = 2.0  # Proportional gain for rotation
        self.goal_tolerance = 0.1  # Distance tolerance to consider reaching a waypoint

        # Velocity limits
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s

        # Initialize velocity publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribe to odometry
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo(f"Loaded {len(self.waypoints)} waypoints.")

    def load_waypoints(self, filename):
        """ Load waypoints from a CSV file """
        waypoints = []
        try:
            with open(filename, 'r') as file:
                reader = csv.reader(file)
                header = next(reader)  # Skip header
                rospy.loginfo(f"CSV Header: {header}")  # Debugging output

                for row in reader:
                    rospy.loginfo(f"Reading row: {row}")  # Debugging output

                    # Ensure row has exactly 3 columns (X, Y, Theta)
                    if len(row) < 3:
                        rospy.logwarn(f"Skipping invalid row: {row}")
                        continue

                    # Convert values properly
                    x, y, theta = float(row[0]), float(row[1]), float(row[2])
                    waypoints.append((x, y, theta))

                rospy.loginfo("Waypoints loaded successfully.")
        except Exception as e:
            rospy.logerr(f"Error loading waypoints: {e}")
        return waypoints

    def odom_callback(self, msg):
        """ Callback function for odometry messages """
        if self.current_waypoint_idx >= len(self.waypoints):
            self.stop_robot()
            return

        # Get current position
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_theta = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

        # Get target waypoint
        target_x, target_y, target_theta = self.waypoints[self.current_waypoint_idx]

        # Compute errors
        error_x = target_x - current_x
        error_y = target_y - current_y
        distance_error = math.sqrt(error_x**2 + error_y**2)

        # Check if the waypoint is reached
        if distance_error < self.goal_tolerance:
            rospy.loginfo(f"Waypoint {self.current_waypoint_idx + 1} reached!")
            self.current_waypoint_idx += 1
            return

        # Compute desired heading
        desired_theta = math.atan2(error_y, error_x)
        error_theta = desired_theta - current_theta

        # Normalize angle error to [-pi, pi]
        error_theta = (error_theta + math.pi) % (2 * math.pi) - math.pi

        # Compute control commands
        linear_speed = self.kp_linear * distance_error
        angular_speed = self.kp_angular * error_theta

        # Apply speed limits
        linear_speed = max(-self.max_linear_speed, min(self.max_linear_speed, linear_speed))
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

        # Publish velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_speed
        cmd_msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(cmd_msg)

        rospy.loginfo(f"Moving to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)}")
        rospy.loginfo(f"Command: v={linear_speed:.2f} m/s, ω={angular_speed:.2f} rad/s")

    def get_yaw_from_quaternion(self, quat):
        """ Convert quaternion to yaw angle (theta) """
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y**2 + quat.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def stop_robot(self):
        """ Stops the robot when all waypoints are reached """
        cmd_msg = Twist()
        self.cmd_vel_pub.publish(cmd_msg)
        rospy.loginfo("All waypoints reached. Robot stopped.")

if __name__ == '__main__':
    follower = WaypointFollower()
    rospy.spin()

