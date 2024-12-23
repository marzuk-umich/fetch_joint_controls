#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.interpolate import interp1d







# # Parameters for the trajectory
# desired_position = np.ones((7, 1)) * 10  # Example desired positions
# initial_position = np.zeros((7, 1))  # Start at zero
# max_vel = 2  # Maximum velocity (example)
# total_time = 10  # Total time in seconds

# update_trajectory_with_optimizer(desired_position, initial_position, max_vel, total_time)


def poly5(x, a, b, c, d, e, f):
    return a*x**5 + b*x**4 + c*x**3 + d*x**2 + e*x + f

def generate_smoothened_trajectory(time_data, position_data, velocity_data, acceleration_data):

    time = time_data[:, 0]

    # Create a finer time grid
    time_fine = np.linspace(time.min(), time.max(), num=1000)

    # Curve fit each joint and plot the position-time graph
    joints = position_data.shape[1]

    fine_tuned_time_data = np.tile(time_fine, (7, 1)).T
    fine_tuned_position_data = np.zeros((len(time_fine), 7))
    velocity_data_fine = np.zeros((len(time_fine)-1, 7))
    acceleration_data_fine = np.zeros((len(time_fine)-2, 7))
    zeros_row = np.zeros((1, 7))



    plt.figure(figsize=(14, 8))
    for joint in range(joints):
        # Interpolate the position data
        interp_func = interp1d(time, position_data[:, joint], kind='cubic')
        position_fine = interp_func(time_fine)
        fine_tuned_position_data[:, joint] = position_fine

        # Calculate velocity (first derivative)
        velocity_data_fine[:, joint] = np.diff(fine_tuned_position_data[:, joint]) / np.diff(time_fine)
        print(np.shape(np.diff(time_fine[:-1])))

        # Calculate acceleration (second derivative)
        acceleration_data_fine[:, joint] = np.diff(velocity_data_fine[:, joint]) / np.diff(time_fine[:-1])

        # Store fine-tuned position data
        

        # Plot original, interpolated, and fitted data
        plt.subplot(3, 3, joint + 1)
        plt.plot(time, velocity_data[:, joint], 'o', label='Original Data', markersize=4)
        plt.plot(time_fine[:-1], velocity_data_fine[:, joint], '--', label='Interpolated', linewidth=1)
        plt.xlabel('Time (s)')
        plt.ylabel('Position (rad)')
        plt.title(f'Joint {joint}')
        plt.legend()
        plt.tight_layout()

    plt.suptitle("Position-Time Graphs with Discretization and Curve Fitting", y=1.02)
    plt.show()
    velocity_data_fine      = np.vstack((velocity_data_fine, zeros_row))
    acceleration_data_fine  = np.vstack((acceleration_data_fine, zeros_row))
    acceleration_data_fine  = np.vstack((acceleration_data_fine, zeros_row))


    return fine_tuned_time_data, fine_tuned_position_data, velocity_data_fine, acceleration_data_fine


def generate_trajectory(desired_position, initial_position, max_vel, total_time):

    # Create time array
    t = np.linspace(0, total_time, num=int(1*total_time))  # Creating a 100 samples per second time array

    # Calculating coefficients for a quintic polynomial
    # x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    # Using boundary conditions:
    # x(0) = initial_position, x(total_time) = desired_position, 
    # x_dot(0) = 0, x_dot(total_time) = 0, x_ddot(0) = 0, x_ddot(total_time) = 0
    A = np.array([
        [1, 0, 0, 0, 0, 0],
        [1, total_time, total_time**2, total_time**3, total_time**4, total_time**5],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 2*total_time, 3*total_time**2, 4*total_time**3, 5*total_time**4],
        [0, 0, 2, 0, 0, 0],
        [0, 0, 2, 6*total_time, 12*total_time**2, 20*total_time**3]
    ])
    # print(initial_position, desired_position)
    B = np.array([
        initial_position, 
        desired_position, 
        0,                  # initial vel
        0,                  # final vel
        0,                  # initial acc
        0                   # final acc
    ])
    coefficients = np.linalg.solve(A, B)
    
    # Generating position, velocity, and acceleration
    position = np.polyval(coefficients[::-1], t)
    velocity = np.polyval(np.polyder(coefficients[::-1], 1), t)
    acceleration = np.polyval(np.polyder(coefficients[::-1], 2), t)
    print("position:", np.shape(position))
    print("velocity:", np.shape(velocity))
    print("acceleration:", np.shape(acceleration))
    


    return t, position, velocity, acceleration



def test_joints_independently(arm_client):
    waypoints = arm_client.create_zero_waypoint()
    for i in range(arm_client.config['joints_num']):
        mid_waypoint = arm_client.create_zero_waypoint()
        mid_waypoint[0, 0, :] = i * 8 + 2
        mid_waypoint[0, 1, i] = 1.48353 / 4. # 85°
        mid_waypoint[0, 2, i] = 0.
        waypoint = arm_client.create_zero_waypoint()
        waypoint[0, 0, :] = i * 8 + 4
        waypoint[0, 1, i] = 1.48353 / 2. # 85°
        waypoint[0, 2, i] = 0.
        home = arm_client.create_zero_waypoint()
        home[0, 0, :] = i * 8 + 8
        waypoints = np.concatenate([waypoints, mid_waypoint, waypoint, home])
    return waypoints


def plot_trajectories(t_des, x_des, v_des, t_real, x_real, v_real):
    fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2, figsize=(10, 7))
    # Plot position
    ax1.plot(t_des, x_des, 'r--', label='Optimizer Data')
    ax1.plot(t_real, x_real, 'b--', label='Computed Data')
    ax1.set_title('Position vs. Time')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (rad)')
    ax1.legend(loc='best')
    ax1.grid(True)

    # Plot velocity
    ax2.plot(t_des, v_des,  'r--', label='Optimizer Data')
    ax2.plot(t_real, v_real, 'b--', label='Computed Data')
    ax2.set_title('Angular velocity vs. Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.legend(loc='best')
    ax2.grid(True)

    # Adjust layout to prevent overlap
    plt.tight_layout()

    # Display the plots
    plt.show()





def plot_trajectories_with_optimizer(waypoint, t_detailed, detailed_data, index):
        # Create subplots
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    axes = axes.flatten()

    # Joint 1
    ax = axes[0]
    time_data = waypoint[:, 0, 0]
    required_data = waypoint[:, index, 0]
    ax.plot(time_data, required_data, 'r--', label='Computed Data', color="red")
    ax.plot(t_detailed, detailed_data[:, 0], 'b--', label='Optimizer Data', color="blue")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (radians)')
    ax.set_title('Joint 1 Position vs Time')
    ax.legend()

    ax = axes[1]
    required_data = waypoint[:, index, 1]
    ax.plot(time_data, required_data, 'r--', label='Computed Data', color="red")
    ax.plot(t_detailed, detailed_data[:, 1], 'b--', label='Optimizer Data', color="blue")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (radians)')
    ax.set_title('Joint 2 Position vs Time')
    ax.legend()

    # Joint 3
    ax = axes[2]
    required_data = waypoint[:, index, 2]
    ax.plot(time_data, required_data, 'r--', label='Computed Data', color="red")
    ax.plot(t_detailed, detailed_data[:, 2], 'b--', label='Optimizer Data', color="blue")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (radians)')
    ax.set_title('Joint 3 Position vs Time')
    ax.legend()

    # Joint 4
    ax = axes[3]
    required_data = waypoint[:, index, 3]
    ax.plot(time_data, required_data, 'r--', label='Computed Data', color="red")
    ax.plot(t_detailed, detailed_data[:, 3], 'b--', label='Optimizer Data', color="blue")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (radians)')
    ax.set_title('Joint 4 Position vs Time')
    ax.legend()

    # Joint 5
    ax = axes[4]
    required_data = waypoint[:, index, 4]
    ax.plot(time_data, required_data, 'r--', label='Computed Data', color="red")
    ax.plot(t_detailed, detailed_data[:, 4], 'b--', label='Optimizer Data', color="blue")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (radians)')
    ax.set_title('Joint 5 Position vs Time')
    ax.legend()

    # Joint 6
    ax = axes[5]
    required_data = waypoint[:, index, 5]
    ax.plot(time_data, required_data, 'r--', label='Computed Data', color="red")
    ax.plot(t_detailed, detailed_data[:, 5], 'b--', label='Optimizer Data', color="blue")
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (radians)')
    ax.set_title('Joint 6 Position vs Time')
    ax.legend()

    # Adjust layout
    plt.tight_layout()
    plt.show()
