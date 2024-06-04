import numpy as np
import matplotlib.pyplot as plt


def generate_trajectory(desired_position, initial_position, max_vel, total_time):
    # Create time array
    t = np.linspace(0, total_time, num=int(100*total_time))  # Creating a 100 samples per second time array

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
    
    # Clip maximum velocity and recompute with new time if necessary
    if np.max(np.abs(velocity)) > max_vel:
        print("Trimimng velocity")
        max_acc = np.max(np.abs(acceleration))
        total_time = (desired_position - initial_position) / max_vel + max_vel / max_acc
        return generate_trajectory(desired_position, initial_position, max_vel, total_time)
    
    return t, position, velocity


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
    ax1.plot(t_des, x_des, 'r-', label='Desired')
    ax1.plot(t_real, x_real, 'b-', label='Real')
    ax1.set_title('Position vs. Time')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (rad)')
    ax1.legend(loc='best')
    ax1.grid(True)

    # Plot velocity
    ax2.plot(t_des, v_des, 'r-', label='Desired')
    ax2.plot(t_real, v_real, 'b-', label='Real')
    ax2.set_title('Angular velocity vs. Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.legend(loc='best')
    ax2.grid(True)

    # Adjust layout to prevent overlap
    plt.tight_layout()

    # Display the plots
    plt.show()