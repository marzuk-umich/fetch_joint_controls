import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the CSV file
df = pd.read_csv('/home/marzuk/catkin_ws/src/fetch_joint_controls/test/robot_trajectory.csv')  # Replace with your filename

# Extract columns
x = df['x'].to_numpy() 
y = df['y'].to_numpy() 
theta = df['theta'].to_numpy()

# Compute arrow directions from theta
u = np.cos(theta)
v = np.sin(theta)

# Plot the trajectory
plt.figure(figsize=(8, 6))
plt.plot(x, y, 'r.-', label='Trajectory')  # Path

# Plot orientation arrows
plt.quiver(x, y, u, v, angles='xy', scale_units='xy', scale=5, color='blue', width=0.003, label='Orientation')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Trajectory with Orientation')
plt.axis('equal')
plt.legend()
plt.show()
