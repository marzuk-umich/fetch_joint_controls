
import numpy as np
import time


# data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_opt_info.npy', allow_pickle=True)
data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_opt_info.npy', allow_pickle=True)

# for i in range(20):
acceleration_data =  data.item().get('ka')[:,0]
print(len(acceleration_data))

from scipy.interpolate import interp1d

# Number of interpolated points between each original point
num_interpolated_points = 20

# New time array with original points and 20 interpolated points between each pair of original points
new_time = np.linspace(0, len(acceleration_data) - 1, len(acceleration_data) + (len(acceleration_data) - 1) * num_interpolated_points)

# Original time array for the acceleration data
original_time = np.arange(len(acceleration_data))

# Interpolate acceleration data while preserving original points
interpolator = interp1d(original_time, acceleration_data, kind='cubic')
interpolated_acceleration = interpolator(new_time)

print(interpolated_acceleration[:100])  # Preview the first 10 interpolated values
