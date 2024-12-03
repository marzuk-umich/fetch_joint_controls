
import numpy as np
import time


# data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_opt_info.npy', allow_pickle=True)
data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_detailed_traj.npy', allow_pickle=True)

# for i in range(20):
data_i =  data.item().get('q')[1:100]
print(data_i)

