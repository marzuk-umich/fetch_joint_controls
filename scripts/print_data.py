
import numpy as np
import time


# data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_opt_info.npy', allow_pickle=True)
# data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_opt_info.npy', allow_pickle=True)
data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/tracking_test2_pi_24/tracking_test2_pi_24_opt_info.npy', allow_pickle=True)
data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/tracking_test2_pi_24/tracking_test2_pi_24_output_traj.npy', allow_pickle=True)
# data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/tracking_test2_pi_24/tracking_test2_pi_24_detailed_traj.npy', allow_pickle=True)


# for i in range(0, 2000, 50):  # start at 0, go to 2000, step by 40
#     print(data.item().get('q')[i, 0])
# print(data.item().get('ka').shape)
# for i in range(0, 40):  # start at 0, go to 2000, step by 40
#     print(data.item().get('ka')[i, 4])

# for i in range(0, 40):  # start at 0, go to 2000, step by 40
#     print(data[i,1,0])
print(data)