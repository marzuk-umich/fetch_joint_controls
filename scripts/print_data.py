
import numpy as np

data = np.load('fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_opt_info.npy', allow_pickle=True)
fail_flag = data.item().get('q')
print(data)
print
flag=0

# a = np.array([0,0,0,0,0,0,0])
# a[:6] = np.array([1,1,1,1,1,1])
# print(np.shape(a))

# if fail_flag[-1] == 0: 
#     print("yaay")