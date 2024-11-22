
import numpy as np

data = np.load('fetch_joint_controls/scripts/center_box_avoidance_opt_info.npy', allow_pickle=True)
fail_flag = data.item().get('fail_flag')
print(data)

flag=0

a = np.array([0,0,0,0,0,0,0])
a[:6] = np.array([1,1,1,1,1,1])
print(np.shape(a))

if fail_flag[-1] == 0: 
    print("yaay")