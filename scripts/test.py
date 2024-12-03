
import numpy as np


P = np.array([-7.85391618e-01,  1.00065447e-02, -1.22795596e-09, -1.16367514e-07, 0.00000000e+00,  5.00065426e-02])

V = np.array([ 1.30899533e-03,  1.30893494e-03, -2.45591220e-07, -2.32735029e-05, 0.00000000e+00,  1.30852094e-03])

dt = 0.5

data = np.load('fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_opt_info.npy', allow_pickle=True)
#data = np.load('/home/marzuk/catkin_ws/src/fetch_joint_controls/scripts/center_box_avoidance_pi_24/center_box_avoidance_pi_24_detailed_traj.npy', allow_pickle=True)
# print(V)
# for i in range(20):
data_i =  data.item().get('ka')
# print(data_i)

for i in range(len(data_i)):
    A = data_i[i]
    
    P += (V * dt )+( 0.5 * A * (dt**2))

    V += A * dt   


    print(P)