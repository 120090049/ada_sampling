import sys
sys.path.append('./rt_erg_lib/')
from double_integrator import DoubleIntegrator
from ergodic_control import RTErgodicControl
from target_dist import TargetDist
from target_dist_t import TargetDist_t

from utils import convert_phi2phik, convert_ck2dist, convert_traj2ck, convert_phik2phi
import numpy as np
from scipy.io import loadmat

import matplotlib.pyplot as plt

import os

if __name__ == '__main__':
    env         = DoubleIntegrator() # robot controller system
    model       = DoubleIntegrator()
    if (0):
        t_dist      = TargetDist(10)
    else:
        data = loadmat('ship_trajectory.mat')
        distance_map = data['F_map']
        t_dist      = TargetDist_t(distance_map)
    
    erg_ctrl    = RTErgodicControl(model, horizon=15, num_basis=5, batch_size=-1)
    
    
    # setting the phik on the ergodic controller
    erg_ctrl.phik = convert_phi2phik(erg_ctrl.basis, t_dist.grid_vals, t_dist.grid)

    print('--- simulating ergodic coverage ---')
    log = {'trajectory' : []}
    tf = 200
    state = env.reset()

    plt.figure(1)
    xy, vals = t_dist.get_grid_spec()
    plt.title('ergodic coverage')
    plt.contourf(*xy, vals, levels=10)
    
    for t in range(tf):
        ctrl = erg_ctrl(state)
        state = env.step(ctrl)
        log['trajectory'].append(state)
        print(state[0], state[1])
        plt.scatter(state[0], state[1])
        plt.pause(0.001)  # 暂停绘图并刷新窗口

    print('--- finished simulating ---')
    xt = np.stack(log['trajectory'])
    # plt.scatter(xt[:tf,0], xt[:tf,1])
    path = xt[:tf,model.explr_idx]
    ck = convert_traj2ck(erg_ctrl.basis, path) # coeffieient 25
    val = convert_ck2dist(erg_ctrl.basis, ck, t_dist.grid) 

    plt.figure(2)
    plt.title('time averaged statistics')
    plt.contourf(*xy, val.reshape(40,20), levels=10)
    
    # plt.figure(3)
    # plt.title('Fourier reconstruction of target distribution')
    # phi = convert_phik2phi(erg_ctrl.basis, erg_ctrl.phik, t_dist.grid)
    # plt.contourf(*xy, phi.reshape(40,20), levels=10)

    plt.show()
