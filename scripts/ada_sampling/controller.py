from double_integrator import DoubleIntegrator
from ergodic_control import RTErgodicControl
from target_dist import TargetDist
from target_dist_t import TargetDist_t

from utils import convert_phi2phik, convert_ck2dist, convert_traj2ck, convert_phik2phi
import numpy as np
from scipy.io import loadmat

import array

class Controller(object): # python (x,y) therefore col index first, row next
    # robot state + controller !
    def __init__(self, start_position, row=40, col=20):
        self.row = row
        self.col = col
        self.location = [start_position[0]/self.col, start_position[1]/self.row]
        grid_2_r_w = np.meshgrid(np.linspace(0, 1, int(self.col)), np.linspace(0, 1, int(self.row)))
        self.grid = np.c_[grid_2_r_w[0].ravel(), grid_2_r_w[1].ravel()]
        
        self.robot_dynamic = DoubleIntegrator() # robot controller system
        self.model = DoubleIntegrator()
        
        self.robot_state     = np.zeros(self.robot_dynamic.observation_space.shape[0])
        self.robot_state[:2] = np.array(self.location)
        self.robot_dynamic.reset(self.robot_state)
        
        self.erg_ctrl    = RTErgodicControl(self.model, horizon=15, num_basis=5, batch_size=-1)

        print("Controller Succcessfully Initialized! Initial position is: ", start_position)
    
    def get_nextpts(self, phi_vals): 
        plt.figure(1)
        plt.title('ergodic coverage')
        self.ax = plt.axes()
     
    def get_nextpts(self, phi_vals): 
        sample_steps = 10
        setpoints = []
        # setpoints = np.zeros((sample_steps, 2))
        # change phi_vals(column vector) from matlab orginization to python orginization 
        
        
        # setting the phik on the ergodic controller
        phi_vals = np.array(phi_vals)
        phi_vals /= np.sum(phi_vals)
        self.erg_ctrl.phik = convert_phi2phik(self.erg_ctrl.basis, phi_vals, self.grid)
        # pre_cord = None
        for i in range(sample_steps):
            ctrl = self.erg_ctrl(self.robot_state)
            self.robot_state = self.robot_dynamic.step(ctrl)
            # cord = [round(self.robot_state[0]*self.col), round(self.robot_state[1]*self.row)]
            # if pre_cord != cord:  
            #     setpoints.append([round(self.robot_state[0]*self.col), round(self.robot_state[1]*self.row) ])
            # pre_cord = cord  
                  
            setpoints.append([self.robot_state[0]*self.col, self.robot_state[1]*self.row ])
            # plt.scatter(self.robot_state[0], self.robot_state[1])
            # plt.pause(0.001)  # 暂停绘图并刷新窗口
        
        setpoints = np.array(setpoints)
        return setpoints
        
   