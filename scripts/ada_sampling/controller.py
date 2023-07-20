from double_integrator import DoubleIntegrator
from ergodic_control import RTErgodicControl
from target_dist import TargetDist
from target_dist_t import TargetDist_t

from utils import convert_phi2phik, convert_ck2dist, convert_traj2ck, convert_phik2phi
import numpy as np
from scipy.io import loadmat



class Controller(object): # python (x,y) therefore col index first, row next
    # robot state + controller !
    def __init__(self, start_position, row=40, col=20):
        self.row = row
        self.col = col
        self.location = [start_position[0]/self.col, start_position[1]/self.row]
        self.robot_dynamic = DoubleIntegrator() # robot controller system
        self.model = DoubleIntegrator()
        
        self.robot_state     = np.zeros(self.robot_dynamic.observation_space.shape[0])
        self.robot_state[:2] = np.array(self.location)

        self.erg_ctrl    = RTErgodicControl(self.model, horizon=15, num_basis=5, batch_size=-1)
    
    def get_nextpts(self, phi_vals): 
        setpoints = []
        # setting the phik on the ergodic controller
        self.erg_ctrl.phik = convert_phi2phik(erg_ctrl.basis, t_dist.grid_vals, t_dist.grid)
        ctrl = self.erg_ctrl(self.robot_state)
        self.robot_state = self.robot_dynamic.step(ctrl)
        setpoints.append([state[0], state[1]])
        return setpoints
        
   