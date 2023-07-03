import numpy as np
import matplotlib.pyplot as plt
from bezier_curve_generator import BezierCurveGenerator
import random

class ShipTrajectoryGenerator:
    def __init__(self, interest_pt_list, curvity):
        self.position = None
        self.position_index = 0
        self.trajectory = None
        self.curvity = curvity
        self.interest_pt_list = interest_pt_list
        self.generate_init_pos()
        self.generate_trajectory()
        
    
    def generate_init_pos(self): # generate the random inital position of the ship (on the ship route)
        num_points = len(self.interest_pt_list)
        if num_points < 2:
            raise ValueError("At least two points are required.")

        # 随机选择两个相邻的点
        point1 = self.interest_pt_list[np.random.randint(num_points - 1)]
        point2 = self.interest_pt_list[np.random.randint(num_points - 1)]

        # 在两个点之间生成随机权重
        weight = np.random.uniform(0, 1)

        # 计算线性插值得到随机点的坐标
        self.position = (1 - weight) * point1 + weight * point2
    
    def generate_trajectory(self):
        # choose next interest point randomly
        if (self.position in self.interest_pt_list):
            index = self.interest_pt_list.index(self.position)
            remaining_items = self.interest_pt_list[:index] + self.interest_pt_list[index+1:]
            interestPT = random.choice(remaining_items)
        else:
            interestPT = random.choice(self.interest_pt_list)
            
        generator = BezierCurveGenerator(self.position, interestPT, self.curvity)

        self.trajectory = generator.generate_random_bezier_curve()

    
    def move(self):
        if self.position in self.interest_pt_list:
            self.generate_trajectory()
            self.trajectory = 0
        self.position_index += 1
        self.position = self.trajectory[self.position_index]


def main():
    
    harbourT = np.array([400, 400])
    interestPT1 = np.array([800, 1500])
    interestPT2 = np.array([3000, 1000])
    tourboat_visit_pts = [harbourT, interestPT1, interestPT2]
    
    ship_tra_controller = ShipTrajectoryGenerator(tourboat_visit_pts, curvity=0.3)
    while True:
        
        

        
if __name__ == '__main__':
    main()
