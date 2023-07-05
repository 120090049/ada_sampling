import numpy as np
import matplotlib.pyplot as plt
from bezier_curve_generator import BezierCurveGenerator
import random, time
import math

class ShipTrajectoryGenerator:
    def __init__(self, interest_pt_list, curvity, frequency):
        self.position = None
        self.position_index = 0
        self.trajectory = None
        self.curvity = curvity
        self.interest_pt_list = interest_pt_list
        self.pre_interestPT = None
        
        self.frequency = frequency # recommend 0.5 - 10 higher the better
        self.generate_init_pos()
        
    
    def generate_init_pos(self): # generate the random inital position of the ship (on the ship route)
        num_points = len(self.interest_pt_list)
        if num_points < 2:
            raise ValueError("At least two points are required.")

        # 随机选择两个相邻的点
        point1 = self.interest_pt_list[np.random.randint(num_points - 1)]
        point2 = self.interest_pt_list[np.random.randint(num_points - 1)]

        weight = np.random.uniform(0, 1)

        # 计算线性插值得到随机点的坐标
        self.position = (1 - weight) * point1 + weight * point2
        generator = BezierCurveGenerator(self.position, point2, self.curvity, self.frequency)
        self.pre_interestPT = point1
        self.trajectory = generator.generate_random_bezier_curve()

        
    
    def generate_trajectory(self):
        # choose the destination
        # if get to one point
        index1 = np.where(np.all(self.interest_pt_list == self.position, axis=1))[0][0]
        index2 = np.where(np.all(self.interest_pt_list == self.pre_interestPT, axis=1))[0][0]
        remaining_items = [value for index, value in enumerate(self.interest_pt_list) if index not in [index1, index2]]
        destination = random.choice(remaining_items)
    
        self.pre_interestPT = self.position
        # generate the curve
        generator = BezierCurveGenerator(self.position, destination, self.curvity, self.frequency)

        self.trajectory = generator.generate_random_bezier_curve()

    
    def move(self):
        if any(np.all(self.position == pt) for pt in self.interest_pt_list):
        # if self.position in self.interest_pt_list:
            self.generate_trajectory()
            self.position_index = 0
        self.position_index += 1
        pre_position = self.position
        self.position = self.trajectory[self.position_index]
        ship_yaw = math.atan2(self.position[1] - pre_position[1], self.position[0] - pre_position[0])
        return self.position[0], self.position[1], ship_yaw



def main():
    
    harbourT1 = np.array([400, 400])
    harbourT2 = np.array([2500, 200])
    interestPT1 = np.array([800, 1500])
    interestPT2 = np.array([3000, 1000])
    tourboat_visit_pts = [harbourT1, harbourT2, interestPT1, interestPT2]
    
    shipT1 = ShipTrajectoryGenerator(tourboat_visit_pts, curvity=0.05, frequency=0.05)
    shipT2 = ShipTrajectoryGenerator(tourboat_visit_pts, curvity=0.05, frequency=0.05)
    
    fig, ax = plt.subplots()

    # 设置坐标轴的范围
    ax.set_xlim(0, 4000)
    ax.set_ylim(0, 2000)
    
    ax.plot(interestPT1[0], interestPT1[1], 'ro', label='Interest Point 1')
    ax.plot(interestPT2[0], interestPT2[1], 'ro', label='Interest Point 2')
    ax.plot(harbourT1[0], harbourT1[1], 'ro', label='harbourT1')
    ax.plot(harbourT2[0], harbourT2[1], 'ro', label='harbourT2')
    

    # 设置初始点的坐标
    T1_x, T1_y, _ = shipT1.move()
    T2_x, T2_y, _ = shipT2.move()
    # 设置时间步长和总时间
    dt = 0.01

    while True:
        new_T1_x, new_T1_y = shipT1.move()
        new_T2_x, new_T2_y = shipT2.move()
        ax.plot([T1_x, new_T1_x], [T1_y, new_T1_y], 'b')
        ax.plot([T2_x, new_T2_x], [T2_y, new_T2_y], 'y')
        
        # 更新当前点的坐标
        T1_x, T1_y = new_T1_x, new_T1_y
        T2_x, T2_y = new_T2_x, new_T2_y
        
        # 刷新图形窗口
        plt.pause(dt)

    plt.show()
        

        
if __name__ == '__main__':
    main()
