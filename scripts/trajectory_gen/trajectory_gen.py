import numpy as np
import matplotlib.pyplot as plt
from bezier_curve_generator import BezierCurveGenerator
import random, time
import math

class ShipTrajectoryGenerator:
    def __init__(self, interest_pt_list, wind, wave, randomness=0.05, frequency = 10):
        self.position = None
        self.position_index = 0
        self.trajectory = None
        self.randomness = randomness
        self.interest_pt_list = interest_pt_list
        self.pre_interestPT = None
        
        self.control_pt = np.array(wind) + np.array(wave)
        
        self.frequency = frequency # recommend 0.5 - 10 higher the better
        self.generate_init_pos()
        
    
    def generate_init_pos(self): # generate the random inital position of the ship (on the ship route)
        num_points = len(self.interest_pt_list)
        if num_points < 2:
            raise ValueError("At least two points are required.")

        # 随机选择两个相邻的点
        point1 = self.interest_pt_list[np.random.randint(num_points - 1)]
        while (True):
            point2 = self.interest_pt_list[np.random.randint(num_points - 1)]
            if np.array_equal(point1, point2):
                break
        weight = np.random.uniform(0, 1)

        # 计算线性插值得到随机点的坐标
        self.position = (1 - weight) * point1 + weight * point2
        generator = BezierCurveGenerator(self.position, point2, self.control_pt, self.randomness, self.frequency)
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
        generator = BezierCurveGenerator(self.position, destination, self.control_pt, self.randomness, self.frequency)

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
    interestPT1 = np.array([800, 1500])
    interestPT2 = np.array([3000, 1000])
    
    harbourF1 = np.array([2500, 200])
    harbourF2 = np.array([3500, 200])
    interestPF1 = np.array([2500, 1600])
    
    tourboat_visit_pts = [harbourT1, interestPT1, interestPT2]
    fishboat_visit_pts = [harbourF1, harbourF1, interestPF1]
    
    wind=[0,1]
    wave=[1,0]
    
    frequency = 0.1
    randomness= 0.1
    shipT1 = ShipTrajectoryGenerator(tourboat_visit_pts, wind, wave, randomness, frequency)
    shipT2 = ShipTrajectoryGenerator(tourboat_visit_pts, wind, wave, randomness, frequency)
    shipT3 = ShipTrajectoryGenerator(tourboat_visit_pts, wind, wave, randomness, frequency)
    
    
    shipF1 = ShipTrajectoryGenerator(fishboat_visit_pts, wind, wave, randomness, frequency)
    shipF2 = ShipTrajectoryGenerator(fishboat_visit_pts, wind, wave, randomness, frequency)
    shipF3 = ShipTrajectoryGenerator(fishboat_visit_pts, wind, wave, randomness, frequency)
    
    fig, ax = plt.subplots()

    # 设置坐标轴的范围
    ax.set_xlim(0, 4000)
    ax.set_ylim(0, 2000)
    
    # wind and wave direction
    ax.arrow(200, 200, 150*wind[0], 150*wind[1], width=1, color='red')
    ax.text(200, 200, "Wind", ha='right', va='bottom', color='red')
    ax.arrow(200, 200, 150*wave[0], 150*wave[1], width=1, color='blue')
    ax.text(200, 200, "Wave", ha='right', va='top', color='blue')
    
    # harbour and interest points
    ax.plot(interestPT1[0], interestPT1[1], 'ro', label='interestPT1')
    ax.plot(interestPT2[0], interestPT2[1], 'ro', label='interestPT2')
    ax.plot(harbourT1[0], harbourT1[1], 'ro', label='harbourT1')
    
    ax.plot(harbourF1[0], harbourF1[1], 'bo', label='harbourF1')
    ax.plot(harbourF2[0], harbourF2[1], 'bo', label='harbourF2')
    ax.plot(interestPF1[0], interestPF1[1], 'ro', label='interestPF1')
    

    # 设置初始点的坐标
    T1_x, T1_y, _ = shipT1.move()
    T2_x, T2_y, _ = shipT2.move()
    T3_x, T3_y, _ = shipT3.move()
    # 设置时间步长和总时间
    dt = 0.01

    while True:
        new_T1_x, new_T1_y, _ = shipT1.move()
        new_T2_x, new_T2_y, _ = shipT2.move()
        new_T3_x, new_T3_y, _ = shipT3.move()
        ax.plot([T1_x, new_T1_x], [T1_y, new_T1_y], 'b')
        ax.plot([T2_x, new_T2_x], [T2_y, new_T2_y], 'y')
        ax.plot([T3_x, new_T3_x], [T3_y, new_T3_y], 'g')
        # 更新当前点的坐标
        T1_x, T1_y = new_T1_x, new_T1_y
        T2_x, T2_y = new_T2_x, new_T2_y
        T3_x, T3_y = new_T3_x, new_T3_y
        
        # 刷新图形窗口
        plt.pause(dt)

    plt.show()
        

        
if __name__ == '__main__':
    main()
