#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

from mpl_toolkits.mplot3d import Axes3D
import scipy.io as sio
import sys

THREED = False

class Map:
    def __init__(self, length, width, grid_cell_size=10, fov_radius=100):
        self.length = int(length/grid_cell_size)
        self.width = int(width/grid_cell_size)
        self.map = np.ones((self.length, self.width))
        self.grid_cell_size = int(grid_cell_size)
        self.fov_radius = fov_radius/self.grid_cell_size
        self.coordiates = None

    # def generate_kernel(self):
    #     self.kernel = 
    
    def update_map(self, coordinates):
        self.coordiates = coordinates
        self.map.fill(1)  # 重置地图为全1
        
        targets_on_map = []
        for coord in coordinates:
            temp_map = np.ones((self.length, self.width))
            
            [ship_x_real, ship_y_real] = coord
            ship_grid_x_real, ship_grid_y_real = ship_x_real/self.grid_cell_size, ship_y_real/self.grid_cell_size
            
            # calculat range
            grid_x_min = max(int(ship_grid_x_real - self.fov_radius), 0)
            grid_x_max = min(int(ship_grid_x_real + self.fov_radius)+1, self.length-1)
            grid_y_min = max(int(ship_grid_y_real - self.fov_radius), 0)
            grid_y_max = min(int(ship_grid_y_real + self.fov_radius)+1, self.width-1)
            for x in range(grid_x_min, grid_x_max):
                for y in range(grid_y_min, grid_y_max):
                    distance = np.sqrt((x - ship_grid_x_real)**2 + (y - ship_grid_y_real)**2)
                    if distance < self.fov_radius:                        
                        temp_map[x, y] = distance / self.fov_radius
            self.map = np.multiply(self.map, temp_map)
            
            targets_on_map.append([coord[0]/self.grid_cell_size, coord[1]/self.grid_cell_size])
            
        return self.map, targets_on_map

    def print_map(self):
        # self.ax.plot_surface(self.x, self.y, self.map, cmap='viridis', alpha=0.8, rstride=1, cstride=1, shade=False)
        # self.ax.contourf(self.x, self.y, self.map, cmap='viridis')

        if (THREED):
            ax.plot_surface(self.x, self.y, self.map, cmap='viridis', alpha=0.8, rstride=1, cstride=1, shade=False)
        else:
            mesh = self.ax.pcolormesh(self.x, self.y, self.map, cmap='viridis')
            plt.colorbar(mesh)

        # meshgrid length and width
        self.x, self.y = np.meshgrid(np.linspace(0, width, int(width/grid_cell_size)), np.linspace(0, length, int(length/grid_cell_size)))

        # smooth kenerl function
        
        if (THREED):
            ax = plt.axes(projection='3d')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            ax.set_title('Distance Map')
            
            ax.set_box_aspect([1, 2, 0.5])
            
            x_major_locator = MultipleLocator(1000)
            ax.xaxis.set_major_locator(x_major_locator)
            y_major_locator = MultipleLocator(1000)
            ax.yaxis.set_major_locator(y_major_locator)
            
        else:
            ax = plt.axes()
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_title('smoothed_map')


            
        x_points = [coord[0]+10 for coord in self.coordiates[:]]
        y_points = [coord[1]+10 for coord in self.coordiates[:]]
        z_points = np.zeros_like(x_points)
        if (THREED):
            self.ax.scatter(y_points, x_points, z_points, color='red', s=10)
            
        plt.show()

        # plt.draw()
        # plt.pause(0.1)
        
    

if __name__ == '__main__':
   
    with open('ship_trajectory.txt', 'r') as file:
        lines = file.readlines()
        
        for line in lines:
            line = line.strip()
            
            if line == '':  # new time step
                map, targets_on_map = Map_output.update_map(current_list)
                
                break
            else:  # 解析列表的行数据
                sublist = [float(item) for item in line.split()]
                current_list.append(sublist)
                
    if WRITE:     
        sio.savemat('ship_trajectory.mat', data_dict, appendmat=False)
        print("Successfully saved")