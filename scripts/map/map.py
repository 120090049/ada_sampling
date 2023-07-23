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
        # meshgrid length and width
        self.x, self.y = np.meshgrid(np.linspace(0, width, int(width/grid_cell_size)), np.linspace(0, length, int(length/grid_cell_size)))

        # smooth kenerl function
        
        if (THREED):
            self.ax = plt.axes(projection='3d')
            
        else:
            self.ax = plt.axes()
            mesh = self.ax.pcolormesh(self.x, self.y, self.map, cmap='viridis', vmin=0, vmax=2)
            plt.colorbar(mesh)

        plt.grid(True)
        plt.ion()  # interactive mode on!!!! 很重要，有了它就不需要 plt.show() 了

    # def generate_kernel(self):
    #     self.kernel = 
    
    def update_map(self, coordinates):
        self.coordiates = coordinates
        self.map.fill(0)  # 重置地图为全1
        
        targets_on_map = []
        for coord in coordinates:
            temp_map = np.zeros((self.length, self.width))
            
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
                        d = 1 - distance / self.fov_radius                      
                        # Apply the smoothstep formula
                        temp_map[x, y] = d * d * (3 - 2 * d)
            self.map += temp_map
            
            targets_on_map.append([coord[0]/self.grid_cell_size, coord[1]/self.grid_cell_size])
            
        return self.map, targets_on_map

    def print_map(self):
        self.ax.cla()  # 清除上一时刻的曲面
        # self.ax.plot_surface(self.x, self.y, self.map, cmap='viridis', alpha=0.8, rstride=1, cstride=1, shade=False)
        # self.ax.contourf(self.x, self.y, self.map, cmap='viridis')

        if (THREED):
            self.ax.plot_surface(self.x, self.y, self.map, cmap='viridis', alpha=0.8, rstride=1, cstride=1, shade=False)
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_zlabel('Z')
            
            # self.ax.set_box_aspect([1, 2, 0.5])
            x_major_locator = MultipleLocator(500)
            self.ax.xaxis.set_major_locator(x_major_locator)
            y_major_locator = MultipleLocator(500)
            self.ax.yaxis.set_major_locator(y_major_locator)
            z_major_locator = MultipleLocator(0.5)
            self.ax.zaxis.set_major_locator(z_major_locator)
            
            self.ax.set_zlim(0, 2)
            self.ax.set_title('Distance Map')
        else:
            self.ax.pcolormesh(self.x, self.y, self.map, cmap='viridis', shading='auto', vmin=0, vmax=2)
            self.ax.set_xlabel('x')
            self.ax.set_ylabel('y')
            self.ax.set_title('smoothed_map')
            

            
        x_points = [coord[0]+10 for coord in self.coordiates[:]]
        y_points = [coord[1]+10 for coord in self.coordiates[:]]
        z_points = np.zeros_like(x_points)
        if (THREED):
            self.ax.scatter(y_points, x_points, z_points, color='red', s=10)
        else:
            self.ax.scatter(y_points, x_points, color='red', s=1)


        plt.draw()
        plt.pause(0.01)
        
    

if __name__ == '__main__':
    WRITE = False
    print("with argument = 1: write data; no argmuent just show 3D map (no data writing)")
    if len(sys.argv) > 1:
        number = int(sys.argv[1])
        if number == 1:
            WRITE = True
        else:
            WRITE = False
    current_list = []
    Map_output = Map(length=4000, width=2000, grid_cell_size=20, fov_radius=300)
    data_dict = {'map_length': Map_output.length, 'map_width': Map_output.width}
    
    
    with open('ship_trajectory.txt', 'r') as file:
        lines = file.readlines()
        
        for line in lines:
            line = line.strip()
            
            if line == '':  # new time step
                map, targets_on_map = Map_output.update_map(current_list)
                # map = 1 - map
                data_dict['F_map'] = map
                data_dict['targets'] = targets_on_map

                # print(map.shape)
                # print(targets_on_map)
                Map_output.print_map()
                current_list = []
                plt.pause(5)
                break
            else:  # 解析列表的行数据
                sublist = [float(item) for item in line.split()]
                current_list.append(sublist)
                
    if WRITE:     
        sio.savemat('ship_trajectory.mat', data_dict, appendmat=False)
        print("Successfully saved")