import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Map:
    def __init__(self, width, height, grid_cell_size=10, fov_radius=100):
        self.width = int(width/grid_cell_size)
        self.height = int(height/grid_cell_size)
        self.map = np.ones((self.width, self.height))
        self.grid_cell_size = int(grid_cell_size)
        self.fov_radius = fov_radius/self.grid_cell_size
        # meshgrid length and width
        self.x, self.y = np.meshgrid(np.linspace(0, height, int(height/grid_cell_size)), np.linspace(0, width, int(width/grid_cell_size)))

        # smooth kenerl function
        
        self.ax = plt.axes(projection='3d')
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_title('smoothed_map')

        plt.grid(True)
        plt.ion()  # interactive mode on!!!! 很重要，有了它就不需要 plt.show() 了

    # def generate_kernel(self):
    #     self.kernel = 
    
    def update_map(self, coordinates):
        self.map.fill(1)  # 重置地图为全1
        
        for coord in coordinates:
            temp_map = np.ones((self.width, self.height))
            
            [ship_x_real, ship_y_real] = coord
            ship_grid_x_real, ship_grid_y_real = ship_x_real/self.grid_cell_size, ship_y_real/self.grid_cell_size
            
            # calculat range
            grid_x_min = max(int(ship_grid_x_real - self.fov_radius), 0)
            grid_x_max = min(int(ship_grid_x_real + self.fov_radius)+1, self.width-1)
            grid_y_min = max(int(ship_grid_y_real - self.fov_radius), 0)
            grid_y_max = min(int(ship_grid_y_real + self.fov_radius)+1, self.height-1)
            for x in range(grid_x_min, grid_x_max):
                for y in range(grid_y_min, grid_y_max):
                    distance = np.sqrt((x - ship_grid_x_real)**2 + (y - ship_grid_y_real)**2)
                    if distance < self.fov_radius:                        
                        temp_map[x, y] = distance / self.fov_radius
            self.map = np.multiply(self.map, temp_map)
        return self.map

    def print_map(self):
        self.ax.cla()  # 清除上一时刻的曲面
        self.ax.plot_surface(self.x, self.y, self.map, cmap='viridis', alpha=0.8, rstride=1, cstride=1, shade=False)

        plt.draw()
        plt.pause(0.1)

if __name__ == '__main__':
    current_list = []
    
    Map_output = Map(width=4000, height=2000, grid_cell_size=20, fov_radius=300)
    with open('ship_trajectory.txt', 'r') as file:
        lines = file.readlines()
        
        for line in lines:
            line = line.strip()
            
            if line == '':  # new time step
                map = Map_output.update_map(current_list)
                Map_output.print_map()
                current_list = []
            else:  # 解析列表的行数据
                sublist = [float(item) for item in line.split()]
                current_list.append(sublist)