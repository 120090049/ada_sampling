
% get python path (not necessary)

% P = py.sys.path; 
% if count(P,'/home/clp/catkin_ws/src/ada_sampling/scripts/ada_sampling/controller.py') == 0
%    insert(P,int32(0),'/home/clp/catkin_ws/src/ada_sampling/scripts/ada_sampling/controller.py');
% end

% import py file
py.importlib.import_module('controller');

% renew the cache for python
clear classes;
obj = py.importlib.import_module('controller');
py.importlib.reload(obj);



% test data
% load('ship_trajectory_old_40_20.mat');
load('ship_trajectory.mat'); % F_map(40,20) map_length map_width targets(8*2)
[Xss, ksx_g, ksy_g] = generate_coordinates(map_length, map_width);
Fss = F_map(:);
map_x = map_width; % 20
map_y = map_length; % 40
map_z = [0,1];

% initialize the class
contoller_bot1 = py.controller.Controller(py.list([2,5]), map_length, map_width);

% run ergodic search algorithm
for i = 1:200
    reshape_list = reshape(Fss, [size(ksx_g,1),size(ksx_g,2)])';
    Fss_for_py = reshape_list(:);
    set_points = double(contoller_bot1.get_nextpts(Fss_for_py));
    % plot_3Dsurf(1, reshape(Fss,[size(ksx_g,1),size(ksx_g,2)]), [0,1]);
    lineStyles = linspecer(10);

    plot_surf2( ksx_g,ksy_g,reshape(Fss,[size(ksx_g,1),size(ksx_g,2)]), map_x, map_y, map_z,25, 5);
    hold on;
    for i = 1:size(set_points,1)
        plot(set_points(i,1), set_points(i,2),'o','MarkerSize',5,'LineWidth',5,'Color',lineStyles(2,:));
    end
    pause(0.1);
end

function [Xss, ksx_g, ksy_g] = generate_coordinates(length, width)
    length = double(length);
    width = double(width);
    [X, Y] = meshgrid(0:width-1, 0:length-1);
    Xss = [X(:), Y(:)];
    ksx_g = repmat(0:width-1, length, 1);
    ksy_g = repmat(0:length-1, width, 1)';
end
