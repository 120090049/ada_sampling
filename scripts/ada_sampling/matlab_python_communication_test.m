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

% initialize the class
contoller_bot1 = py.controller.Controller(py.list([2,5]), 40, 20);

% test data
load('ship_trajectory.mat'); % F_map(40,20) map_length map_width targets(8*2)
Fss = F_map(:);
% run ergodic search algorithm
set_points = double(contoller_bot1.get_nextpts(Fss));
set_points