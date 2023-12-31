% 假设 set_points 是一个5x2的数组
% 例如：
% set_points = [1.25, 2.75;
%               3.60, 4.10;
%               1.50, 2.60;
%               1.25, 2.75;
%               2.30, 3.45];

% 步骤 1: 四舍五入数组中的所有元素
rounded_points = round(set_points);

% 步骤 2: 删除重复的行
unique_points = unique(rounded_points, 'rows');

disp('原始数组 set_points:');
disp(set_points);

disp('四舍五入后的数组 rounded_points:');
disp(rounded_points);

disp('去重后的数组 unique_points:');
disp(unique_points);