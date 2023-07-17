function plot_3Dsurf(fig_num, z_array, mapz_range)
% plot_surf2( ksx_g,ksy_g,reshape(est_s2,[size(ksx_g,1),size(ksx_g,2)]), map_x, map_y, [0 10],25, 5);
figure(fig_num);
% 创建 x、y 的网格
[width, length] = size(z_array);

[x, y] = meshgrid(0:length-1, 0:width-1);
% 绘制三维曲面
surf(x, y, z_array, 'FaceColor', 'interp', 'EdgeColor', 'none');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Surface');
% 设置颜色映射范围
clim(mapz_range);

% 添加颜色栏
colorbar;


end