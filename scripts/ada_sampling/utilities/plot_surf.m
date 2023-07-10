function plot_surf(x,y,z, fontsize, linsize)

if nargin < 4
    fontsize = 25;  % 25
    linsize = 5;  % 5    
end

% get the corners of the domain in which the data occurs.
min_x = min(min(x));
min_y = min(min(y));
max_x = max(max(x));
max_y = max(max(y));
 
% the image data you want to show as a plane.
planeimg = abs(z);
 
% set hold on so we can show multiple plots / surfs in the figure.

 
% do a normal surface plot.
% surf(x,y,z);
 
% set a colormap (but this has no effect because the next colormap
% command overwrites it)
colormap(gray);
 
% desired z position of the image plane.
imgzposition = 0;
 
% plot the image plane using surf.
surf([min_x max_x],[min_y max_y],repmat(imgzposition, [2 2]),...
    planeimg,'facecolor','texture')
 
% set a colormap for the figure.
colormap(jet);
view(0,90);

xlim([0 42]);
ylim([0 32]);

xlabel('X','FontSize',fontsize);
ylabel('Y','FontSize',fontsize);
zlabel('Temperature (degrees Celsius)','FontSize',fontsize);
caxis([15,23]); %29
set(gca,'LineWidth',linsize,'fontsize',fontsize,'fontname','Times','FontWeight','Normal');
% grid on;
colorbar;

end