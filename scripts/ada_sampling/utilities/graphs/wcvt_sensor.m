g_num = 9;
it_num = 25;
s_num = 1e6;
kp = 0.5;
kw = 50000;
%h = [-0.05;0.05;zeros(g_num-2,1)];
%h = (rand(g_num,1)-0.5)/10;
h = zeros(g_num,1);
wts = [0.01;zeros(g_num-1,1)];
h = h-max(h);
wts = wts - max(wts);
B = [1,0;0,1;zeros(7,2)];
K = 0.02*ones(2,g_num);

rng(50)
%
%  Initialize the generators.
%
g = rand ( 2, g_num )/1.5;
%g = randi([1 599],2,10)/100;
%
%  Carry out the iteration.
%
step = 1 : it_num;
e = nan ( it_num, g_num );
gm = nan ( it_num, 1 );
cf = nan(it_num, 1);

for it = 1 : it_num
    it
    %
    %  Compute the Delaunay triangle information T for the current nodes.
    %
    t = wdelaunay(g(1,:),g(2,:),wts);
    [PC,~] = powercenters(t,g(1,:),g(2,:),wts);
    %
    %  Display the Voronoi cells.
    %
        %
        subplot ( 1, 3, 1 );
        %figure;
        plot(PC(:,1), PC(:,2), 'r.');
        hold on;
        voronoi(g(1,:),g(2,:),'r--');
        powerdiagram(g(1,:),g(2,:),t,'g',wts);
        plot(g(1,:),g(2,:), 'r+');
        text(g(1,:)'+0.02,g(2,:)',int2str((1:g_num)'));
        title_string = sprintf ( 'Weighted Voronoi' );
        title ( title_string );
        axis equal
        axis ([  0.0, 1.0, 0.0, 1.0 ])
        axis square
        drawnow
        hold off;
        %
        %  Display the Delaunay triangulation.
        %
%         subplot ( 2, 2, 2 );
%         %figure;
%         trimesh (t,g(1,:),g(2,:),zeros(g_num,1), 'EdgeColor', 'r' )
%         hold on
%         scatter ( g(1,:), g(2,:), 'k.' )
%         text(g(1,:)',g(2,:)',int2str((1:g_num)'));
%         title_string = sprintf ( 'Weighted Delaunay, step %d', it );
%         title ( title_string );
%         axis ( [  0.0, 1.0, 0.0, 1.0 ] )
%         axis square
%         view ( 2 )
%         drawnow
%         hold off
    %
    %  Generate sample points.
    %  New option for fixed grid sampling.
    %
    s2 = floor ( sqrt ( s_num ) );
    s2 = max ( s2, 2 );
    s_num = s2 * s2;
    [ sx, sy ] = meshgrid ( linspace ( 0.0, 1.0, s2 ) );
    sx = reshape ( sx, 1, s2 * s2 );
    sy = reshape ( sy, 1, s2 * s2 );
    s = [ sx; sy ];
    %
    %  For each sample point, find K, the index of the nearest generator.
    %  We do this efficiently by using the Delaunay information with
    %  Matlab's DSEARCH command, rather than a brute force nearest neighbor
    %  computation.
    %  
    k = powercellidx (g(1,:),g(2,:),s(1,:),s(2,:),wts);

    m = zeros(g_num,1);
    accumM = accumarray (k, ones(s_num,1));
    m(1:length(accumM)) = accumM;
    
    g_new = g;
    sumx = zeros(g_num,1);
    sumy = zeros(g_num,1);
    accumX = accumarray ( k, s(1,:) );
    accumY = accumarray ( k, s(2,:) );
    sumx(1:length(accumX)) = accumX;
    sumy(1:length(accumY)) = accumY;
    g_new(1,m~=0) = sumx(m~=0) ./ m(m~=0);
    g_new(2,m~=0) = sumy(m~=0) ./ m(m~=0);
    
    cf(it) = 0;
    for i = 1:s_num
        cf(it) = cf(it)+(s(1,i)-g(1,k(i)))^2+(s(2,i)-g(2,k(i)))^2-h(k(i));
    end
    
    dw = zeros(g_num,1);
    [edge,pc] = findboundary(t,PC);
    nedges = size(edge,1);
    gamma = zeros(g_num,1);
    
    for i = 1:g_num-1
        for j = (i+1):g_num
            idx = sum(edge == repmat([i j],[nedges 1]),2);
            idx = find(idx == 2);
            dgamma = 0;
            for count = 1:length(idx)
                dgamma = dgamma + 0.5*(norm(g(:,i)'-pc(idx(count),:))^2-h(i)...
                    -norm(g(:,j)'-pc(idx(count),:))^2+h(j));
            end
            if ~isempty(idx)
                dgamma = dgamma/length(idx);
            end
            gamma(i) = gamma(i)+dgamma;
            gamma(j) = gamma(j)-dgamma;
        end
    end
    dw(m~=0) = gamma(m~=0) ./ m(m~=0);
    
%     L = zeros(g_num);
%     for i = 1:g_num-1
%         for j = (i+1):g_num
%             isneighbor = find(sum(edge == repmat([i j],[nedges 1]),2)==2,1);
%             if ~isempty(isneighbor)
%                 L(i,j) = -1;
%                 L(j,i) = -1;
%             end
%         end
%     end
%     for i = 1:g_num
%         L(i,i) = -sum(L(i,:));
%     end
%     M_inv = zeros(g_num);
%     for i = 1:g_num
%         if m(i) ~= 0
%             M_inv(i,i) = 1/m(i);
%         end
%     end
    % psd_mat = kw*M_inv*L/2
    %
    %  Compute the energy.
    %
    %e(it,1) = (sum ( ( s(1,:) - g(1,k(:)) ).^2 ...
    %              + ( s(2,:) - g(2,k(:)) ).^2 -wts(k(:))')) / s_num;
    e(it,:) = wts-h;

    %
    %  Display the energy.
    %
        subplot ( 1, 3, 2 )
        %figure;
        plot ( step, e )
        title ( 'Estimation Error' )
        xlabel ( 'Step' )
%         ylabel ( 'Weightings' )
        grid on
        axis equal
        xlim([0 25]);
        ylim([-0.08 0.08])
        axis square
        drawnow
    %
    %  Compute the generator motion.
    %
    gm(it,1) = sum ( ( g_new(1,:) - g(1,:) ).^2 ...
                   + ( g_new(2,:) - g(2,:) ).^2 ) / g_num;
    %
    %  Display the generator motion.
    %
        subplot ( 1, 3, 3 )
        plot ( step, cf, 'm-*' )
        title ( 'Cost Function' )
        xlabel ( 'Step' )
        ylabel ( 'Energy' )
        grid on;
        axis equal
        xlim([0 25]);
        axis square
        drawnow
    %
    %  Continue?

    %
    %     s = input ( 'RETURN, or Q to quit: ', 's' );
    % 
    %     if ( s == 'q' | s == 'Q' )
    %       break
    %     end
    pause(0.02);

    %
    %  Update the generators.
    %
    g = g+kp*(g_new-g);
    wts = wts - kw*dw;
    wts = wts - max(wts);
%      h = h + B*K*(wts-h);
end

%
%  Terminate.
%