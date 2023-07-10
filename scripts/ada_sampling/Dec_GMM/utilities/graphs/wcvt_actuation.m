g_num = 9;
it_num = 50;
s_num = 1e6;
Kp = diag([0.5 0.5]);
Kd = zeros(2,2,g_num);
Kd(:,:,1) = [0.05 0.05;0.05 0.05];
Kd(:,:,9) = -[0.05 0;0 0.05];
Ki = Kd+repmat(Kp,[1 1 g_num]);
Kihat = zeros(2,2,g_num)+repmat(Kp,[1 1 g_num]);
kw = 20000;
wts = zeros(g_num,1);
h = zeros(g_num,1);
lambda = zeros(2,2,g_num);
Lambda = zeros(2,2,g_num);
kk = 1;

for i = 1:g_num
    wts(i) = norm(Kihat(:,:,i));
    h(i) = norm(Ki(:,:,i));
end

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
        subplot ( 2, 2, 1 );
        plot(PC(:,1), PC(:,2), 'r.');
        hold on;
        voronoi(g(1,:),g(2,:),'r--');
        powerdiagram(g(1,:),g(2,:),t,'g',wts);
        plot(g(1,:),g(2,:), 'r+');
        text(g(1,:)'+0.02,g(2,:)',int2str((1:g_num)'));
        title_string = sprintf ( 'Weighted Voronoi, step %d', it );
        title ( title_string );
        axis equal
        axis ([  0.0, 1.0, 0.0, 1.0 ])
        axis square
        drawnow
        hold off;
        %
        %  Display the Delaunay triangulation.
        %
        subplot ( 2, 2, 2 );
        trimesh (t,g(1,:),g(2,:),zeros(g_num,1), 'EdgeColor', 'r' )
        hold on
        scatter ( g(1,:), g(2,:), 'k.' )
        text(g(1,:)',g(2,:)',int2str((1:g_num)'));
        title_string = sprintf ( 'Weighted Delaunay, step %d', it );
        title ( title_string );
        axis ( [  0.0, 1.0, 0.0, 1.0 ] )
        axis square
        view ( 2 )
        drawnow
        hold off
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
    
    cf(it) = 0;
    for i = 1:s_num
        cf(it) = cf(it)+(s(1,i)-g(1,k(i)))^2+(s(2,i)-g(2,k(i)))^2-h(k(i));
    end
    
    g_ct = g;
    sumx = zeros(g_num,1);
    sumy = zeros(g_num,1);
    accumX = accumarray ( k, s(1,:) );
    accumY = accumarray ( k, s(2,:) );
    sumx(1:length(accumX)) = accumX;
    sumy(1:length(accumY)) = accumY;
    g_ct(1,m~=0) = sumx(m~=0) ./ m(m~=0);
    g_ct(2,m~=0) = sumy(m~=0) ./ m(m~=0);
    
    dw = zeros(g_num,1);
    [edge,pc] = findboundary(t,PC);
    edge = unique(edge,'rows');
    nneighbor = size(edge,1);
    dneighbor = zeros(g_num,1);
    for k = 1:nneighbor
        i = edge(k,1);
        j = edge(k,2);
        delta = wts(i)-wts(j)-norm(Kihat(:,:,i))+norm(Kihat(:,:,j));
        dneighbor(i) = dneighbor(i)+delta;
        dneighbor(j) = dneighbor(j)-delta;
    end
    dw(m~=0) = dneighbor(m~=0) ./ m(m~=0);

    %
    %  Compute the energy.
    %
    %e(it,1) = (sum ( ( s(1,:) - g(1,k(:)) ).^2 ...
    %              + ( s(2,:) - g(2,k(:)) ).^2 -wts(k(:))')) / s_num;
    e(it,:) = wts-h;

    %
    %  Display the energy.
    %
        subplot ( 2, 2, 3 )
        %figure;
        plot ( step, e )
        title ( 'Weightings' )
        xlabel ( 'Step' )
        ylabel ( 'Weightings' )
        grid
    %
    %  Compute the generator motion.
    %
    %
    %  Display the generator motion.
    %
    %
    %  Continue?

    %
    %     s = input ( 'RETURN, or Q to quit: ', 's' );
    % 
    %     if ( s == 'q' | s == 'Q' )
    %       break
    %     end

    %
    %  Update the generators.
    %
    g_new = g;
    for i = 1:g_num
        Kihat(:,:,i) = Kihat(:,:,i)+(lambda(:,:,i)-Kihat(:,:,i)*Lambda(:,:,i));
        g_new(:,i) = g(:,i)+Ki(:,:,i)*(g_ct(:,i)-g(:,i));
        lambda(:,:,i) = lambda(:,:,i)+(g_new(:,i)-g(:,i))*(g_ct(:,i)-g(:,i))';
        Lambda(:,:,i) = Lambda(:,:,i)+(g_ct(:,i)-g(:,i))*(g_ct(:,i)-g(:,i))';
    end
    
    gm(it,1) = sum ( ( g_new(1,:) - g(1,:) ).^2 ...
                   + ( g_new(2,:) - g(2,:) ).^2 ) / g_num;
    if it == it_num
%         figure;
        subplot ( 2, 2, 4 )
        plot ( step, cf, 'm-*' )
        title ( 'Cost Function' )
        xlabel ( 'Step' )
        ylabel ( 'Energy' )
        grid;
        draw
    end
    pause(0.02);
    g = g_new;
    wts = wts - kw*dw;
end

%
%  Terminate.
%