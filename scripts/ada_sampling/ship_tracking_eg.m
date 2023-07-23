% distributed mixture of gaussian processes
% load ship_trajectory.mat
% initialte data
% Created by Wenhao Luo (whluo12@gmail.com)

% with distributed coverage control and consensus density function learning



function [rms_stack, var_stack, cf, max_mis, model, pred_h, pred_Var] = main_bot_distribute(varargin)
    %% Bots initialization
    % import py file
    py.importlib.import_module('controller');
    % renew the cache for python
    % clear classes;
    % obj = py.importlib.import_module('controller');
    % py.importlib.reload(obj);

    global num_gau num_bot Xss Fss eta

    %% read data for map
    load('ship_trajectory_old.mat'); % F_map(200,100) map_length map_width targets(8*2)
    [Xss, ksx_g, ksy_g] = generate_coordinates(map_length, map_width);
    Fss = F_map(:);
    Fss_for_centroid = Fss;
    Xss_T = Xss';
    
    
    map_x = map_width; % 20
    map_y = map_length; % 40
    map_z = [0,1];
    N = length(Fss);
    d = size(Xss,2); %2
    
    def_hyp2.mean = [];
    def_hyp2.cov = [0.5 0.5];
    def_hyp2.lik = -2;    
    parser = inputParser;
    addOptional(parser, 'hyp2_new', def_hyp2);
    parse(parser, varargin{:});
    hyp2_new = parser.Results.hyp2_new;
    
    %% dataset dependent variables
    num_gau = 3; % GMM
    beta = 1;    % beta for GP-UCB:   mu + beta*s2
    
    eta = 0.1; % consensus filter
    g_num = 3; %num of GPs and bots
    num_bot = g_num; % number of robots
    bots = [];
    
    kp = 0.5; % move towards centroid
    it_num = 40;
    save_flag = false;
    
    unit_sam = 4;  % draw some samples from each distribution
    num_init_sam = unit_sam*num_gau + 1;%10; % initial number of samples for each robot
    
    stop_flag = 0;
    
    
    rng(200)
    g=[];
    
    plot_3Dsurf(10, reshape(Fss,[size(ksx_g,1),size(ksx_g,2)]), [0,1]);
    
    %% Add noise to Fss data==0, to fit the GMM
    noise = normrnd(0.1, 0.05, size(Fss)); % 标准差为0.03
    Fss_zero_indices = (Fss == 0); % 选择 Fss 中为0的元素的逻辑索引
    Fss(Fss_zero_indices) = Fss(Fss_zero_indices) - noise(Fss_zero_indices);    
    
    [label_rss, model_rss, ~] = mixGaussEm_gmm(Fss', num_gau); % centralized GMM version
    
    
    if numel(unique(label_rss))~=num_gau
        error('reduce num_gau!');
        return;
    end
    
    uniqueElements = unique(label_rss);
    
    % 使用 histcounts 函数统计每个唯一元素的个数
    counts = histcounts(label_rss, [uniqueElements, max(uniqueElements)+1]);
    
    for i = 1:num_gau
        fprintf('Gaussian %d has：%d, Sigma=%d, mu=%d\n', i, counts(i), model_rss.Sigma(:,:,i), model_rss.mu(i) );
        end
        disp(model_rss);
        
        
        %% get initial data of the distance map (3 from each gaussian component)
        pilot_Xs_stack = zeros(unit_sam*num_gau,2,num_bot);
        for ikoo = 1:num_bot % get random 9 points for initialization % get 9 points from 3 gaussian
            
            ind_ttmp = zeros(unit_sam,num_gau);
            
            for kik = 1:num_gau 
                sap_tmp = find(label_rss==kik); % points of the kik th gaussian
                ind_ttmp(:,kik) = sap_tmp(randperm(length(sap_tmp),unit_sam)); % get unit_sam=3 points from kik th gaussian
            end
            
            ind_ttmp = ind_ttmp(:);
            pilot_Xs_stack(:,:,ikoo) = Xss(ind_ttmp,:);
        end
        
        


    if isempty(bots)  %nargin < 1
        
        init_abg = zeros(1,num_gau);
        tmp_init = cell(1,num_bot);
        [tmp_init{:}] = deal(init_abg);
        bots = struct('alpha_K',tmp_init,'belta_K',tmp_init,'gamma_K',tmp_init,...
                      'self_alpha',tmp_init,'self_belta',tmp_init,'self_gamma',tmp_init,...
                      'dot_alpha_K',tmp_init,'dot_belta_K',tmp_init,'dot_gamma_K',tmp_init,...
                      'Nm_ind',[]);
        
        packets = struct('alpha_K',[],'belta_K',[],'gamma_K',[]);
        
        
        %% set initial position of the bots and initialize the ergodic controller
        for ijk = 1:num_bot % get all init points (10th is the starting point)
            bots(ijk).Xs = pilot_Xs_stack(:,:,ijk); % starting points for the robots, can be set of points from pilot survey
            
            start_location_x = map_x/2 + (-1)^(ijk)*(map_x/2-1);

            start_location_y = (ijk-1)*map_y/(num_bot-1)-1;
            if start_location_y < 0
                start_location_y = 0;
            end
            bots(ijk).Xs(end+1,:) = [start_location_x start_location_y]; % replace by starting points in a smaller area
            
            bots(ijk).Nm_ind = get_idx(bots(ijk).Xs, Xss);  % initial index for each robot (1-945)
            g(ijk,:) = bots(ijk).Xs(end,:); % in coverage control, specify generator's positions (starting positions)
            bots(ijk).Fs = Fss(bots(ijk).Nm_ind);

            % initialize the controller
            py.list([2,5])
            py.list([start_location_x,start_location_y])
            bots(ijk).controller = py.controller.Controller(py.list([start_location_x,start_location_y]), map_y, map_x);
        end


        % seperate into two loops since we want separate control of rng,
        % otherwise the above commands will generate same robot positions.
 
        for ijk = 1:num_bot
            %% eq 13 get local GMM
            % get local GMM with only 10 known data % initialize: model.mu, Sigma, alpha
            [~, model, ~, ~, ~] = mixGaussEm_gmm(Fss(bots(ijk).Nm_ind)', num_gau);  % train GMM
            Nm = length(bots(ijk).Nm_ind);
            bots(ijk).mu_K = model.mu;
            bots(ijk).Sigma_K = model.Sigma;

            bots(ijk).self_alpha = model.w; %  w: [0.5078    0.4922]    ai1, ai2 (13)        
            [~, alpha_mnk] = mixGaussPred_gmm(Fss(bots(ijk).Nm_ind)', model); 
            % p(yj|...) 
                % predict the sample belongs to which Gaussian
                % return 1st argument label, 2rd probability of each data point belongs to which Gaussian
                % Nm (10 data) x num_gau (3)
            self_alpha = sum(alpha_mnk,1); % [3.0111    2.7309    4.2580]   Ui,ig_alpha
            y_mn = Fss(bots(ijk).Nm_ind);    %   10 y of data points
    
            bots(ijk).belta_K = sum(alpha_mnk.*y_mn,1);  %  1 x num_gau  belta_mk
            bots(ijk).gamma_K = sum(((repmat(y_mn,[1,num_gau])-model.mu).^2).*alpha_mnk, 1);  %  1 x num_gau
            bots(ijk).alpha_K = self_alpha; % for robot i, [alpha i1, i2, i3]
            
            %% initalize pack and its neighbour
            for ijk_rec = 1:num_bot
                bots(ijk).packets(ijk_rec) = packets;   % initialize packets struct for every robots
            end
            if ijk ~= num_bot && ijk~=1
                bots(ijk).neighbor = [ijk+1, ijk-1]; %setdiff(1:num_bot, ijk);    % find(adj_A(ijk,:)>0);  % get neighbor id, intialize from a fully connected graph
            elseif ijk == num_bot
                bots(ijk).neighbor = [1, ijk - 1];
            elseif ijk == 1
                bots(ijk).neighbor = [ijk + 1, num_bot];
            end
        end
        
        for ijk = 1:num_bot     % initialize their own packs
            packets.alpha_K = bots(ijk).alpha_K;
            packets.belta_K = bots(ijk).belta_K;
            packets.gamma_K = bots(ijk).gamma_K;
            bots(ijk).packets(ijk) = packets;
        end

        
    end
   

    g = g';
    
    
    %% initiate swarm status
    
    s_num = length(Fss); %1e6;
    
    
    %  Carry out the iteration.
    step = 1 : it_num;
    cf = nan(it_num, 1);
    max_mis = nan(it_num, 1);
    rms_stack = nan(it_num, 1);
    var_stack = nan(it_num, 1);
    
    %%  initialize for consensus loop
    max_iter = 1000; % consensus communication round
    
    % first round communication
    for ijk = 1:num_bot
        bots = transmitPacket(bots, ijk); % update communication packets
    end
    
    
    for it = 1 : it_num
        it    % coverage control iteration step
        loop_flag = true;
        cur_iter = 1;
    
        % after consensus loop, updated variable:  1) bots.neighbor, 2) new
        % local model, 3) bots.Nm_ind (after execution)
        
        %% begin consensus process to refine local model for each robot
        %% step 2,3
        if ~stop_flag
            while loop_flag  % for first round, use neighbors defined by default from the part of initialization above
                for ijk = 1:num_bot
                    %             bots(ijk).neighbor = find(adj_A(ijk,:)>0);   % confirm neighbors
                    bots = transmitPacket(bots, ijk); % update communication packets
                    bots = updateBotComputations(bots, ijk);  % then update self and consensus
                end
             
                cur_iter = cur_iter+1;
                if cur_iter > max_iter % transimit 1000 times
                    break;
                end
           
            end   
        end % end of consensus part
        
   
        %% step 1 recompute the Voronoi region
        %Compute the Delaunay triangle information T for the current nodes.
        %  For each sample point, find K, the index of the nearest generator.
        %  We do this efficiently by using the Delaunay information with
        %  Matlab's DSEARCHccss command, rather than a brute force nearest neighbor
        %  computation
        %  s is all points and g is the generator/agent position
        is_collinear = true;
        temp_g = g;
        while is_collinear
            try
                t = delaunay(temp_g(1,:), temp_g(2,:));
                is_collinear = false;
            catch
                % 调整第二个点的坐标，使其微调
                temp_g(1, 2) = temp_g(1, 2) + 1;
            end
        end
        index_label = powercellidx (g(1,:),g(2,:),Xss_T(1,:),Xss_T(2,:)); % for each point, label it by nearest neighbor (there are 945 1/2/3 labels)  
        

        %% step 4 + 5 + 6-1 
        pred_h_list = cell(1, num_bot);
        pred_Var_list = cell(1, num_bot);
        for i = 1:num_bot
            pred_h_list{i} = zeros(length(Fss), 1);
            pred_Var_list{i} = zeros(length(Fss), 1);
        end

        pred_h = zeros(length(Fss),1);
        pred_Var = zeros(length(Fss),1);
        
        for i = 1:num_bot % for each robot use its local data and use learned GMM model to predict environmental phenomenon
            [h, Var, ~] = gmm_pred_wafr(Xss(index_label==i,:), Fss(index_label==i), bots(i), 'hyp2_new', hyp2_new);
            pred_h(index_label==i) = h;
            pred_Var(index_label==i) = Var;
            % [pred_h(index_label==iij), pred_Var(index_label==iij), ~] = gmm_pred_wafr(Xss(index_label==iij,:), Fss(index_label==iij), bots(iij), 'hyp2_new', hyp2_new);
            pred_h_list{i}(index_label==i) = h;
            pred_Var_list{i}(index_label==i) = Var;
        end
        
        %% 6-2 evaluate h(q)
        est_mu = pred_h;
        est_mu(est_mu > 1) = 1;
        est_mu(est_mu < 0) = 0;
        est_s2 = abs(pred_Var);
        % beta = 10;
        
        
      
        %% HEURISTIC FUNCTION
        beta = 20;
        phi_func = est_mu + beta*est_s2;
        
        
        
        % evaluate prediction perfermance
            idx_train = unique([bots.Nm_ind]); % 3 column, 1 for each bots
            idx_test = setdiff(1:length(Fss),idx_train); % all index for not trained data
        
        rms_stack(it) = sqrt(sum( (pred_h(idx_test) - Fss(idx_test)).^2 )/(length(Fss(idx_test))));  % prediction error
        var_stack(it) = mean(pred_Var);  % uncertainty
        [max_mis(it), ~] = max(abs(Fss(idx_test)-est_mu(idx_test))); % max of misprediction
        
        
        %% Step 7 Compute local adaptive converge control law
        
        
        %% calcuate the centroid of UCB function
        g_new = g;

        m = zeros(g_num,1);
        phi_func_for_centroid = phi_func;
        phi_func_for_centroid(phi_func_for_centroid < 0) = 0;
        
        accumM = accumarray (index_label, phi_func);  
        m(1:length(accumM)) = accumM;  
        
        sumx = zeros(g_num,1);
        sumy = zeros(g_num,1);
        accumX = accumarray ( index_label, Xss_T(1,:)'.*phi_func_for_centroid );     %   \sigma_{V_i} \q_x \phi(q) \dq   ucb
        accumY = accumarray ( index_label, Xss_T(2,:)'.*phi_func_for_centroid );   %  \sigma_{V_i} \q_y \phi(q) \dq     ucb
        sumx(1:length(accumX)) = accumX;
        sumy(1:length(accumY)) = accumY;  

        % new centroid of phi_func (h(q) in the paper)
        g_new(1,m~=0) = sumx(m~=0) ./ m(m~=0);   % get x coordinate for the new centroid
        g_new(2,m~=0) = sumy(m~=0) ./ m(m~=0);   % get y coordinate for the new centroid
        
        
        %% calculate real centroid 
        g_actual = g;
        m = zeros(g_num,1);

        %% plot partitation  
        % matrix = reshape(index_label, 200, 100)';
        % colormap('jet');  % 选择一个颜色映射，这里使用 'jet'，你可以根据需要选择其他的颜色映射
        % imagesc(matrix);
        % colorbar;  % 添加颜色条，用于解释图像中的颜色对应数值
        % 
    
        accumM = accumarray (index_label, Fss_for_centroid);   % this computes \sigma_{V_i} \phi(q) \dq for all voronoi cell V_i  % ones(s_num,1)
        m(1:length(accumM)) = accumM;  % \sigma_{V_i} \phi(q) \dq for each V   (g_num x 1)
        
        accumX = accumarray ( index_label, Xss_T(1,:)'.*Fss_for_centroid );     %   \sigma_{V_i} \q_x \phi(q) \dq   actual density function
        accumY = accumarray ( index_label, Xss_T(2,:)'.*Fss_for_centroid );   %  \sigma_{V_i} \q_y \phi(q) \dq     actual density function
        sumx(1:length(accumX)) = accumX;
        sumy(1:length(accumY)) = accumY;
        g_actual(1,m>0) = sumx(m~=0) ./ m(m~=0);   % get x coordinate for the actual centroid
        g_actual(2,m>0) = sumy(m~=0) ./ m(m~=0);   % get y coordinate for the actual centroid
        
        %% Done get centroid coordinate
        proj_g_idx = get_idx(g_new', Xss);    % get idx of the inferred centroid
        proj_g_idx_actual = get_idx(g_actual', Xss);  % get idx of the actual centroid

        g_new = Xss(proj_g_idx,:)';     % get projected inferred centroid
        g_actual = Xss(proj_g_idx_actual,:)';    % get projected acutal centroid
            
    
        
        figure(1);
%% plot mu       
        plot_surf2( ksx_g,ksy_g,reshape(est_mu,[size(ksx_g,1),size(ksx_g,2)]), map_x, map_y, map_z,25, 5);
        % plot_surf2( ksx_g,ksy_g,reshape(Fss,[size(ksx_g,1),size(ksx_g,2)]), map_x, map_y, map_z,25, 5);
        hold on;
        lineStyles = linspecer(10);
        colormap(linspecer);
        hold on;
        [edge_x, edge_y] = voronoi(g(1,:),g(2,:),t); %'r--'
        hold on;
 
%% plot trajectory       
        for idx_plot = 1:g_num % bots trajectory print
            
            plot(bots(idx_plot).Xs(num_init_sam:end,1),bots(idx_plot).Xs(num_init_sam:end,2),'LineWidth',5,'Color',lineStyles(7,:));
            hold on;
            plot(bots(idx_plot).Xs(end,1),bots(idx_plot).Xs(end,2),'o','MarkerSize',20,'LineWidth',5,'Color',lineStyles(2,:))
            hold on;
        end
        
%% plot centroid        
        plot(g_actual(1,:),g_actual(2,:), '*','MarkerSize',20,'LineWidth',5,'Color',lineStyles(9,:));
        plot(g_new(1,:),g_new(2,:), '*','MarkerSize',10,'LineWidth',5,'Color',lineStyles(8,:));
        hold on;

%% plot division edge        
        plot(edge_x,edge_y,'--','Color',lineStyles(9,:),'LineWidth',5);
        hold on;
        
        text(g(1,:)'+2,g(2,:)',int2str((1:g_num)'),'FontSize',25);

        axis ([  0, map_x, 0, map_y ])
        drawnow
        xlabel('X','FontSize',25);
        ylabel('Y','FontSize',25);

        set(gca,'LineWidth',5,'fontsize',25,'fontname','Times','FontWeight','Normal');
        
        hold off;
        
        if ismember(it,[1 11])
            pause(0.1)
        end
        
        cf(it) = 0;
        for i = 1:s_num
            cf(it) = cf(it)+((Xss_T(1,i)-g(1,index_label(i))).*Fss(i))^2+((Xss_T(2,i)-g(2,index_label(i))).*Fss(i)) ^2;  % get summation of distance to goal
        end

%%  Display the energy.
        figure(2);
        subplot ( 1, 2, 1 )
        plot(step, rms_stack, 'r-s');
        title ( 'RMS Error' )
        xlabel ( 'Step' )
        grid on
        axis equal
        xlim([0 45]);
        axis square
        drawnow
    
        figure(2);
        subplot ( 1, 2, 2 )
        plot ( step, cf, 'm-' )
        title ( 'Cost Function' )
        xlabel ( 'Step' )
        ylabel ( 'Energy' )
        grid on;
        axis equal
        xlim([0 35]);
        axis square
        drawnow
        
        pause(1);
        
        
    % Step 7 Update the generators.
        % pred_h_list = cell(1, num_bot);
        % pred_Var_list = cell(1, num_bot);
        % for i = 1:num_bot
        %     pred_h_list{i} = zeros(length(Fss), 1);
        %     pred_Var_list{i} = zeros(length(Fss), 1);
        % end
        g = g+kp*(g_new-g); %g_new is the centroid   g is the next visit point 
        proj_g_idx = get_idx(g', Xss); 
        
        stop_count = 0;
        for ijk = 1:g_num
            set_points = double(bots(ijk).controller.get_nextpts(pred_h_list{ijk}));
            set_points = round(set_points);
            set_points = unique(set_points, 'rows'); % remove redundant coordinates
            
            set_points_indexs = get_idx(set_points, Xss); 

            for i = 1:size(set_points_indexs, 1)
                index = set_points_indexs(i);

                if bots(ijk).Nm_ind(end) ~= index
                    bots(ijk).Nm_ind(end+1) = index; % visited position index
                    bots(ijk).Xs(end+1,:) = Xss(index,:);  % add new position to visited positions set
                end
            end

            bots(ijk).Nm_ind = bots(ijk).Nm_ind(:)';

        end
       
        
        if stop_count == num_bot
            stop_flag = 1;
        end
        
        %% plot 
        % look into (phi_func = est_mu + beta*est_s2)
        plot_3Dsurf(4, reshape(est_mu,[size(ksx_g,1),size(ksx_g,2)]), [0,1]);
        plot_3Dsurf(3, reshape(est_s2,[size(ksx_g,1),size(ksx_g,2)]), [0, 4]);
        pause(1);
 
        
    end
    
end


function idx = get_idx(Xtest, Xs)
    Distance = pdist2(Xtest, Xs,'euclidean');
    [~, idx] = min(Distance,[],2);
end

% Transmit bot n's packet to any neighbors for whom packetLost returns false
function bots = transmitPacket(bots, n)
num_bot = numel(bots);
for j=1:num_bot
    %if ~packetLost(norm(bots(n).state.p - bots(j).state.p))
    if ismember(j, bots(n).neighbor)
        bots(j).packets(n) = bots(n).packets(n);
    end
    %end
end
end

% update cycle


function bots = updateBotComputations(bots, n)
    global Fss eta
    num_gau = numel(bots(n).alpha_K);
    Nm = length(bots(n).Nm_ind);        %  initialization
    
    % resort bot mu and ind
    % [~, resort_ind] = sort(bots(n).mu_K, 'ascend'); % in case some components are with different orders during each node local computation
    % bots(n).mu_K = bots(n).mu_K(resort_ind);
    % bots(n).Sigma_K = bots(n).Sigma_K(:,:,resort_ind);
    % bots(n).alpha_K = bots(n).alpha_K(resort_ind);
    
    model = struct;
    model.mu = bots(n).mu_K;
    model.Sigma = bots(n).Sigma_K;
    model.w = norm_prob(bots(n).alpha_K); %bots(n).alpha_K;
    [~, alpha_mnk] = mixGaussPred_gmm(Fss(bots(n).Nm_ind)', model); % get alpha_mnk:   label and responsibility(Nm * num_gau)
        %1.0000         0
        %1.0000         0
        %1.0000         0
        %1.0000         0
        %1.0000         0
        %0.0299    0.9701
        %1.0000         0
    
    self_alpha = sum(alpha_mnk,1); %./Nm;  % 1 x num_gau    Nm*alpha_mk
    y_mn = Fss(bots(n).Nm_ind);    %   Nm x 1
    self_belta = sum(alpha_mnk.*y_mn,1);  %  1 x num_gau  belta_mk
    self_gamma = sum(((repmat(y_mn,[1,num_gau])-model.mu).^2).*alpha_mnk, 1);  %  1 x num_gau
    
    bots(n).self_alpha = self_alpha;
    bots(n).self_belta = self_belta;
    bots(n).self_gamma = self_gamma;
    
    %% after compute local summary stats, we update estimate of global stats using packets
    % without considering age of the packets
    
    num_neighbor = length(bots(n).neighbor);
    
    %% start consensus based dynamic estimation process
    stack_alpha_neighbor = reshape([bots(n).packets(bots(n).neighbor).alpha_K].',[num_gau, num_neighbor]).';
    stack_belta_neighbor = reshape([bots(n).packets(bots(n).neighbor).belta_K].',[num_gau, num_neighbor]).';
    stack_gamma_neighbor = reshape([bots(n).packets(bots(n).neighbor).gamma_K].',[num_gau, num_neighbor]).';
    
    %% X_dot  dot_alpha_K = [alphai1, i2, i3] 
    % bots(n).self_alpha = u_alpha
    %  note the difference self_alpha should be Nm*alpha or just alpha ?
    bots(n).dot_alpha_K = sum(stack_alpha_neighbor - bots(n).alpha_K,1) + bots(n).self_alpha - bots(n).alpha_K; 
    bots(n).dot_belta_K = sum(stack_belta_neighbor - bots(n).belta_K,1) + bots(n).self_belta - bots(n).belta_K;
    bots(n).dot_gamma_K = sum(stack_gamma_neighbor - bots(n).gamma_K,1) + bots(n).self_gamma - bots(n).gamma_K;
    
    bots(n).alpha_K = bots(n).alpha_K + eta*bots(n).dot_alpha_K;
    bots(n).belta_K = bots(n).belta_K + eta*bots(n).dot_belta_K;
    bots(n).gamma_K = bots(n).gamma_K + eta*bots(n).dot_gamma_K;
    
    
    bots(n).Sigma_K = bots(n).gamma_K./bots(n).alpha_K;
    bots(n).mu_K = bots(n).belta_K./(bots(n).alpha_K);
    
    kss = zeros(1,1,num_gau);
    for ijj = 1:num_gau
        
        %if bots(n).Sigma_K(ijj)<10^-5
        %    pause;
        %end
        
        kss(:,:,ijj) = bots(n).Sigma_K(ijj);
    end
    bots(n).Sigma_K = kss;

    %% end of estimation and parameter updates
    bots(n).packets(n).alpha_K = bots(n).alpha_K;
    bots(n).packets(n).belta_K = bots(n).belta_K;
    bots(n).packets(n).gamma_K = bots(n).gamma_K;

end



function y = loggausspdf(X, mu, Sigma)
    d = size(X,1);   %  X:   d x Nm
    X = bsxfun(@minus,X,mu);
    [U,p]= chol(Sigma);
    if p ~= 0
        error('ERROR: Sigma is not PD.');
    end
    Q = U'\X;
    q = dot(Q,Q,1);  % quadratic term (M distance)
    c = d*log(2*pi)+2*sum(log(diag(U)));   % normalization constant
    y = -(c+q)/2;
end

function y = norm_prob(X)
    % X:  n x d where d is the num_gau
    y = X./sum(X,2);
end


function [Xss, ksx_g, ksy_g] = generate_coordinates(length, width)
    length = double(length);
    width = double(width);
    [X, Y] = meshgrid(0:width-1, 0:length-1);
    Xss = [X(:), Y(:)];
    ksx_g = repmat(0:width-1, length, 1);
    ksy_g = repmat(0:length-1, width, 1)';
end
