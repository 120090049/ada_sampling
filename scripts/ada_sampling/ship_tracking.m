% distributed mixture of gaussian processes
% load ship_trajectory.mat
% initialte data
% Created by Wenhao Luo (whluo12@gmail.com)

% with distributed coverage control and consensus density function learning

function [rms_stack, var_stack, cf, max_mis, model, pred_h, pred_Var] = main_bot_distribute(varargin)
    global num_gau num_bot Xss Fss eta
    
    load('ship_trajectory.mat'); % use stalk count data
    [Xss, ksx_g, ksy_g] = generate_coordinates(map_length, map_width);
    Fss = reshape(F_map, [], 1);
    
    def_hyp2.mean = [];
    def_hyp2.cov = [0.5 0.5];
    def_hyp2.lik = -2;
    
    parser = inputParser;
    addOptional(parser, 'algo', 'gmm'); % how to select break-off swarm
    addOptional(parser, 'bots', []);
    addOptional(parser, 'num_gau', 2);
    addOptional(parser, 'beta', 1);
    addOptional(parser, 'unit_sam', 30);
    addOptional(parser, 'eta', 0.1);
    addOptional(parser, 'g_num', 3);
    addOptional(parser, 'hyp2_new', def_hyp2);
    addOptional(parser, 'kp', 0.5);
    addOptional(parser, 'it_num', 40);
    addOptional(parser, 'save_flag', false);
    
    parse(parser, varargin{:});
    
    algo = parser.Results.algo;
    num_gau = parser.Results.num_gau;
    beta = parser.Results.beta;
    unit_sam = parser.Results.unit_sam;
    eta = parser.Results.eta;
    g_num = parser.Results.g_num;
    bots = parser.Results.bots;
    hyp2_new = parser.Results.hyp2_new;
    kp = parser.Results.kp;
    it_num = parser.Results.it_num;
    save_flag = parser.Results.save_flag;
    
    
    %%%%%%%%%%%%%%%%%%% dataset dependent variables
    map_x = map_width;
    map_y = map_length;
    map_z = [0,1];
    % beta = 1; % beta for GP-UCB:   mu + beta*s2
    % num_gau = 3;
    % unit_sam = 3; % draw some samples from each distribution
    num_init_sam = unit_sam*num_gau + 1;%10; % initial number of samples for each robot
    % eta = 0.1; % estimator gain
    % g_num = 3;  %4
    k = g_num; % number of robots
    num_bot = g_num;
    stop_flag = 0;
    %%%%%%%%%%%%%%%%%%%
    
    N = length(Fss);
    d = size(Xss,2); %2
    % kp = 0.5;
    
    rng(200)
    g=[];
    
    [label_rss, model_rss, ~] = mixGaussEm_gmm(Fss', num_gau); % centralized GMM version
    
    if unique(label_rss)~=num_gau
        error('reduce num_gau!');
    end
    
    
    % get initial data of the distance map (3 from each gaussian component)
    pilot_Xs_stack = zeros(unit_sam*num_gau,2,num_bot);
    
    for ikoo = 1:num_bot % get random 9 points for initialization % get 9 points from 3 gaussian
        ind_ttmp = zeros(unit_sam,num_gau);
        
        for kik = 1:num_gau 
            sap_tmp = find(label_rss==kik); % points of the kik th gaussian
            ind_ttmp(:,kik) = sap_tmp(randperm(length(sap_tmp),unit_sam)); % get unit_sam=3 points from kik th gaussian
        end
        
        ind_ttmp = ind_ttmp(:);
        
        % pilot_Xs = [rand(num_init_sam,1)*map_x rand(num_init_sam,1)*map_y];
        pilot_Xs_stack(:,:,ikoo) = Xss(ind_ttmp,:);
        
    end
    
    %%%%%%% initialization
    if isempty(bots)  %nargin < 1
    
        init_abg = zeros(1,num_gau);
        tmp_init = cell(1,num_bot);
        [tmp_init{:}] = deal(init_abg);
        bots = struct('alpha_K',tmp_init,'belta_K',...
            tmp_init,'gamma_K',tmp_init,'self_alpha',tmp_init,'self_belta',tmp_init,'self_gamma',tmp_init,...
            'dot_alpha_K',tmp_init,'dot_belta_K',tmp_init,'dot_gamma_K',tmp_init,'Nm_ind',[]);
        
        packets = struct('alpha_K',[],'belta_K',[],'gamma_K',[]);
        
        
        if numel(unique(label_rss))~= num_gau
            warning('unexpected label_rss');
            return;
        end
        
        for ijk = 1:k % get all init points (10th is the starting point)
            bots(ijk).Xs = pilot_Xs_stack(:,:,ijk); % starting points for the robots, can be set of points from pilot survey
            
            bots(ijk).Xs(end+1,:) = [rand*map_x rand*map_y]; % replace by starting points in a smaller area
            
            bots(ijk).Nm_ind = get_idx(bots(ijk).Xs, Xss);  % initial index for each robot (1-945)
            g(ijk,:) = bots(ijk).Xs(end,:); % in coverage control, specify generator's positions (starting positions)
            bots(ijk).Fs = Fss(bots(ijk).Nm_ind);
        end
        % seperate into two loops since we want separate control of rng,
        % otherwise the above commands will generate same robot positions.
        
        for ijk = 1:k
            [~, model, ~, ~, ~] = mixGaussEm_gmm(Fss(bots(ijk).Nm_ind)', num_gau); % get GMM with only 10 known data % initialize: mu, Sigma, alpha
            Nm = length(bots(ijk).Nm_ind);
            bots(ijk).mu_K = model.mu;
            bots(ijk).Sigma_K = model.Sigma;
            bots(ijk).self_alpha = model.w; %  w: [0.3011 0.2732 0.4256]
            %         bots(ijk).alpha_K = model.w;
            
            [~, alpha_mnk] = mixGaussPred_gmm(Fss(bots(ijk).Nm_ind)', model); % 1st argument label, 2rd probability of each data point belongs to which Gaussian
                                                                                % Nm (10 data) x num_gau (3)
            self_alpha = sum(alpha_mnk,1); % [3.0111    2.7309    4.2580]   ./Nm;  % 1 x num_gau    Nm*alpha_mk
            y_mn = Fss(bots(ijk).Nm_ind);    %   10 y of data points
    
            bots(ijk).belta_K = sum(alpha_mnk.*y_mn,1);  %  1 x num_gau  belta_mk
            bots(ijk).gamma_K = sum(((repmat(y_mn,[1,num_gau])-model.mu).^2).*alpha_mnk, 1);  %  1 x num_gau
            bots(ijk).alpha_K = self_alpha; % for robot i, [alpha i1, i2, i3]
            
            for ijk_rec = 1:num_bot
                bots(ijk).packets(ijk_rec) = packets;   % initialize packets struct for every robots
            end
            if ijk ~= num_bot && ijk~=1
                bots(ijk).neighbor = [ijk+1, ijk-1];%setdiff(1:num_bot, ijk);    % find(adj_A(ijk,:)>0);  % get neighbor id, intialize from a fully connected graph
            elseif ijk == num_bot
                bots(ijk).neighbor = ijk - 1;
            elseif ijk == 1
                bots(ijk).neighbor = ijk + 1;
            end
        end
        
        for ijk = 1:num_bot     % reset packets to the bot themselves
            packets.alpha_K = bots(ijk).alpha_K;
            packets.belta_K = bots(ijk).belta_K;
            packets.gamma_K = bots(ijk).gamma_K;
            bots(ijk).packets(ijk) = packets;
        end
        
        
    else
        g_num = length(bots);
        k = g_num;
    end
    g = g';
    
    
    %% initiate swarm status
    
    s_num = length(Fss); %1e6;
    s = Xss';
    
    
    %  Carry out the iteration.
    %
    step = 1 : it_num;
    e = nan ( it_num, g_num );
    gm = nan ( it_num, 1 );
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
    
    hist_alpha_K = zeros(max_iter,num_gau,num_bot);
    hist_belta_K = zeros(max_iter,num_gau,num_bot);
    hist_gamma_K = zeros(max_iter,num_gau,num_bot);
    
    hist_alpha_K_norm = zeros(max_iter,num_gau,num_bot);
    hist_belta_K_norm = zeros(max_iter,num_gau,num_bot);
    hist_gamma_K_norm = zeros(max_iter,num_gau,num_bot);
    
    hist_mu_K_norm = zeros(max_iter,num_gau,num_bot);
    
    true_alpha_K = repmat(model_rss.w,[max_iter 1]);
    
    for it = 1 : it_num
        it    % coverage control iteration step
        loop_flag = true;
        cur_iter = 1;
    
        % after consensus loop, updated variable:  1) bots.neighbor, 2) new
        % local model, 3) bots.Nm_ind (after execution)
        
        %% begin consensus process to refine local model for each robot
    % step 2,3
        if ~stop_flag
            while loop_flag  % for first round, use neighbors defined by default from the part of initialization above
                for ijk = 1:num_bot
                    %             bots(ijk).neighbor = find(adj_A(ijk,:)>0);   % confirm neighbors
                    bots = transmitPacket(bots, ijk); % update communication packets
                    bots = updateBotComputations(bots, ijk);  % then update self and consensus
                end
                for dijk = 1:num_bot
                    hist_alpha_K(cur_iter,:,dijk,it) = bots(dijk).alpha_K;  % record bot 1's estimate of alpha_K
                    hist_belta_K(cur_iter,:,dijk,it) = bots(dijk).belta_K;
                    hist_gamma_K(cur_iter,:,dijk,it) = bots(dijk).gamma_K;
                    
                    hist_alpha_K_norm(cur_iter,:,dijk,it) = norm_prob(bots(dijk).alpha_K);  % record bot 1's estimate of alpha_K
                    hist_belta_K_norm(cur_iter,:,dijk,it) = norm_prob(bots(dijk).belta_K);
                    hist_gamma_K_norm(cur_iter,:,dijk,it) = norm_prob(bots(dijk).gamma_K);
                    
                    hist_mu_K_norm(cur_iter,:,dijk,it) = bots(dijk).mu_K;
                end
                cur_iter = cur_iter+1;
                if cur_iter > max_iter
                    cur_iter = cur_iter - 1;
                    %              figure;plot(1:cur_iter, hist_alpha_K_norm(:,1,1)); % plot converging profile for robot 1 w.r.t. alpha_1
                    break;
                end
                %     figure(1);
                %     hold on;
                %     plot(1:cur_iter, true_alpha_K(1:cur_iter,1),1:cur_iter, hist_alpha_K(1:cur_iter,1));
            end   % end of consensus part
        end
        
        
        %     ind_current = vertcat(model.ind);
        %     [mu_gpml, s2_gpml, hyp_gpml, rms_gpml] = gpml_rms(ind_current,Xss,Fss,Xss,Fss);
        
        %
        %  Compute the Delaunay triangle information T for the current nodes.
        %
    % step 1
        t = delaunay(g(1,:),g(2,:));
        
        %  For each sample point, find K, the index of the nearest generator.
        %  We do this efficiently by using the Delaunay information with
        %  Matlab's DSEARCHccss command, rather than a brute force nearest neighbor
        %  computation
        %  s is all points and g is the generator/agent position
        k = powercellidx (g(1,:),g(2,:),s(1,:),s(2,:)); % for each point, label it by nearest neighbor (there are 945 1/2/3 labels) 
    
    
    
    % step 4,5,6
        pred_h = zeros(length(Fss),1);
        pred_Var = zeros(length(Fss),1);
        %     pred_rms = zeros(length(num_bot),1);
     
    
        for iij = 1:num_bot % for each robot use its local data and use learned GMM model to predict environmental phenomenon
            [pred_h(k==iij), pred_Var(k==iij), ~] = gmm_pred_wafr(Xss(k==iij,:), Fss(k==iij), bots(iij), 'hyp2_new', hyp2_new);
        end
        
        est_mu = abs(pred_h);
        est_s2 = abs(pred_Var);
        phi_func = est_mu + beta*est_s2;
        
        % analysis
        idx_train = unique([bots.Nm_ind]);
        idx_test = setdiff(1:length(Fss),idx_train);
        
        rms_stack(it) = sqrt(sum( (pred_h(idx_test) - Fss(idx_test)).^2 )/(length(Fss(idx_test))));
        var_stack(it) = mean(pred_Var);
        [max_mis(it), ~] = max(abs(Fss(idx_test)-est_mu(idx_test)));
        
        g_new = g;
        
        m = zeros(g_num,1);
        accumM = accumarray (k, phi_func);   % this computes \sigma_{V_i} \phi(q) \dq for all voronoi cell V_i 
        m(1:length(accumM)) = accumM;  % \sigma_{V_i} \phi(q) \dq for each V   (g_num x 1)
        
        sumx = zeros(g_num,1);
        sumy = zeros(g_num,1);
        accumX = accumarray ( k, s(1,:)'.*phi_func );     %   \sigma_{V_i} \q_x \phi(q) \dq   ucb
        accumY = accumarray ( k, s(2,:)'.*phi_func );   %  \sigma_{V_i} \q_y \phi(q) \dq     ucb
        sumx(1:length(accumX)) = accumX;
        sumy(1:length(accumY)) = accumY;  % same as above
        g_new(1,m~=0) = sumx(m~=0) ./ m(m~=0);   % get x coordinate for the new centroid
        g_new(2,m~=0) = sumy(m~=0) ./ m(m~=0);   % get y coordinate for the new centroid
        
        g_actual = g;
        
        m = zeros(g_num,1);
        accumM = accumarray (k, Fss);   % this computes \sigma_{V_i} \phi(q) \dq for all voronoi cell V_i  % ones(s_num,1)
        m(1:length(accumM)) = accumM;  % \sigma_{V_i} \phi(q) \dq for each V   (g_num x 1)
        
        accumX = accumarray ( k, s(1,:)'.*Fss );     %   \sigma_{V_i} \q_x \phi(q) \dq   actual density function
        accumY = accumarray ( k, s(2,:)'.*Fss );   %  \sigma_{V_i} \q_y \phi(q) \dq     actual density function
        sumx(1:length(accumX)) = accumX;
        sumy(1:length(accumY)) = accumY;
        g_actual(1,m~=0) = sumx(m~=0) ./ m(m~=0);   % get x coordinate for the actual centroid
        g_actual(2,m~=0) = sumy(m~=0) ./ m(m~=0);   % get y coordinate for the actual centroid
        
        %     proj_g_idx = get_idx(g_new', Xss);    % get idx of the inferred centroid
        %     proj_g_idx_actual = get_idx(g_actual', Xss);  % get idx of the actual centroid
        %
        %     g_new = Xss(proj_g_idx,:)';     % get projected inferred centroid
        %     g_actual = Xss(proj_g_idx_actual,:)';    % get projected acutal centroid
        
        %     subplot ( 1, 3, 1 );
        
        
        if ismember(it,[1 11])
            pause(0.1)
            figure(50);
        else
            figure(50) % 20
        end
        
        
        plot_surf2( ksx_g,ksy_g,reshape(est_mu,[size(ksx_g,1),size(ksx_g,2)]), map_x, map_y, map_z,25, 5      );
        
        hold on;
        lineStyles = linspecer(10);
        colormap(linspecer);
        hold on;
        [edge_x, edge_y] = voronoi(g(1,:),g(2,:),t); %'r--'
        hold on;
        %     plot(g(1,:),g(2,:), 'b+');
        
        for idx_plot = 1:g_num
            
            plot(bots(idx_plot).Xs(num_init_sam:end,1),bots(idx_plot).Xs(num_init_sam:end,2),'LineWidth',5,'Color',lineStyles(1,:));
            hold on;
            plot(bots(idx_plot).Xs(end,1),bots(idx_plot).Xs(end,2),'o','MarkerSize',20,'LineWidth',5,'Color',lineStyles(2,:))
            hold on;
        end
        
        
        hold on;
        plot(g_actual(1,:),g_actual(2,:), '*','MarkerSize',20,'LineWidth',5,'Color',lineStyles(9,:));
        
        hold on;
        plot(edge_x,edge_y,'--','Color',lineStyles(9,:),'LineWidth',5);
        
        hold on;
        
        text(g(1,:)'+2,g(2,:)',int2str((1:g_num)'),'FontSize',25);
        %     title_string = sprintf ( 'Weighted Voronoi' );
        %     title ( title_string );
    %     axis equal
        axis ([  0, map_x, 0, map_y ])
        %     axis square
        drawnow
        xlabel('X','FontSize',25);
        ylabel('Y','FontSize',25);
        %     zlabel('Temperature (degrees Celsius)','FontSize',20);
        %     caxis([15,29]);
        set(gca,'LineWidth',5,'fontsize',25,'fontname','Times','FontWeight','Normal');
        %     grid on;
        
        hold off;
        
        if ismember(it,[1 11])
            pause(0.1)
        end
        
        cf(it) = 0;
        for i = 1:s_num
            cf(it) = cf(it)+((s(1,i)-g(1,k(i))).*Fss(i))^2+((s(2,i)-g(2,k(i))).*Fss(i)) ^2;  % get summation of distance to goal
        end
        %  Display the energy.
        %
        figure(21);
        subplot ( 1, 2, 1 )
    
        %         plot ( step, e )
        plot(step, rms_stack, 'r-s');
        title ( 'RMS Error' )
        xlabel ( 'Step' )
        %         ylabel ( 'Weightings' )
        grid on
        axis equal
        xlim([0 45]);
        %         ylim([-0.08 0.08])
        axis square
        drawnow
    
       
        figure(21);
        subplot ( 1, 2, 2 )
        
        
        %     figure(22);
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
        g = g+kp*(g_new-g); %g_new is the centroid   g is the next visit point 
        proj_g_idx = get_idx(g', Xss); 
        
        stop_count = 0;
        for ijk = 1:g_num
            if bots(ijk).Nm_ind(end) ~= proj_g_idx(ijk)
                %     if ~ismember(proj_g_idx(ijk), bots(ijk).Nm_ind)
                bots(ijk).Nm_ind(end+1) = proj_g_idx(ijk); % visited position index
                bots(ijk).Xs(end+1,:) = Xss(proj_g_idx(ijk),:);  % add new position to visited positions set
                %     end
            else
                stop_count = stop_count + 1;  % robot does not change
            end
            bots(ijk).Nm_ind = bots(ijk).Nm_ind(:)';
        end
       
        
        if stop_count == num_bot
            stop_flag = 1;
        end
        
    end
    
    date_time = datetime('now');
    dat_name = strcat('sim1_gmm_',num2str(date_time.Month), '_', num2str(date_time.Day), '_', ...
        num2str(date_time.Year), '_', num2str(date_time.Hour), '_', ...
        num2str(date_time.Minute), '_', num2str(round(date_time.Second)), '.mat');
    
    if save_flag
        save(dat_name);  %save('iros_sim2_decgp_temp_t.mat');
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
    
    %      [label, model, llh, break_flag] = mixGaussEm_rss(Fss(bots(n).Nm_ind)', num_gau);
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
