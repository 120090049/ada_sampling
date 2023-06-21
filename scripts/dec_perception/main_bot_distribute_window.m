% WAFR 2018
% distributed mixture of gaussian processes
% load sample_data_stalk_count.mat
% initialte data
% Created by Wenhao Luo (whluo12@gmail.com)
% Date: 08/26/2018

% with distributed coverage control and consensus density function learning

function [indices] = main_bot_distribute(Fss_input, seq_length, varargin)
%old return vars: rms_stack, var_stack, cf, max_mis, model, pred_h, pred_Var
% Get path to simulator
%display('Initializing MATLAB simulator');

paths = {'modeling','utilities','SFO'}; 

path_local_sim_init = pwd;
warned = 0;

for i = 1:length(paths) 
   addpath(genpath(strcat(path_local_sim_init, '/', paths{i}))); 
   if(exist(paths{i}) ~= 7) 
      warning('The path %s was not correctly added.  Make sure that you are in the directory of the simulator!', paths{i}); 
      warned = 1;
   end
end

if(warned == 0) 
   addpath(pwd);
   %display('MATLAB simulator initialized successfully!');
end
%RAN INIT HERE TO BE COMPATIBLE WITH PYTHON
global num_gau num_bot Xss Fss eta

load('sample_data_stalk_count.mat'); % use stalk count data

% lat = ncread("air.sig995.mon.mean.nc", "lat");
% lat_y = lat(1:45);
% lon = ncread("air.sig995.mon.mean.nc", "lon");
% lon_x = lon(1:21);
% time = ncread("air.sig995.mon.mean.nc", "time");
% air = ncread("air.sig995.mon.mean.nc", "air");

%SET ENV SIZE AND FILTER XSS ACCORDINGLY
env_x = 20; %20 for animals, 20 for weather
env_y = 20; %20 for animals, 44 for weather
Xss = Xss(Xss(:,1) <= env_x & Xss(:,2) <= env_y, :);

Xss_base = Xss;
Fss_base = Fss;
Fss = [];
Xss_big = [];
Fss = reshape(Fss_input, [], 1);
min_air = min(Fss,[],"all");
max_air = max(Fss,[],"all");
Fss = Fss - min_air;
for times = 1:seq_length
    %air_x_y_1 = air(1:21, 1:45, times);
    %min_air = min(air_x_y_1,[],"all");
    %air_x_y_1 = air_x_y_1 - min_air;
    %air_x_y_1 = air_x_y_1' + times;
    %min_air = min(air_x_y_1,[],"all");
    %max_air = max(air_x_y_1,[],"all");
    %Fss = [Fss; reshape(air_x_y_1, [], 1)];
    times;
    Xss_temp = [Xss ones(size(Xss, 1), 1)*times];
    Xss_big = [Xss_big; Xss_temp];
end
Xss = Xss_big;
Fss_big = Fss;
% air_x_y_1 = air(1:21, 1:45, 1);
% min_air = min(air_x_y_1,[],"all");
% air_x_y_1 = air_x_y_1 - min_air;
% air_x_y_1 = air_x_y_1';
% min_air = min(air_x_y_1,[],"all");
% max_air = max(air_x_y_1,[],"all");
% Fss = reshape(air_x_y_1, [], 1);

def_hyp2.mean = [];
def_hyp2.cov = [0.5 0.5];
def_hyp2.lik = -5;

def_hyp2 = [];

parser = inputParser;
addOptional(parser, 'algo', 'gmm'); % how to select break-off swarm
addOptional(parser, 'bots', []);
addOptional(parser, 'num_gau', 3);
addOptional(parser, 'beta', 10);
addOptional(parser, 'unit_sam', 30); %reduced from 360 to accelerate train script
addOptional(parser, 'eta', 0.1);
addOptional(parser, 'g_num', 3);
addOptional(parser, 'hyp2_new', def_hyp2);
addOptional(parser, 'kp', 0.5);
addOptional(parser, 'it_num', seq_length);
addOptional(parser, 'save_flag', false);
%For zeroing out mean in UCB
addOptional(parser, 'kappa', 0);

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
%Zeroing out mean if needed for UCB
kappa = parser.Results.kappa;
%Mutual Info for GPs
gamma_t = 0;
gamma_gp = zeros(g_num, 1);


%%%%%%%%%%%%%%%%%%% dataset dependent variables
while true
map_x = env_x;
map_y = env_y;
map_z = [min_air - min_air,max_air - min_air];
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
d = size(Xss,2);
% kp = 0.5;

rng(200)
g=[];

[label_rss, model_rss, llh_rss] = mixGaussEm_gmm(Fss', num_gau); % centralized GMM version

if size(unique(label_rss),2)~=num_gau
    %error('reduce num_gau!');
    num_gau = num_gau - 1;
else
    break
end

end
pilot_Xs_stack = zeros(unit_sam*num_gau,3,num_bot);

for ikoo = 1:num_bot
    ind_ttmp = zeros(unit_sam,num_gau);
    
    for kik = 1:num_gau
        sap_tmp = find(label_rss==kik);
        try
            ind_ttmp(:,kik) = sap_tmp(randperm(length(sap_tmp),unit_sam));
        catch
            ind_ttmp(:,kik) = sap_tmp(randi(length(sap_tmp),1,unit_sam));
        end
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
    
    for ijk = 1:k
        bots(ijk).Xs = pilot_Xs_stack(:,:,ijk); %[rand(num_init_sam,1)*map_x rand(num_init_sam,1)*map_y];  %[rand*map_x rand*map_y];  %  starting points for the robots, can be set of points from pilot survey
        
        bots(ijk).Xs(end+1,:) = [rand*map_x rand*map_y, 1]; % replace by starting points in a smaller area
        
        bots(ijk).Nm_ind = get_idx(bots(ijk).Xs, Xss);  % initial index for each robot
        g(ijk,:) = bots(ijk).Xs(end,:); % in coverage control, specify generator's positions (starting positions)
        bots(ijk).Fs = Fss(bots(ijk).Nm_ind);
    end
    % seperate into two loops since we want separate control of rng,
    % otherwise the above commands will generate same robot positions.
    
    for ijk = 1:k
        [~, model, ~, ~, ~] = mixGaussEm_gmm(Fss(bots(ijk).Nm_ind)', num_gau); % initialize: mu, Sigma, alpha
        Nm = length(bots(ijk).Nm_ind);
        bots(ijk).mu_K = model.mu;
        bots(ijk).Sigma_K = model.Sigma;
        bots(ijk).self_alpha = model.w;
        %         bots(ijk).alpha_K = model.w;
        
        [~, alpha_mnk] = mixGaussPred_gmm(Fss(bots(ijk).Nm_ind)', model); % get alpha_mnk:   Nm x num_gau
        self_alpha = sum(alpha_mnk,1); %./Nm;  % 1 x num_gau    Nm*alpha_mk
        y_mn = Fss(bots(ijk).Nm_ind);    %   Nm x 1
        bots(ijk).belta_K = sum(alpha_mnk.*y_mn,1);  %  1 x num_gau  belta_mk
        % mu_K = self_belta./(Nm*self_alpha);  % 1 x num_gau   mu_mk
        bots(ijk).gamma_K = sum(((repmat(y_mn,[1,num_gau])-model.mu).^2).*alpha_mnk, 1);  %  1 x num_gau
        bots(ijk).alpha_K = self_alpha;
        
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



%g = randi([1 599],2,10)/100;
%
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

indices = [];

for it = 1 : it_num
    it;    % coverage control iteration step
    %for ten_samps = 1:10
    loop_flag = true;
    cur_iter = 1;

    temp_Fss = [];
    temp_Xss = [];
    for ijk = 1:num_bot
        Nm = length(bots(ijk).Nm_ind);
        for dijk = 1:Nm
            if size(temp_Xss, 1) == 0 || ismember(Xss(bots(ijk).Nm_ind(dijk), :), temp_Xss, 'rows') == 0
                temp_Fss = [temp_Fss; Fss(bots(ijk).Nm_ind(dijk))];
                temp_Xss = [temp_Xss; Xss(bots(ijk).Nm_ind(dijk), :)];
                bots(ijk).Nm_ind(dijk) = size(temp_Fss, 1);
            else
                [Lia, Locb] = ismember(Xss(bots(ijk).Nm_ind(dijk), :), temp_Xss, 'rows');
                bots(ijk).Nm_ind(dijk) = Locb;
            end
        end
    end
    Fss = temp_Fss;
    Xss = temp_Xss;

    for temp_it = it-0:it+0
        if temp_it > 0
            t = ones(size(Xss_base, 1), 1)*temp_it;
            Xss_t = [Xss_base t];
            Xss = [temp_Xss; Xss_t];
            Fss = [temp_Fss; Fss_big((env_x+1)*(env_y+1)*(it-1)+1:(env_x+1)*(env_y+1)*it)];
        end
    end

    s_num = length(Fss); %1e6;
    s = Xss';

    % after consensus loop, updated variable:  1) bots.neighbor, 2) new
    % local model, 3) bots.Nm_ind (after execution)
    
    %% begin consensus process to refine local model for each robot
    
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
    t = delaunay(g(1,:),g(2,:));
    
    %  For each sample point, find K, the index of the nearest generator.
    %  We do this efficiently by using the Delaunay information with
    %  Matlab's DSEARCH command, rather than a brute force nearest neighbor
    %  computation.
    %
    k = powercellidx (g(1,:),g(2,:),s(1,:),s(2,:)); % for each point, label it by nearest neighbor  wts k - num_points x 1
    pred_h = zeros(length(Fss),1);
    pred_Var = zeros(length(Fss),1);
    %     pred_rms = zeros(length(num_bot),1);
    
    Nm_inds_all_bots = [];
    for iij = 1:num_bot
        %         idx_tmp = find(k==iij); % get corresponding index of the monitored points by bot iij
        %         bots(iij).Nm_ind = bots(iij).Nm_ind(:)';
        [pred_h(k==iij), pred_Var(k==iij), ~] = gmm_pred_wafr(Xss(k==iij,:), Fss(k==iij), bots(iij), 'hyp2_new', hyp2_new);
        Nm_inds_all_bots = [Nm_inds_all_bots; reshape(unique(bots(iij).Nm_ind), [], 1)];
    end
    %     [pred_h, pred_Var, pred_rms, llh] = map_pred(Xss, Fss, model);
    %unimodel GP
    [Xss_C, Xss_ia, ~] = unique(Xss(Xss(:,3) == it, :), 'rows', 'sorted');
    Fss_C = Fss(Xss(:,3) == it);
    [~, ~, ~, ~, pred_post] = gpml_rms([],Xss(Nm_inds_all_bots,:),Fss(Nm_inds_all_bots),Xss_C,Fss_C(Xss_ia));
    
    est_mu = abs(pred_h);
    est_s2 = abs(pred_Var);
    %     est_hyp = hyp_gpml;
    %UNCOMMENT BELOW FOR STANDARD VAR REDU HEURISTIC FOR MGP
    %Change this maybe to make beta large and alpha*est_mu?
    %phi_func = kappa*est_mu + beta*est_s2;
    %BELOW IS FOR LOWER BOUND MGP
    phi_func = zeros(length(Fss),1);
    for iij = 1:num_bot
        phi_func(k == iij) = kappa*est_mu(k == iij) + beta*(sqrt(est_s2(k == iij) + gamma_gp(iij)) - sqrt(gamma_gp(iij)));
    end
    
    idx_train = unique([bots.Nm_ind]);
    idx_test = setdiff(1:length(Fss),idx_train);
    
    rms_stack(it) = sqrt(sum( (pred_h(idx_test) - Fss(idx_test)).^2 )/(length(Fss(idx_test)))); %mean(pred_rms);
    var_stack(it) = mean(pred_Var);
    [max_mis(it), ~] = max(abs(Fss(idx_test)-est_mu(idx_test)));
    
    g_new = g;
    
    m = zeros(g_num,1);
    accumM = accumarray (k, phi_func);   % this computes \sigma_{V_i} \phi(q) \dq for all voronoi cell V_i  % ones(s_num,1)
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
    
    
%     if ismember(it,[1 11])
%         pause(0.1)
%         figure(50);
%     else
%         figure(50) % 20
%     end
%     
%     
%     %         hold on;
%     %plot_surf2( ksx_g,ksy_g,reshape(est_mu,[size(ksx_g,1),size(ksx_g,2)]), map_x, map_y, map_z,25, 5      );
%     plot_surf2( ksx_g,ksy_g,reshape(est_mu(size(temp_Fss,1)+1:end),[size(ksx_g,1),size(ksx_g,2)]), map_x, map_y, map_z,25, 5      );
%     
%     hold on;
%     lineStyles = linspecer(10);
%     colormap(linspecer);
%     hold on;
%     [edge_x, edge_y] = voronoi(g(1,:),g(2,:),t); %'r--'
%     hold on;
%     %     plot(g(1,:),g(2,:), 'b+');
%     
%     %     if it~=1
%     for idx_plot = 1:g_num
%         
%         plot(bots(idx_plot).Xs(num_init_sam:end,1),bots(idx_plot).Xs(num_init_sam:end,2),'LineWidth',5,'Color',lineStyles(1,:));
%         hold on;
%         plot(bots(idx_plot).Xs(end,1),bots(idx_plot).Xs(end,2),'o','MarkerSize',20,'LineWidth',5,'Color',lineStyles(2,:))
%         hold on;
%     end
%     %     end
%     
%     
%     hold on;
%     plot(g_actual(1,:),g_actual(2,:), '*','MarkerSize',20,'LineWidth',5,'Color',lineStyles(9,:));
%     
%     hold on;
%     plot(edge_x,edge_y,'--','Color',lineStyles(9,:),'LineWidth',5);
%     
%     hold on;
%     
%     text(g(1,:)'+2,g(2,:)',int2str((1:g_num)'),'FontSize',25);
%     %     title_string = sprintf ( 'Weighted Voronoi' );
%     %     title ( title_string );
% %     axis equal
%     axis ([  0, map_x, 0, map_y ])
%     %     axis square
%     drawnow
%     xlabel('X','FontSize',25);
%     ylabel('Y','FontSize',25);
%     %     zlabel('Temperature (degrees Celsius)','FontSize',20);
%     %     caxis([15,29]);
%     set(gca,'LineWidth',5,'fontsize',25,'fontname','Times','FontWeight','Normal');
%     %     grid on;
%     
%     hold off;
%     
%     if ismember(it,[1 11])
%         pause(0.1)
%     end
    
    cf(it) = 0;
    for i = 1:s_num
        cf(it) = cf(it)+((s(1,i)-g(1,k(i))).*Fss(i))^2+((s(2,i)-g(2,k(i))).*Fss(i)) ^2;  % get summation of distance to goal
    end
    
%     %  Display the energy.
%     %
%     figure(21);
%     subplot ( 1, 2, 1 )
% 
%     %         plot ( step, e )
%     plot(step, rms_stack, 'r-s');
%     title ( 'RMS Error' )
%     xlabel ( 'Step' )
%     %         ylabel ( 'Weightings' )
%     grid on
%     axis equal
%     xlim([0 35]);
%     %         ylim([-0.08 0.08])
%     axis square
%     drawnow
% 
%     
%     %
%     %  Compute the generator motion.
%     
%     %
%     %  Display the generator motion.
%     %
%     figure(21);
%     subplot ( 1, 2, 2 )
%     
%     
%     %     figure(22);
%     plot ( step, cf, 'm-' )
%     title ( 'Cost Function' )
%     xlabel ( 'Step' )
%     ylabel ( 'Energy' )
%     grid on;
%     axis equal
%     xlim([0 35]);
%     
%     axis square
%     drawnow
%     
%     pause(1);
    
    %
    %  Update the generators.

%     for ijkl = 1:num_bot
%         [M,I] = maxk(est_s2(k==ijkl), 1);
%         g_new(:,ijkl) = Xss(I, :)';
%     end
    
    g = g+kp*(g_new-g); %g_new; %
    
    %     proj_g_idx = get_idx(g', Xss);
    
    proj_g_idx = get_idx(g', Xss);  % sample the actually visited point instead of the centroid
    
    stop_count = 0;
    for ijk = 1:g_num
        gamma_gp(ijk) = gamma_gp(ijk) + est_s2(proj_g_idx(ijk),:);
        if bots(ijk).Nm_ind(end) ~= proj_g_idx(ijk)
            %     if ~ismember(proj_g_idx(ijk), bots(ijk).Nm_ind)
            bots(ijk).Nm_ind(end+1) = proj_g_idx(ijk);
            bots(ijk).Xs(end+1,:) = Xss(proj_g_idx(ijk),:);
            %     end
        else
            stop_count = stop_count + 1;
        end
        bots(ijk).Nm_ind = bots(ijk).Nm_ind(:)';
    end
    
    % after consensus loop, updated variable:  1) bots.neighbor, 2)
    % bots.Nm_ind, 3) new local model, 4) packets
    %     bots = updateBotModel(bots);
    
    
    %     date_time = datetime('now');
    %     dat_name = strcat('sim1_gmm_',num2str(date_time.Month), '_', num2str(date_time.Day), '_', ...
    %         num2str(date_time.Year), '_', num2str(date_time.Hour), '_', ...
    %         num2str(date_time.Minute), '_', num2str(round(date_time.Second)), '.mat');
    %
    %     save(dat_name);  %save('iros_sim2_decgp_temp_t.mat');
    
    if stop_count == num_bot
        stop_flag = 1;
    end
    
    %end %ten_samps loop
    %MAX VAR SAMPLE SELECT FOR RNP (uncomment below)
%     [M_it, I_it] = maxk(est_s2(Xss(:,3) == it), 10);
%     vals = Xss(I_it, :);
%     if size(unique(vals, 'rows'), 1) == 10
%         idxs = find(ismember(Xss_big, vals, 'rows'));
%     else
%         %disp('for loop method');
%         idxs = randperm((env_x+1)*(env_y+1), 10)';
%         for i_it = 1:10
%             curr_idx = find(ismember(Xss_big, vals(i_it, :), 'rows'));
%             idxs(i_it) = curr_idx;
%         end
%     end
%     idxs = mod(idxs, (env_x+1)*(env_y+1));
%     idxs(idxs == 0) = (env_x+1)*(env_y+1);
%     idxs = idxs - 1;
%     indices = [indices idxs];
    %MAX VAR SAMPLE SELECT FOR RNP (uncomment above)
    %MI SAMPLE SELECT FOR RNP (below)
    idxs = [];
    mask = (Xss_C(:,3) == it);
    n = sum(mask);
    V = 1:n;
    v = V(randperm(length(V)));
    F = sfo_fn_mi(pred_post, V);
    k = 10;
    a2 = 0;
    A2 = [];
    for l = 1:k
        l;
        for t = (floor((l - 1)*n/k))+1:(floor(l*n/k))
            t;
            if mask(t) == 1
                mInfo_2 = F([A2 v(t)]);
                if (l - 1)*n/k < t && t < (l - 1)*n/k + n/(k*exp(1))
                    if mInfo_2 > a2
                        a2 = mInfo_2;
                    end
                else
                    if (mInfo_2 > a2 || t == floor(l*n/k)) %&& (flag2 == false)
                        A2 = [A2 v(t)];
                        I_it = v(t);%find(ismember(est_s2, s2(t)), 1);
                        vals = Xss(I_it, :);
                        curr_idx = find(ismember(Xss_big, vals, 'rows'));
                        if isempty(curr_idx)
                            disp(vals)
                            disp(Xss_big)
                            error('empty curr idx')
                        end
                        idxs = [idxs; curr_idx];
                        a2 = 0;
                        break
                        %flag2 = true;
                    end
                end
            end
        end
    end
    idxs = mod(idxs, (env_x+1)*(env_y+1));
    idxs(idxs == 0) = (env_x+1)*(env_y+1);
    idxs = idxs - 1;
    indices = [indices idxs];
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

function bots = updateBotModel(bots)
global num_bot num_gau Xss Fss
packets = struct('alpha_K',[],'belta_K',[],'gamma_K',[]);

for ijk = 1:length(bots)
    model = struct();
    
    model.Sigma = bots(ijk).gamma_K./bots(ijk).alpha_K;
    
    kss = zeros(1,1,num_gau);
    for ijj = 1:num_gau
        kss(:,:,ijj) =  model.Sigma(ijj);
    end
    model.Sigma = kss;
    
    model.mu = bots(ijk).belta_K./(bots(ijk).alpha_K);
    model.w = norm_prob(bots(ijk).alpha_K);
    
    
    [~, model, ~, ~, ~] = mixGaussEm_rss(Fss(bots(ijk).Nm_ind)', model); % initialize with converged previous mu, Sigma, alpha, and new Nm_ind
    Nm = length(bots(ijk).Nm_ind);
    bots(ijk).mu_K = model.mu;
    bots(ijk).Sigma_K = model.Sigma;
    bots(ijk).self_alpha = model.w;
    %             bots(ijk).alpha_K = model.w;
    
    [~, alpha_mnk] = mixGaussPred_rss(Fss(bots(ijk).Nm_ind)', model); % get alpha_mnk:   Nm x num_gau
    self_alpha = sum(alpha_mnk,1); %./Nm;  % 1 x num_gau    Nm*alpha_mk
    y_mn = Fss(bots(ijk).Nm_ind);    %   Nm x 1
    bots(ijk).belta_K = sum(alpha_mnk.*y_mn,1);  %  1 x num_gau  belta_mk
    % mu_K = self_belta./(Nm*self_alpha);  % 1 x num_gau   mu_mk
    bots(ijk).gamma_K = sum(((repmat(y_mn,[1,num_gau])-model.mu).^2).*alpha_mnk, 1);  %  1 x num_gau
    bots(ijk).alpha_K = self_alpha;
    
    for ijk_rec = 1:num_bot
        bots(ijk).packets(ijk_rec) = packets;   % initialize packets struct for every robots
    end
    bots(ijk).neighbor = setdiff(1:num_bot, ijk);    % find(adj_A(ijk,:)>0);  % get neighbor id, intialize from a fully connected graph
    
end

for ijk = 1:num_bot     % reset packets to the bot themselves
    packets.alpha_K = bots(ijk).alpha_K;
    packets.belta_K = bots(ijk).belta_K;
    packets.gamma_K = bots(ijk).gamma_K;
    bots(ijk).packets(ijk) = packets;
end


end



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

% mu_k = ;
% sigma_k = zeros(1,num_gau);
% gamma_k = zeros(1,num_gau);


%      [label, model, llh, break_flag] = mixGaussEm_rss(Fss(bots(n).Nm_ind)', num_gau);
[~, alpha_mnk] = mixGaussPred_gmm(Fss(bots(n).Nm_ind)', model); % get alpha_mnk:   Nm x num_gau
self_alpha = sum(alpha_mnk,1); %./Nm;  % 1 x num_gau    Nm*alpha_mk
y_mn = Fss(bots(n).Nm_ind);    %   Nm x 1
self_belta = sum(alpha_mnk.*y_mn,1);  %  1 x num_gau  belta_mk
% mu_K = self_belta./(Nm*self_alpha);  % 1 x num_gau   mu_mk
self_gamma = sum(((repmat(y_mn,[1,num_gau])-model.mu).^2).*alpha_mnk, 1);  %  1 x num_gau
% self_Sigma = reshape(self_gamma./(Nm*self_alpha),[1,1,num_gau]);  % 1 x num_gau   gamma_mk

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

bots(n).dot_alpha_K = sum(stack_alpha_neighbor - bots(n).alpha_K,1) + bots(n).self_alpha - bots(n).alpha_K; %  note the difference self_alpha should be Nm*alpha or just alpha ?
bots(n).dot_belta_K = sum(stack_belta_neighbor - bots(n).belta_K,1) + bots(n).self_belta - bots(n).belta_K;
bots(n).dot_gamma_K = sum(stack_gamma_neighbor - bots(n).gamma_K,1) + bots(n).self_gamma - bots(n).gamma_K;

bots(n).alpha_K = bots(n).alpha_K + eta*bots(n).dot_alpha_K;
bots(n).belta_K = bots(n).belta_K + eta*bots(n).dot_belta_K;
bots(n).gamma_K = bots(n).gamma_K + eta*bots(n).dot_gamma_K;


bots(n).Sigma_K = bots(n).gamma_K./bots(n).alpha_K;
bots(n).mu_K = bots(n).belta_K./(bots(n).alpha_K);

kss = zeros(1,1,num_gau);
for ijj = 1:num_gau
    
    if bots(n).Sigma_K(ijj)<10^-5
        disp('Sigma too low')
        %pause;
    end
    
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