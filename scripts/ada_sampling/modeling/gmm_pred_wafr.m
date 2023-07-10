% WAFR 2018 
% using learned GMM model to predict environmental phenomenon
% Created by Wenhao Luo (whluo12@gmail.com)
% Date: 07/16/2018
% Mixture of GPs based on converged GMM and local training data

function [pred_h, pred_Var, pred_rms] = gmm_pred_wafr(Xtest, Ftest, model_bot_n, varargin)

% Xtest:   Ntest x d
% Ftest;   Ntest x 1
% model_bot_n: struct containing robot n's model knowledge

global Xss Fss num_gau num_bot

parser = inputParser;
addOptional(parser, 'hyp2_new', []); % how to select break-off swarm

% 0-maximum, 99-minimum,  any other number directly corresponds to # of
% branch
parse(parser, varargin{:});

hyp2_new = parser.Results.hyp2_new;



% model_bots1 = struct('w',[]);
model_bot_n.Sigma = model_bot_n.gamma_K./model_bot_n.alpha_K;
model_bot_n.mu = model_bot_n.belta_K./(model_bot_n.alpha_K);
model_bot_n.w = norm_prob(model_bot_n.alpha_K);

kss = zeros(1,1,num_gau);
for ijj = 1:num_gau
kss(:,:,ijj) = model_bot_n.Sigma(ijj);
end
model_bot_n.Sigma = kss;

Nm_ind = unique(model_bot_n.Nm_ind);

[label, R] = mixGaussPred_gmm(Fss(Nm_ind,:)', model_bot_n); % assume knowledge of bot1's data index set, and then classify the training data with probability

mu = zeros(length(Ftest),num_gau);
s2 = zeros(length(Ftest),num_gau);
rms = zeros(1,num_gau);

% l1 = find(label==1);
% l2 = find(label==2);
% l3 = find(label==3);
% l4 = find(label==4);
% test_indd = 501:600; % 34 for marginal comparison that shows the benefits of using GMM instead of single Gaussian (even the most appropriate one with highest probability in R)

for ijk = 1:num_gau
    
    if length(unique(find(label==ijk)))==1
        if isempty(hyp2_new)
            [mu(:,ijk), s2(:,ijk), ~, rms(ijk)] = gpml_rms([],Xss(Nm_ind,:),Fss(Nm_ind),Xtest,Ftest);
        else
            [mu(:,ijk), s2(:,ijk), ~, rms(ijk)] = gpml_rms([],Xss(Nm_ind,:),Fss(Nm_ind),Xtest,Ftest, hyp2_new);
        end
    else
        
        if isempty(hyp2_new)
            [mu(:,ijk), s2(:,ijk), ~, rms(ijk)] = gpml_rms(find(label==ijk),Xss(Nm_ind,:),Fss(Nm_ind),Xtest,Ftest);
        else
            [mu(:,ijk), s2(:,ijk), ~, rms(ijk)] = gpml_rms(find(label==ijk),Xss(Nm_ind,:),Fss(Nm_ind),Xtest,Ftest, hyp2_new);
        end
    end

end

% [mu_1, s2_1, hyp2_1, rms_1] = gpml_rms(l1,Xss,Fss,Xss(test_indd,:),Fss(test_indd));
% [mu_2, s2_2, hyp2_2, rms_2] = gpml_rms(l2,Xss,Fss,Xss(test_indd,:),Fss(test_indd));
% [mu_3, s2_3, hyp2_3, rms_3] = gpml_rms(l3,Xss,Fss,Xss(test_indd,:),Fss(test_indd));
% [mu_4, s2_4, hyp2_4, rms_4] = gpml_rms(l4,Xss,Fss,Xss(test_indd,:),Fss(test_indd));

[PP_exp, PP_out, PP] = gt_pred(Xss(Nm_ind,:), R, Xtest);   % using the first 100 as test case (by default in main_distributed_example script)

% start to filter infeasible component

pred_mu_mat = mu; % assume Ntest x num_gau
PP_out_tmp = PP_out.*~isnan(pred_mu_mat);
norm_PP_out = PP_out_tmp./sum(PP_out_tmp,2);
pred_mu_mat(isnan(pred_mu_mat)) = 0;  % set NaN elements to 0

% end of filtering

muu_pp = norm_PP_out.*pred_mu_mat;
muu_pp = sum(muu_pp,2);
% Ftest;
% pred_s2_mat = s2 + (pred_mu_mat - muu_pp).^2; % assume Ntest x num_gau
% s2_pp = norm_PP_out.*pred_s2_mat; % this version is to use norm_PP_out to
% compute variance, but could potentially decrease variance 
% s2_pp = sum(s2_pp,2);

pred_s2_mat = s2;

muu_pp_rep = repmat(muu_pp, [1, num_gau]);
pred_s2_mat(~isnan(mu)) = s2(~isnan(mu)) + (pred_mu_mat(~isnan(mu)) - muu_pp_rep(~isnan(mu))).^2; % assume Ntest x num_gau
s2_pp = PP_out.*pred_s2_mat; % note that we use PP_out instead of squeezed norm_PP_out to preserve variance
s2_pp = sum(s2_pp,2);

pred_h = muu_pp;
pred_Var = s2_pp;
pred_rms = sqrt(sum( (muu_pp - Ftest).^2 )/(length(Ftest)));



end



function y = norm_prob(X)
% X:  n x d where d is the num_gau
y = X./sum(X,2);

end