%% mixture of gp learning
%% final version

% input: all model structure with K dimension
%        all training data Xs   N x d
%        testing data X_test
%        R the probability label for each model for all train data  N x K

% output:  predicted value from weighted sum of all gp models

% 
% ind_gate = 1:16
% K = 3;
% 
% R = zeros(length(Fs),K);
% R(ind_gate,1) = 1
% ind_gate = 17:32;
% R(ind_gate,2) = 1;
% ind_gate = 33:length(Fs);
% R(ind_gate,3) = 1;

function [pred_h, pred_Var, pred_rms] = gpm_pred(Xs, model, R, Xtest, Ftest)
K = length(model);
N = size(Xs,1);
ntest = length(Ftest);
% n_test = size(X_test,1);

% R = zeros(N,K);
% 
% for ijk = 1:K
%     ind_act = model(ijk).ind;
%     R(ind_)
% end

% get the learned gating function value for each testing data w.r.t. each
% GP experts
[PP_exp, PP_out, PP] = gt_pred(Xs, R, Xtest);
good_PP = PP; % choose which learned gating function to use, PP_exp


pred_f = zeros(ntest, K); % n_test x K predicted mean for each data from each GP experts
pred_var = zeros(ntest,K);  % n-test x K predicted variance for each data from each GP experts
pred_ucb = zeros(ntest,K);

for ijk = 1:K
    pred_f(:,ijk) = model(ijk).mu;   % instead, we can also use gpopt_em to recompute the mu and var if we only want the pred value over Xtest
    pred_var(:,ijk) = model(ijk).var;
    pred_ucb(:,ijk) = model(ijk).ucb;
%     [pred_f(:,ijk), pred_var(:,ijk)] = gpml_rms(model(ijk).ind, Xs, Fs, X_test);
end

pred_h = good_PP.*pred_f;
pred_h = sum(pred_h,2); % n_test x 1 final predicted value

delta_matrix = bsxfun(@minus, pred_f, pred_h);
delta_matrix = delta_matrix.^2; % n_test x K delta squared term  (f-h)^2

pred_Var = sum((pred_var + delta_matrix).*good_PP, 2);

if nargin == 5
    pred_rms = sqrt(sum( (pred_h - Ftest).^2 )/(length(Ftest)));
end


end