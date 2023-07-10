%% function for learning the gating function

% input: all training data X     n x d
%        all labels of probabilisty R   n x K
%        all models model      struct with K dimensions

% output: predicted probability of gating function for each cluster:
%      n_test x K
%  PP_exp:   softmax  (sum is 1)      n_test x K
%  PP_out:   standard normalize with + and -    (sum is 1)
%  PP:   raw data of gp predicted probability    (sum maybe close to 1)

function [PP_exp, PP_out, PP] = gt_pred(Xs, R, X_test)

K = size(R,2); % get number of models
N = size(Xs,1);
d = size(Xs,2);
n_test = size(X_test,1);
% start to learn gating function for each cluster

PP = zeros(n_test,K);

for ijk = 1:K
   [mu_gp, var_gp] = gpml_rms(1:N, Xs, R(:,ijk), X_test);
   PP(:,ijk) = mu_gp; 
end

PP_out = bsxfun(@rdivide, PP, sum(PP,2));

exp_PP = exp(PP);
sumexp_PP = sum(exp_PP,2);
PP_exp = bsxfun(@rdivide, exp_PP, sumexp_PP);

% 
% norm_PP = bsxfun(@minus, PP, min(PP,[],2));%    PP - min(PP,[],2);
% sum_PP = max(PP,2) - min(PP,2);
% PP = bsxfun(@rdivide, norm_PP, sum_PP);

end