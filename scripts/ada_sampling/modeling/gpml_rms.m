function [mu, s2, hyp2, rms] = gpml_rms(ind_train,Xs,Fs,Xtest_new,Ftest_new, hyp2_given)

% Modified from GPML Matlab Toolbox by Rasmussen and Williams
% create by Wenhao Luo  01/28/2018 (whluo12@gmail.com)
% Xs   -   N x d  coordinates for all grids
% Fs   -   N x 1  Realization of all grids (such as temperature readings)
% ind_train - N_train x 1  index of training data
% Xtest_new  - N_test x d  coordinates for all testing grids
% Ftest_new  - N_test x 1  Realization of all testing grids (ground truth)

% output:
% mu - N_test x 1   Predicted value on all testing grids
% s2 - N_test x 1   Variance of prediction on all testing grids
% hyp2 - hyperparameters of GP model
% rms - 1 x 1     RMS error from ground truth

meanfunc = [];                    % empty: don't use a mean function
covfunc = @covSEiso;              % Squared Exponental covariance function
likfunc = @likGauss;              % Gaussian likelihood
hyp2 = struct('mean', [], 'cov', [0 0], 'lik', -1);
covfunc = @covSEiso; hyp2.cov = [0; 0]; hyp2.lik = log(0.1);

% ind_train = ind_mi_seg;

ind_train = unique(ind_train);

hyp2 = minimize(hyp2, @gp, -100, @infGaussLik, [], covfunc, likfunc, Xs(ind_train,:), Fs(ind_train)-mean(Fs(ind_train)));
if nargin == 6
    if ~isempty(hyp2_given)
        hyp2 = hyp2_given;    
    end
end

[mu, s2] = gp(hyp2, @infGaussLik, [], covfunc, likfunc, Xs(ind_train,:), Fs(ind_train)-mean(Fs(ind_train)), Xtest_new);
mu = mu + mean(Fs(ind_train));
if nargin > 4
rms = sqrt(sum( (mu - Ftest_new).^2 )/(length(Ftest_new)));
end

end