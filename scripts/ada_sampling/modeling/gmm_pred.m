% using learned GMM model to predict environmental phenomenon
% Created by Lingpeng Chen(clp020528@gmail.com)
% Date: 07/14/2023
% Mixture of GPs based on converged GMM and local training data

function [pdf_values, predicted_values] = gmm_pred(Xtest, model)

    % Xtest:   Ntest x d
    num_gau = length(model.mu);
    
    variances = zeros(1, 5);
    for ijj = 1:num_gau
        variances(ijj) = model.Sigma(:,:,ijj);
    end
    
    means = model.mu;
    
    weights = model.w;
    
    % 计算每个高斯分布的概率密度函数值
    pdf_values = zeros(size(Xtest));
    for i = 1:numel(means)
        pdf_values = pdf_values + weights(i) * normpdf(Xtest, means(i), sqrt(variances(i)));
    end
    
    % 预测值即为概率密度函数值最大的高斯分布对应的均值
    [~, index] = max(pdf_values);
    predicted_values = means(index);
    
    % 输出预测值数组
    disp(predicted_values);

end


