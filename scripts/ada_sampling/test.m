
load('ship_trajectory.mat'); % F_map(200,100) map_length map_width targets(8*2)
Fss = reshape(F_map, [], 1);

    
num_gau = 10;
[label_rss, model_rss, ~] = mixGaussEm_gmm(Fss', num_gau); % centralized GMM version

% 高斯混合模型参数
mu =  model_rss.mu;
sigma = reshape(model_rss.Sigma, 1, []); % 将 Sigma 转换为列表
w = model_rss.w;

% 采样次数
numSamples = 20000;

% 生成采样数据
samples = [];
for i = 1:numSamples
    % 通过随机选择一个组件
    component = randsample(1:length(w), 1, true, w);
    
    % 根据所选组件的均值和方差进行采样
    sample = normrnd(mu(component), sqrt(sigma(component)));
    
    % 将采样结果添加到样本数组中
    samples = [samples, sample];
end



% 绘制每个组件的高斯曲线
hold on;
x = linspace(min(samples), max(samples), 100);
for i = 1:length(mu)
    y = normpdf(x, mu(i), sqrt(sigma(i)));
    plot(x, y, 'LineWidth', 2);
end
hold off;


% 绘制直方图
figure;
histogram(samples, 'Normalization', 'pdf','BinWidth', 0.01, 'DisplayName', 'Samples');
hold on;
histogram(Fss, 'Normalization', 'pdf', 'BinWidth', 0.01, 'DisplayName', 'Fss');
hold off;
xlabel('Value');
ylabel('Probability Density');
title('Distribution Comparison');
legend;






% 进行 Kolmogorov-Smirnov 检验
[h, p, ksstat] = kstest2(samples, Fss);

% 显示结果
fprintf('Kolmogorov-Smirnov test:\n');
fprintf('   KS statistic: %.4f\n', ksstat);
fprintf('   p-value: %.4f\n', p);
if h
    fprintf('   The distributions are significantly different.\n');
else
    fprintf('   The distributions are not significantly different.\n');
end

% 可以进一步比较直方图之间的差异，如柱子差异、柱子高度差异等

