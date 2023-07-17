figure;  % 创建一个新的图形窗口
load('ship_trajectory.mat'); % F_map(200,100) map_length map_width targets(8*2)
Fss = reshape(F_map, [], 1);
% Fss = Fss(Fss ~= 0);
% 生成正态分布的随机数

noise = normrnd(0.05, 0.03, size(Fss)); % 均值为0，标准差为0.05
% 在0附近的元素减去正态分布


Fss_zero_indices = (Fss == 0); % 选择 Fss 中为0的元素的逻辑索引
Fss(Fss_zero_indices) = Fss(Fss_zero_indices) - noise(Fss_zero_indices);

for index = 1:1
    subplot(1, 1, index);  % 将图像分成4行5列，并选择当前子图
    

    num_gau = 5;

        [label_rss, model_rss, ~] = mixGaussEm_gmm(Fss', num_gau); % centralized GMM version
        [label, R] = mixGaussPred_gmm(Fss', model_rss);
        % 使用 unique 函数获取数组中的唯一元素
        uniqueElements = unique(label_rss);
        
        % 使用 histcounts 函数统计每个唯一元素的个数
        counts = histcounts(label_rss, 1:(num_gau+1));
        
        % 显示每个唯一元素及其个数
        for i = 1:num_gau
            fprintf('Gaussian %d has：%d, Sigma=%d, mu=%d\n', i, counts(i), model_rss.Sigma(:,:,i), model_rss.mu(i) );
        end
        
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
        
        
        % 绘制直方图
        histogram(samples, 'Normalization', 'pdf','BinWidth', 0.01, 'DisplayName', 'Samples');
        histogram(Fss, 'Normalization', 'pdf', 'BinWidth', 0.01, 'DisplayName', 'Fss');
        hold off;
        xlabel('Value');
        ylabel('Probability Density');
        legend;
        
        
        % 进行 Kolmogorov-Smirnov 检验
        [h, p, ksstat] = kstest2(samples, Fss);
        title(['GMM component: ', num2str(num_gau), ', KS-div: ', num2str(ksstat)]);  % 在图像上显示索引

end

% 调整子图的布局
set(gcf, 'Position', get(0, 'Screensize'));  % 全屏显示图形窗口
set(gcf, 'PaperPositionMode', 'auto');  % 调整打印位置模式

% 显示所有绘制的图像
print(gcf, '-dpng', 'all_plots.png');  % 保存图像为 PNG 文件
