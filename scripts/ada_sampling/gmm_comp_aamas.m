%%%  Script to prove effectiveness of pre-process using GMM 

% load sample_data_new.mat

load sample_data_stalk_count.mat

rng(500)
bat_train = floor(linspace(10,100,19));
bat_sample = randperm(length(Fss), 100);

num_train = length(bat_train);

rms_gpml_comp = zeros(1,num_train);
rms_gmm_comp = zeros(1,num_train);
num_gau = 3;

% [label_r, model_r, llh_r] = mixGaussEm_rss(Fss', num_gau); % prior knowledge of ground-truth label for the training data (not required)

hyp2_new.mean = [];
hyp2_new.cov = [0.5 0.5];
hyp2_new.lik = -2;

% for new agriculture dataset
% hyp2_new.mean = [];
% hyp2_new.cov = [-0.3 2.5];
% hyp2_new.lik = 1.833;
% 
% hyp2_new=[];


num_rand_round = 10;
rand_seed_set = 50*[1:num_rand_round]; %20
rms_gpml_comp = zeros(num_train,num_rand_round);
rms_gmm_comp = zeros(num_train,num_rand_round);
toc_gpml_comp = zeros(num_train,num_rand_round);
toc_gmm_comp = zeros(num_train,num_rand_round);


for jjj = 1:num_rand_round
    
    rng(rand_seed_set(jjj))
    bat_sample = randperm(length(Fss), 100);
    for ijk =1:num_train
        jjj
        ijk
        idx_train = bat_sample(1:bat_train(ijk));   % get training samples at each number of samples
        idx_test = 1:length(Fss); %setdiff(1:length(Fss),idx_train);  % all the rest samples are for testing purpose
        
        tic
        [~, ~, ~, rms_gpml_comp(ijk,jjj)] = gpml_rms(idx_train,Xss,Fss,Xss(idx_test,:),Fss(idx_test),hyp2_new);
        toc_gpml_comp(ijk,jjj) = toc;
        
        tic
        [label_rss, model_rss, llh_rss] = mixGaussEm_rss(Fss(idx_train)', num_gau); % use EM for
        
        %   [label_rss, model_rss, llh_rss] = mixGaussEm_rss(Fss(idx_train)',
        %   label_r(idx_train)); % use this only if we want to use prior knowlege
        %   of labels for each training sample
        
        [pred_h, pred_Var, rms_gmm_comp(ijk,jjj)] = gmm_pred_cen(Xss(idx_train,:), Fss(idx_train), Xss(idx_test,:), Fss(idx_test), model_rss, hyp2_new);
        toc_gmm_comp(ijk,jjj) = toc;
        
    end
end


    
% figure(120);plot(rms_gpml_comp,'b-')
% figure(120);hold on;plot(rms_gmm_comp,'r-');

lineStyles = linspecer(10); 

rms_gmm_mean = mean(rms_gmm_comp');
rms_gpml_mean = mean(rms_gpml_comp');

rms_gmm_max = max(rms_gmm_comp');
rms_gpml_max = max(rms_gpml_comp');
rms_gmm_min = min(rms_gmm_comp');
rms_gpml_min = min(rms_gpml_comp');


idx_plo = bat_train; %1:numel(rms_gpml_comp);
font_size = 20;
figure;  % plot RMS error first
plot(idx_plo,rms_gmm_mean,'Color',lineStyles(10,:),'LineWidth',2,'LineStyle','-');
hold on;
plot(idx_plo,rms_gpml_mean,'Color',lineStyles(2,:),'LineWidth',2,'LineStyle','-');
hold on;
f = [rms_gmm_min fliplr(rms_gmm_max)];
fill([idx_plo fliplr(idx_plo)], f,lineStyles(10,:),'linestyle','none');
alpha(.2)
hold on;
f = [rms_gpml_min fliplr(rms_gpml_max)];
fill([idx_plo fliplr(idx_plo)], f,lineStyles(2,:),'linestyle','none');
alpha(.2)

% f = [rms_gmm_min fliplr(rms_gmm_max)];
% fill([idx_plo fliplr(idx_plo)], f,'Color',lineStyles(10,:));
% alpha(.3)
% hold on;
% plot(idx_plo,rms_gmm_min,'Color',lineStyles(7,:),'LineWidth',2,'LineStyle','-');
% hold on;
% plot(idx_plo,rms_gpml_min,'Color',lineStyles(8,:),'LineWidth',2,'LineStyle','-');

set(gca,'LineWidth',2,'fontsize',font_size,'fontname','Times','FontWeight','Normal');
legend('Mixture of GPs','Uni-model GP');
xlabel('Number of Training Samples','FontName','Times New Roman','FontSize',font_size);
ylabel('RMS Error','FontName','Times New Roman','FontSize',font_size);
set(gca,'linewidth',2,'fontsize',font_size,'fontname','Times');
% set(gca,'XTickLabel',{'10';'15';'20';'25';'30';'35';'40';'45';'50';'55';'60';'65';'70';'75';'80';'85';'90';'95';'100'}, 'FontSize', 20)



% f = [rms_gmm_max; flipdim(rms_gmm_min,1)];
%   fill([idx_plo; flipdim(idx_plo,1)], f, [7 7 7]/8);
    
  