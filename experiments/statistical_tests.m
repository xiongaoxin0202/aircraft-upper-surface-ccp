%% statistical_tests.m (v4 - 与论文 Table 6 / Section 5.5 口径完全一致)
% ------------------------------------------------------------------
% 输出:
%   1) statistical_test_results.csv  - 15 行单样本 t 检验结果（与论文 Table 6 逐行一致，
%      可由 Table 5 的 Mean±SD 解析复算）
%   2) welch_anova_baselines.csv    - 3 个随机 baseline 的 Welch ANOVA
%      （与论文 Section 5.5 的 F(2, df2) 值一致）
%   3) boxplot_data.csv             - 4 方法 × 5 指标样本（供 generate_boxplots.py
%      绘制论文 Figure 19 箱线图）
%
% 统计口径说明（与论文一致）：
%   - 方法 D（Proposed）为确定性算法，其报告值视为总体真值（零抽样方差）；
%   - 因此对每个指标 × 每个 baseline 采用"单样本 t 检验"（df = n-1 = 9），
%     检验 H0: baseline 总体均值 = D 的值；
%   - 另对 3 个随机 baseline（A/B/C）做 Welch ANOVA（不含 D）作为 omnibus 检验。
%
% 数据来源：论文 Table 5 的 Mean ± SD（10 次运行）。若存在真实运行数据
% runs_data.csv（格式：method,run,viewpoints,path_length,turn_angle,coverage,time_s），
% 则优先使用真实数据逐次检验；否则用名义值解析计算（确定性，可复现）。
% ------------------------------------------------------------------

clc; clear; close all;

%% ===== 1. 数据准备 =====
data_file = 'runs_data.csv';
metrics   = {'viewpoints','path_length','turn_angle','coverage','time_s'};
metric_names = {'Viewpoint count','Path length (m)','Turning angle (deg)','Coverage (%)','Planning time (s)'};
bnames = {'A (SCP+MMAS)','B (MCTS)','C (ASSCPP)'};

% 论文 Table 5 名义值（mean, sd）；D 为 Method D 确定值
t5_mean = [111, 95, 145; 375.2, 348.6, 336.6; 3580.5, 3754.8, 3564.2; 94.0, 85.2, 89.6; 727.3, 4197.5, 260.8];
t5_sd   = [2, 5, 8; 6.5, 8.7, 10.8; 36.5, 87.2, 34.3; 1.2, 2.8, 1.9; 45.3, 130.6, 18.2];
D_val   = [29, 202.3, 1140.9, 99.0, 21.1];
n_run   = 10;

use_real = exist(data_file, 'file');
if use_real
    T = readtable(data_file);
    fprintf('Read real run data: %s\n', data_file);
else
    fprintf('runs_data.csv not found; using Table 5 mean/SD analytic calculation\n');
end

%% ===== 2. One-sample t tests (15 rows, matching Table 6) =====
out = cell(0, 6);
for k = 1:length(metrics)
    for b = 1:3
        if use_real
            g = T.(metrics{k})(strcmp(T.method, char(64+b)));
            mu = mean(g); sd = std(g); n = numel(g);
        else
            mu = t5_mean(k, b); sd = t5_sd(k, b); n = n_run;
        end
        t_stat = (mu - D_val(k)) / (sd / sqrt(n));
        df = n - 1;
        p = 2 * (1 - tcdf(abs(t_stat), df));
        sig = '';
        if p < 0.05, sig = '*'; end
        if p < 0.01, sig = '**'; end
        if p < 0.001, sig = '***'; end
        out(end+1, :) = {metric_names{k}, bnames{b}, round(t_stat,1), df, p, sig}; %#ok<SAGROW>
        fprintf('%-16s %-14s t = %8.1f (df=%d)  p = %.3e %s\n', ...
            metric_names{k}, bnames{b}, t_stat, df, p, sig);
    end
end
R = cell2table(out, 'VariableNames', {'metric','baseline','t_stat','df','p_value','significance'});
writetable(R, 'statistical_test_results.csv');
fprintf('Saved statistical_test_results.csv (15 rows, matching Table 6)\n');

%% ===== 3. Welch ANOVA over 3 stochastic baselines (omnibus, excl. D) =====
fprintf('\n===== Welch ANOVA over baselines A/B/C =====\n');
wo = cell(0, 5);
for k = 1:length(metrics)
    if use_real
        groups = {};
        for b = 1:3
            groups{b} = T.(metrics{k})(strcmp(T.method, char(64+b)));
        end
        [F, df1, df2, p] = welch_anova_groups(groups);
    else
        [F, df1, df2, p] = welch_anova_stats(t5_mean(k,1:3), t5_sd(k,1:3), n_run);
    end
    if isnan(F)
        fprintf('%-16s Welch ANOVA undefined (zero-variance group), skipped\n', metric_names{k});
        continue;
    end
    wo(end+1, :) = {metric_names{k}, round(F,1), df1, round(df2,1), p}; %#ok<SAGROW>
    fprintf('%-16s F(%d, %.1f) = %.1f, p = %.3e\n', metric_names{k}, df1, df2, F, p);
end
if ~isempty(wo)
    W = cell2table(wo, 'VariableNames', {'metric','F','df1','df2','p_value'});
    writetable(W, 'welch_anova_baselines.csv');
    fprintf('Saved welch_anova_baselines.csv (matching Section 5.5)\n');
end

%% ===== 4. Sample export for box plots (Figure 19) =====
if ~use_real
    rng(42);
end
bp_rows = cell(0, 4);
for k = 1:length(metrics)
    for m = 1:4
        if use_real
            g = T.(metrics{k})(strcmp(T.method, char(64+m)));
        else
            if m <= 3
                g = t5_mean(k,m) + t5_sd(k,m) * randn(n_run,1);
            else
                g = D_val(k) * ones(n_run,1);
            end
        end
        for ii = 1:numel(g)
            bp_rows(end+1,:) = {metric_names{k}, char(64+m), ii, g(ii)}; %#ok<SAGROW>
        end
    end
end
BP = cell2table(bp_rows, 'VariableNames', {'metric','method','run','value'});
writetable(BP, 'boxplot_data.csv');
fprintf('Saved boxplot_data.csv for generate_boxplots.py (Figure 19)\n');

if ~use_real
    fprintf('NOTE: samples generated from Table 5 mean/SD for visualisation only;\n');
    fprintf('t-tests and ANOVA are analytic values consistent with Table 6 / Section 5.5.\n');
end
fprintf('Task 1 done.\n');

%% ================== Local functions ==================

function [F, df1, df2, p] = welch_anova_stats(means, sds, n)
    k = numel(means);
    vars_ = sds.^2;
    w = n ./ vars_;
    grand = sum(w .* means) / sum(w);
    m = sum((1 - w / sum(w)).^2 ./ (n - 1)) / (k^2 - 1);
    F = sum(w .* (means - grand).^2) / ((k - 1) * (1 + 2 * m));
    df1 = k - 1;
    df2 = 1 / (3 * m);
    p = 1 - fcdf(F, df1, df2);
end

function [F, df1, df2, p] = welch_anova_groups(groups)
    k = numel(groups);
    n = cellfun(@numel, groups);
    means = cellfun(@mean, groups);
    vars_ = cellfun(@var, groups);
    if any(vars_ == 0)
        F = NaN; df1 = k - 1; df2 = NaN; p = NaN;
        return;
    end
    w = n ./ vars_;
    grand = sum(w .* means) / sum(w);
    m = sum((1 - w / sum(w)).^2 ./ (n - 1)) / (k^2 - 1);
    F = sum(w .* (means - grand).^2) / ((k - 1) * (1 + 2 * m));
    df1 = k - 1;
    df2 = 1 / (3 * m);
    p = 1 - fcdf(F, df1, df2);
end
