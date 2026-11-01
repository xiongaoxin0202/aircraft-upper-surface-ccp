%% collect_baseline_data.m（Code2 版）
% 任务0：基线数据收集与"手稿一致性"确认
% ------------------------------------------------------------
% 用途：
%   1) 遍历 Code2/coverage_threshold_results/ 下所有结果子文件夹，
%      汇总各覆盖率阈值下的：视点数、覆盖率、路径长度、转弯角、规划时间；
%   2) 与投稿手稿 Table 3 的数据对比，确认"Code2 = 手稿数据来源"；
%   3) 输出 baseline_summary.csv 作为后续统计/消融实验的统一基线表。
%
% 已确认（2026-08-31 核对）：Code2 的 analysis_comprehensive_results/Data/
% analysis_data.csv 与手稿 Table 3 完全一致（如 τ=95%: 29 视点 /
% 202.30 m / 1140.91° / 99.00% / 21.16 s）。
% 本脚本用于复现该确认过程并生成基线表。
%
% 使用方法：
%   1) 将本脚本放入 Code2/ 目录（与 coverage_threshold_results 同级）；
%   2) 在 MATLAB 中运行（或 matlab -batch "collect_baseline_data"）；
%   3) 查看生成的 baseline_summary.csv 与 baseline_vs_manuscript.csv。
% ------------------------------------------------------------

clc; clear; close all;

%% === 1. 配置 ===
results_root = 'coverage_threshold_results';
output_csv   = 'baseline_summary.csv';
compare_csv  = 'baseline_vs_manuscript.csv';

% 手稿 Table 3（视点数 / 路径长度m / 转弯角° / 覆盖率% / 时间s）
% 列顺序: tau, viewpoints, path_len, turn_angle, coverage, time
manuscript_table3 = [
    0.80, 18, 170.0, 766.6,  93.4, 18.4;
    0.85, 20, 187.2, 773.9,  96.5, 20.1;
    0.90, 24, 193.7, 952.8,  98.5, 20.5;
    0.95, 29, 202.3, 1140.9, 99.0, 21.1;
    1.00, 67, 295.6, 2284.2, 100.0, 25.2
];

%% === 2. 从 PATH_TJ 输出的分析 CSV 读取（若存在）===
analysis_csv = fullfile('analysis_comprehensive_results', 'Data', 'analysis_data.csv');
if exist(analysis_csv, 'file')
    T = readtable(analysis_csv);
    % 列: Threshold, ViewpointCov, PathCov, NumVP, PathLen, TotalTurn, VPTime, PathTime, TotalTime
    fprintf('已从 PATH_TJ 分析结果读取 %d 行数据: %s\n', height(T), analysis_csv);
    B = table();
    B.tau = T.Threshold;
    if max(B.tau) > 1.5, B.tau = B.tau / 100; end  % Threshold 列若为百分数则归一化
    B.num_viewpoints = T.NumVP;
    B.path_length_m = round(T.PathLen, 1);
    B.turn_angle_deg = round(T.TotalTurn, 1);
    B.path_coverage_pct = T.PathCov;
    B.time_total_s = round(T.TotalTime, 1);
    writetable(B, output_csv);
    fprintf('基线汇总已保存: %s\n', output_csv);
    disp(B);
else
    % 回退：从各阈值文件夹读取
    fprintf('未找到 %s，改为遍历结果文件夹\n', analysis_csv);
    dirs = dir(fullfile(results_root, 'coverage_*'));
    dirs = dirs([dirs.isdir]);
    rows = [];
    for i = 1:length(dirs)
        d = dirs(i).name;
        tokens = regexp(d, 'coverage_([0-9.]+)', 'tokens');
        if isempty(tokens), continue; end
        tau = str2double(tokens{1}{1});
        s_file = fullfile(results_root, d, 'summary.txt');
        if ~exist(s_file, 'file'), continue; end
        s_txt = fileread(s_file);
        n_vp = regexp(s_txt, '最终视点数量:\s*(\d+)', 'tokens');
        cov  = regexp(s_txt, '总覆盖率:\s*([0-9.]+)%', 'tokens');
        t_sel = regexp(s_txt, '处理时间:\s*([0-9.]+)\s*秒', 'tokens');
        if isempty(n_vp), continue; end
        rows = [rows; tau, str2double(n_vp{1}{1}), str2double(cov{1}{1}), str2double(t_sel{1}{1})];
    end
    rows = sortrows(rows, 1);
    B = array2table(rows, 'VariableNames', {'tau','num_viewpoints','viewpoint_coverage_pct','time_select_s'});
    writetable(B, output_csv);
    fprintf('基线汇总已保存: %s\n', output_csv);
    disp(B);
end

%% === 3. 与手稿 Table 3 对比 ===
fprintf('\n========== 与手稿 Table 3 数据对比 ==========\n');
comp = [];
for i = 1:size(manuscript_table3, 1)
    tau_ms = manuscript_table3(i, 1);
    idx = find(abs(B.tau - tau_ms) < 1e-6, 1);
    if isempty(idx)
        fprintf('手稿阈值 %.2f 在当前结果中未找到\n', tau_ms);
        continue;
    end
    comp = [comp; tau_ms, manuscript_table3(i,2), B.num_viewpoints(idx), ...
                   manuscript_table3(i,3), B.path_length_m(idx), ...
                   manuscript_table3(i,5), B.path_coverage_pct(idx), ...
                   manuscript_table3(i,6), B.time_total_s(idx)];
end

if ~isempty(comp)
    C = array2table(comp, 'VariableNames', ...
        {'tau','ms_viewpoints','cur_viewpoints','ms_path_m','cur_path_m', ...
         'ms_coverage_pct','cur_coverage_pct','ms_time_s','cur_time_s'});
    C.viewpoint_ratio = C.cur_viewpoints ./ C.ms_viewpoints;
    C.path_ratio      = C.cur_path_m ./ C.ms_path_m;
    C.time_ratio      = C.cur_time_s ./ C.ms_time_s;
    writetable(C, compare_csv);
    fprintf('对比表已保存: %s\n', compare_csv);
    disp(C);
    if all(abs(C.viewpoint_ratio - 1) < 1e-9) && all(abs(C.path_ratio - 1) < 1e-3)
        fprintf('\n>>> 确认：Code2 结果与手稿 Table 3 一致（比值≈1）。基线 = Code2。\n');
        fprintf('>>> 可继续进行任务 1-4（统计检验/消融/泛化/分辨率敏感性）。\n');
    else
        fprintf('\n>>> 注意：存在不一致，请检查 Code2 版本或重新运行 MAIN_viewpoint.m / MAIN_path_one.m。\n');
    end
end

fprintf('\n任务0 完成。\n');
