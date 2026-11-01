%% generalization_experiments.m（Code2 版）
% 任务3：多机型泛化实验（对应审稿意见 R1-5/R1-8：单一机型、泛化性有限）
% 基线：Code2 目录（FOV + Z-Buffer 模型，参数与手稿一致）
% ------------------------------------------------------------
% 在多个机型点云上运行简化版覆盖路径规划管线：
%   体素下采样 -> 法线估计 -> 全局朝外重定向 -> 上表面提取 ->
%   视点外推(按机型尺度缩放) -> 距离过滤 -> 贪心选择(τ=95%) -> 路径指标
% 机型：A340（宽体）、B757（窄体）、里-2（运输机）；对照：A350（论文主模型）
%
% 使用方法：
%   1) 将本脚本放入 Code2/ 目录（机型点云放 Code2/3D_dataset/，可自建）；
%   2) 修改下方 model_list 增加/删减机型；
%   3) MATLAB 中运行（或 matlab -batch "generalization_experiments"）。
%   运行时长：每机型 5-15 分钟（取决于点云规模与候选视点数）。
%
% 注意：
%   - 本脚本使用"全局法线 + 质心朝外重定向"作为简化处理；
%     正式论文中建议对每个机型重新运行 ACNS 分割（MAIN_UP.m 管线）以保持一致，
%     或如实说明多机型实验采用全局重定向以验证尺度适应性；
%   - 距离参数按机型特征尺寸自动缩放（scale = 包围盒最大边长/66.8m）。
% ------------------------------------------------------------

clc; clear; close all;

%% === 配置 ===
model_list = {
    '3D_dataset/A340.pcd',          'A340';
    '3D_dataset/pc_airliner757.ply', 'B757';
    '3D_dataset/里-2.ply',          'Li-2'
};
% A350 对照：直接读 Code2 已有结果
a350_folder = 'coverage_threshold_results/coverage_0.95';

% 通用参数（与 Code2 MAIN_viewpoint.m 一致）
fov_h = 75; fov_v = 65;
near_plane = 0.5; far_plane = 5.46;
min_safe_distance = 3.5;   % A350 基准安全距离（米），按机型缩放
coverage_threshold = 0.95;
max_candidates = 3000;     % 候选视点上限（控制计算量）
voxel_size = 0.1;          % 体素下采样尺寸（米）

rows = cell(0, 8);

%% === 逐机型运行 ===
for m = 1:size(model_list, 1)
    fpath = model_list{m, 1};
    mname = model_list{m, 2};
    fprintf('\n========== 机型: %s (%s) ==========\n', mname, fpath);
    if ~exist(fpath, 'file')
        fprintf('文件不存在，跳过（请将机型点云放入 Code2/3D_dataset/）\n');
        continue;
    end

    % 1) 加载 + 体素下采样
    pc = pcread(fpath);
    if ~isempty(voxel_size)
        pc = pcdownsample(pc, 'gridAverage', voxel_size);
    end
    pts = pc.Location;
    fprintf('下采样后点数: %d\n', size(pts, 1));

    % 2) 特征尺度（包围盒最大边长），A350 特征长度约 66.8m
    bbox_len = max(max(pts) - min(pts));
    scale = bbox_len / 66.8;
    fprintf('特征长度: %.1f m, 尺度因子: %.2f\n', bbox_len, scale);

    % 3) 法线估计 + 全局朝外重定向
    nrm = pcnormals(pc);
    centroid = mean(pts, 1);
    dir_vec = pts - centroid;
    flip = dot(nrm, dir_vec, 2) < 0;
    nrm(flip, :) = -nrm(flip, :);
    nrm = nrm ./ vecnorm(nrm, 2, 2);

    % 4) 上表面提取（法线朝上的点）
    up_mask = nrm(:, 3) > 0.3 & (pts(:, 3) > centroid(3));
    if sum(up_mask) < 1000
        fprintf('上表面点数过少 (%d)，改用全模型\n', sum(up_mask));
        up_mask = true(size(pts, 1), 1);
    end
    target_pts = pts(up_mask, :);
    target_nrm = nrm(up_mask, :);
    fprintf('上表面点数: %d\n', size(target_pts, 1));

    % 5) 视点外推（按尺度缩放）+ 随机抽样子集
    d_safe = min_safe_distance * scale;
    cand_vp = target_pts + target_nrm * d_safe;
    n_cand_all = size(cand_vp, 1);
    if n_cand_all > max_candidates
        rng(1);
        sel_idx = randperm(n_cand_all, max_candidates);
        cand_vp = cand_vp(sel_idx, :);
        cand_nrm = target_nrm(sel_idx, :);
    else
        cand_nrm = target_nrm;
    end
    fprintf('候选视点: %d (总 %d), d_safe=%.2f m\n', size(cand_vp,1), n_cand_all, d_safe);

    % 6) 距离过滤（剔除侵入机体的视点）
    kd_tree = KDTreeSearcher(target_pts);
    [~, dist] = knnsearch(kd_tree, cand_vp);
    keep = dist >= d_safe * 0.8;
    cand_vp = cand_vp(keep, :); cand_nrm = cand_nrm(keep, :);
    fprintf('过滤后候选视点: %d\n', size(cand_vp, 1));

    % 7) 覆盖矩阵（FOV + Z-Buffer）+ 贪心选择（τ=95%）
    n_pts = size(target_pts, 1);
    n_cand = size(cand_vp, 1);
    cov_mat = false(n_pts, n_cand);
    tic;
    for i = 1:n_cand
        cov_mat(:, i) = computeCoverageWithOcclusion(cand_vp(i,:), -cand_nrm(i,:), ...
            target_pts, near_plane, far_plane, fov_h, fov_v, [], []);
    end
    t_cov = toc;
    fprintf('覆盖矩阵计算耗时: %.1f s\n', t_cov);

    tic;
    [sel_vp, ~, covered, ~] = greedy_select(cov_mat, cand_vp, cand_nrm, coverage_threshold, 500);
    t_greedy = toc;
    cov_pct = mean(covered) * 100;
    fprintf('视点: %d, 覆盖率: %.2f%%, 贪心耗时: %.1f s\n', size(sel_vp, 1), cov_pct, t_greedy);

    % 8) 路径指标（NN）
    path_len = NaN; turn_ang = NaN;
    if size(sel_vp, 1) >= 3
        dist_matrix = pdist2(sel_vp, sel_vp);
        order = solveTSP(dist_matrix);
        path_len = calculatePathCost(order, dist_matrix);
        ordered = sel_vp(order, :);
        turn_ang = total_turn_angle(ordered);
        fprintf('路径长度: %.1f m, 总转弯角: %.1f°\n', path_len, turn_ang);
    end

    rows(end+1,:) = {mname, bbox_len, size(target_pts,1), size(sel_vp,1), cov_pct, path_len, turn_ang, t_cov+t_greedy}; %#ok<SAGROW>
end

%% === 并入 A350 对照 ===
if exist(fullfile(a350_folder, 'summary.txt'), 'file')
    s_txt = fileread(fullfile(a350_folder, 'summary.txt'));
    n_vp = regexp(s_txt, '最终视点数量:\s*(\d+)', 'tokens');
    cov  = regexp(s_txt, '总覆盖率:\s*([0-9.]+)%', 'tokens');
    rows(end+1,:) = {'A350 (主模型)', 66.8, NaN, str2double(n_vp{1}{1}), str2double(cov{1}{1}), NaN, NaN, NaN}; %#ok<SAGROW>
end

%% === 保存汇总 ===
if ~isempty(rows)
    R = cell2table(rows, 'VariableNames', ...
        {'model','feature_length_m','num_target_points','num_viewpoints','coverage_pct','path_length_m','turn_angle_deg','planning_time_s'});
    writetable(R, 'generalization_results.csv');
    fprintf('\n泛化实验汇总已保存: generalization_results.csv\n');
    disp(R);
end

fprintf('\n任务3 完成。\n');

%% ================== 局部函数（与 Code2 一致） ==================

function coverage = computeCoverageWithOcclusion(viewpoint, direction, points, near, far, fov_h, fov_v, ~, ~)
    N = size(points, 1);
    coverage = false(N, 1);
    vdir = direction(:)' / norm(direction);
    rel_vec = points - viewpoint;
    dist_proj = rel_vec * vdir';
    in_range = (dist_proj >= near & dist_proj <= far);
    if ~any(in_range), return; end
    [u_vec, v_vec] = buildCameraBasis(vdir);
    h_comp = rel_vec * u_vec';
    v_comp = rel_vec * v_vec';
    tan_half_h = tand(fov_h / 2);
    tan_half_v = tand(fov_v / 2);
    h_limit = dist_proj * tan_half_h;
    v_limit = dist_proj * tan_half_v;
    in_fov = (abs(h_comp) <= h_limit) & (abs(v_comp) <= v_limit);
    in_frustum = in_range & in_fov;
    frustum_idx = find(in_frustum);
    n_frustum = length(frustum_idx);
    if n_frustum == 0, return; end
    f_dists = sqrt(sum((points(frustum_idx, :) - viewpoint).^2, 2));
    f_dist_proj = dist_proj(frustum_idx);
    h_norm = h_comp(frustum_idx) ./ (f_dist_proj * tan_half_h + eps);
    v_norm = v_comp(frustum_idx) ./ (f_dist_proj * tan_half_v + eps);
    farthest_visible = min(max(f_dists), far);
    min_angular_res = atand(0.03 / max(farthest_visible, 1));
    grid_h = max(100, min(800, ceil(fov_h / min_angular_res)));
    grid_v = max(67, min(600, ceil(fov_v / min_angular_res)));
    h_idx = round((h_norm + 1) / 2 * (grid_h - 1)) + 1;
    v_idx = round((v_norm + 1) / 2 * (grid_v - 1)) + 1;
    h_idx = max(1, min(grid_h, h_idx));
    v_idx = max(1, min(grid_v, v_idx));
    linear_idx = (v_idx - 1) * grid_h + h_idx;
    [sorted_cells, sort_order] = sort(linear_idx);
    sorted_dists = f_dists(sort_order);
    cell_boundaries = find(diff([0; sorted_cells]) > 0);
    n_occupied = length(cell_boundaries);
    cell_starts = cell_boundaries;
    cell_ends = [cell_boundaries(2:end) - 1; length(sorted_cells)];
    visible_mask = false(n_frustum, 1);
    for c = 1:n_occupied
        range = cell_starts(c):cell_ends(c);
        [~, local_argmin] = min(sorted_dists(range));
        global_idx = sort_order(range(local_argmin));
        visible_mask(global_idx) = true;
    end
    coverage(frustum_idx(visible_mask)) = true;
end

function [u, v] = buildCameraBasis(n)
    n = n(:)';
    if abs(n(3)) < 0.9
        u = cross(n, [0, 0, 1]);
    else
        u = cross(n, [0, 1, 0]);
    end
    u = u / norm(u);
    v = cross(n, u);
    if norm(v) < 1e-12
        u = [1, 0, 0];
        v = [0, 1, 0];
    else
        v = v / norm(v);
    end
end

function [sel_vp, sel_nrm, covered, iter] = greedy_select(cov_mat, cand_pts, cand_nrm, tau, max_iter)
    n_pts = size(cov_mat, 1);
    n_cand = size(cov_mat, 2);
    sel_vp = []; sel_nrm = [];
    covered = false(n_pts, 1);
    remaining = 1:n_cand;
    iter = 0;
    while ~all(covered) && iter < max_iter && ~isempty(remaining)
        iter = iter + 1;
        if mean(covered) >= tau, break; end
        counts = zeros(length(remaining), 1);
        for i = 1:length(remaining)
            counts(i) = sum(cov_mat(:, remaining(i)) & ~covered);
        end
        [mx, bi] = max(counts);
        if mx == 0, break; end
        gi = remaining(bi);
        sel_vp = [sel_vp; cand_pts(gi, :)]; %#ok<AGROW>
        sel_nrm = [sel_nrm; cand_nrm(gi, :)]; %#ok<AGROW>
        covered = covered | cov_mat(:, gi);
        remaining(bi) = [];
    end
end

function path_order = solveTSP(dist_matrix)
    n = size(dist_matrix, 1);
    if n <= 1, path_order = 1:n; return; end
    visited = false(1, n); path_order = zeros(1, n);
    curr = 1; visited(1) = true; path_order(1) = 1;
    for i = 2:n
        unv = find(~visited);
        [~, idx] = min(dist_matrix(curr, unv));
        path_order(i) = unv(idx); visited(unv(idx)) = true; curr = unv(idx);
    end
end

function cost = calculatePathCost(path, dist_matrix)
    n = length(path); cost = 0;
    for i = 1:(n-1), cost = cost + dist_matrix(path(i), path(i+1)); end
    cost = cost + dist_matrix(path(end), path(1));
end

function turn = total_turn_angle(path)
    n = size(path, 1);
    turn = 0;
    if n < 3, return; end
    seg = diff(path);
    seglen = vecnorm(seg, 2, 2);
    dirs = seg ./ max(seglen, eps);
    for i = 2:size(dirs, 1)
        turn = turn + acosd(max(-1, min(1, dot(dirs(i-1,:), dirs(i,:)))));
    end
end
