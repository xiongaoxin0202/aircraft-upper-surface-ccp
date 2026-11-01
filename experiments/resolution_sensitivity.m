%% resolution_sensitivity.m（Code2 版）
% 任务4：采样分辨率敏感性分析（对应审稿意见 R1-8：测量不确定性未量化）
% 基线：Code2 目录（FOV + Z-Buffer 模型，参数与手稿一致）
% ------------------------------------------------------------
% 在 A350 上表面点云上，以不同体素下采样尺寸运行精简覆盖流程（τ=95%），
% 报告视点数、覆盖率、路径长度、规划时间随分辨率的变化率，量化测量不确定性。
%
% 使用方法：
%   1) 将本脚本放入 Code2/ 目录；
%   2) 检查输入：默认使用 seg_all/UP-AIR.pcd（当前测试集）。
%      若需 0.05 m 档（高于当前测试集密度），请先用原始点云
%      生成更密的上表面点云，并修改 pc_file 指向该文件；
%   3) MATLAB 中运行（或 matlab -batch "resolution_sensitivity"）。
%   运行时长：每档 3-10 分钟。
% ------------------------------------------------------------

clc; clear; close all;

%% === 配置 ===
pc_file = 'seg_all/UP-AIR.pcd';   % 上表面点云（基准密度约 0.1 m 体素）
voxel_sizes = [0.1, 0.2, 0.4];    % 体素尺寸档位（米）。若 pc_file 为更密数据，
                                  % 可改为 [0.05, 0.1, 0.2]
% 通用参数（与 Code2 MAIN_viewpoint.m 一致）
fov_h = 75; fov_v = 65;
near_plane = 0.5; far_plane = 5.46;
min_safe_distance = 3.5;
coverage_threshold = 0.95;
max_candidates = 3000;

if ~exist(pc_file, 'file')
    error('未找到 %s，请检查路径', pc_file);
end

pc0 = pcread(pc_file);

rows = cell(0, 7);
for vi = 1:length(voxel_sizes)
    vs = voxel_sizes(vi);
    fprintf('\n========== 体素尺寸: %.2f m ==========\n', vs);

    % 降采样（基准档若文件已是该密度则直接使用）
    if vi == 1 && vs <= 0.1
        pc = pc0;
    else
        pc = pcdownsample(pc0, 'gridAverage', vs);
    end
    target_pts = pc.Location;
    fprintf('点数: %d\n', size(target_pts, 1));

    % 法线 + 视点外推（朝外重定向）
    nrm = pcnormals(pc);
    centroid = mean(target_pts, 1);
    flip = dot(nrm, target_pts - centroid, 2) < 0;
    nrm(flip, :) = -nrm(flip, :);
    nrm = nrm ./ vecnorm(nrm, 2, 2);

    cand_vp = target_pts + nrm * min_safe_distance;
    n_all = size(cand_vp, 1);
    if n_all > max_candidates
        rng(2);
        sel = randperm(n_all, max_candidates);
        cand_vp = cand_vp(sel, :); cand_nrm = nrm(sel, :);
    else
        cand_nrm = nrm;
    end

    % 距离过滤
    kd_tree = KDTreeSearcher(target_pts);
    [~, dist] = knnsearch(kd_tree, cand_vp);
    keep = dist >= min_safe_distance;
    cand_vp = cand_vp(keep, :); cand_nrm = cand_nrm(keep, :);
    fprintf('候选视点: %d\n', size(cand_vp, 1));

    % 覆盖矩阵（FOV + Z-Buffer）+ 贪心
    n_pts = size(target_pts, 1);
    n_cand = size(cand_vp, 1);
    cov_mat = false(n_pts, n_cand);
    tic;
    for i = 1:n_cand
        cov_mat(:, i) = computeCoverageWithOcclusion(cand_vp(i,:), -cand_nrm(i,:), ...
            target_pts, near_plane, far_plane, fov_h, fov_v, [], []);
    end
    t_cov = toc;

    tic;
    [sel_vp, ~, covered, ~] = greedy_select(cov_mat, cand_vp, cand_nrm, coverage_threshold, 500);
    t_greedy = toc;
    cov_pct = mean(covered) * 100;
    fprintf('视点: %d, 覆盖率: %.2f%%, 耗时: %.1f s (覆盖矩阵 %.1f s + 贪心 %.1f s)\n', ...
        size(sel_vp, 1), cov_pct, t_cov+t_greedy, t_cov, t_greedy);

    % 路径长度（NN）
    path_len = NaN;
    if size(sel_vp, 1) >= 3
        dist_matrix = pdist2(sel_vp, sel_vp);
        order = solveTSP(dist_matrix);
        path_len = calculatePathCost(order, dist_matrix);
        fprintf('路径长度: %.1f m\n', path_len);
    end

    rows(end+1,:) = {vs, size(target_pts,1), size(cand_vp,1), size(sel_vp,1), cov_pct, path_len, t_cov+t_greedy}; %#ok<SAGROW>
end

%% === 输出汇总与变化率 ===
R = cell2table(rows, 'VariableNames', ...
    {'voxel_size_m','num_points','num_candidates','num_viewpoints','coverage_pct','path_length_m','planning_time_s'});
writetable(R, 'resolution_sensitivity_results.csv');
fprintf('\n分辨率敏感性汇总已保存: resolution_sensitivity_results.csv\n');
disp(R);

if size(R, 1) > 1
    fprintf('\n相对基准(%.2f m)的变化率:\n', R.voxel_size_m(1));
    for i = 2:size(R, 1)
        fprintf('  %.2f m: 视点数 %+.1f%%, 覆盖率 %+.2f pp, 路径 %+.1f%%, 时间 %+.1f%%\n', ...
            R.voxel_size_m(i), ...
            100*(R.num_viewpoints(i)/R.num_viewpoints(1)-1), ...
            R.coverage_pct(i)-R.coverage_pct(1), ...
            100*(R.path_length_m(i)/R.path_length_m(1)-1), ...
            100*(R.planning_time_s(i)/R.planning_time_s(1)-1));
    end
end

fprintf('\n任务4 完成。\n');

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
