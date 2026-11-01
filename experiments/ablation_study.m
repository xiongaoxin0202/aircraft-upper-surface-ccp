%% ablation_study.m（Code2 版）
% 任务2：消融实验（对应审稿意见 R1-7：无消融实验）
% 基线：Code2 目录（生成手稿 Table 3 数据的代码，FOV + Z-Buffer 模型）
% ------------------------------------------------------------
% 5 项消融（每项输出对比表，全部写入 ablation_results.csv）：
%   A1 结构引导分割 vs 全局法线        -> 法线翻转率、无效视点比例
%   A2 全局距离场过滤 开/关            -> 视点数、覆盖率、安全违规视点数
%   A3 覆盖率阈值截断 τ=95% vs τ=100%  -> 视点数、路径长度、覆盖率
%   A4 轨迹平滑 折线 vs 样条           -> 路径长度、转弯角、曲率违例数
%   A5 TSP 排序 NN vs 随机顺序         -> 路径长度、求解时间
%
% 使用方法：
%   1) 将本脚本放入 Code2/ 目录；
%   2) 确认存在 seg_all/ 与 coverage_threshold_results/；
%   3) MATLAB 中运行（或 matlab -batch "ablation_study"）。
%
% 注意：核心覆盖函数 computeCoverageWithOcclusion 与 Code2 的
%       MAIN_viewpoint.m 完全一致（FOV 视锥 + Z-Buffer 遮挡剔除）。
% ------------------------------------------------------------

clc; clear; close all;

%% === 通用参数（与 Code2 MAIN_viewpoint.m 一致）===
fov_h = 75;             % 水平视场角（度）
fov_v = 65;             % 垂直视场角（度）
near_plane = 0.5;       % 最近探测距离（米）
far_plane = 5.46;       % 最远探测距离（米）
min_safe_distance = 3.5;% 最小安全距离（米）
coverage_threshold = 0.95;
min_turn_radius = 2;    % 假设最小转弯半径（米），按实际无人机修改
max_candidates = 3000;  % 候选视点上限（控制计算量）

results = cell(0, 5);

%% ============ A1: 结构引导分割 vs 全局法线 ============
fprintf('\n========== A1: 结构引导分割 vs 全局法线 ==========\n');
try
    pc_up = pcread('seg_all/UP-AIR.pcd');
    pts = pc_up.Location;

    % 全局法线估计（KNN，k=20）
    global_normals = pcnormals(pc_up);

    % 分区法线：加载 7 个分区，分别估计 + 朝外重定向，合并
    part_normals = zeros(size(pts));
    part_assigned = false(size(pts,1), 1);
    kd = KDTreeSearcher(pts);
    for k = 1:7
        fname = sprintf('seg_all/%d.pcd', k);
        if ~exist(fname, 'file'), continue; end
        pc_part = pcread(fname);
        nrm = pcnormals(pc_part);
        c = mean(pc_part.Location, 1);
        dir_vec = pc_part.Location - c;
        flip = dot(nrm, dir_vec, 2) < 0;
        nrm(flip, :) = -nrm(flip, :);
        idx = knnsearch(kd, pc_part.Location);
        part_normals(idx, :) = nrm;
        part_assigned(idx) = true;
    end
    if ~any(part_assigned), error('A1: 未找到分区文件'); end
    part_normals(~part_assigned, :) = global_normals(~part_assigned, :);
    part_normals = part_normals ./ vecnorm(part_normals, 2, 2);
    global_normals = global_normals ./ vecnorm(global_normals, 2, 2);

    % 法线翻转率
    dotprod = sum(global_normals .* part_normals, 2);
    flip_ratio = mean(dotprod < 0) * 100;

    % 无效视点比例：沿法线外推 d_safe 后侵入机体（到点云距离 < d_safe/2）
    d_safe = min_safe_distance;
    vp_global = pts + global_normals * d_safe;
    vp_part   = pts + part_normals * d_safe;
    kd_tree = KDTreeSearcher(pts);
    [~, dist_g] = knnsearch(kd_tree, vp_global);
    [~, dist_p] = knnsearch(kd_tree, vp_part);
    invalid_g = mean(dist_g < d_safe/2) * 100;
    invalid_p = mean(dist_p < d_safe/2) * 100;

    fprintf('法线翻转率: 全局 %.2f%% | 无效视点比例: 全局外推 %.2f%% vs 分区外推 %.2f%%\n', ...
            flip_ratio, invalid_g, invalid_p);
    results(end+1,:) = {'A1','normal_flip_ratio_pct','global',flip_ratio,NaN}; %#ok<SAGROW>
    results(end+1,:) = {'A1','invalid_viewpoint_ratio_pct','global',invalid_g,NaN}; %#ok<SAGROW>
    results(end+1,:) = {'A1','invalid_viewpoint_ratio_pct','partitioned',invalid_p,NaN}; %#ok<SAGROW>
catch ME
    fprintf('A1 失败: %s\n', ME.message);
end

%% ============ A1b: 完整机身模型上的法线岐义（原始 PCA 法线） ============
% 说明：原始 PCA 最小特征向量的符号任意（不做任何方向一致化），
% 与分区重定向参考法线对比，展示全局法线估计的正常岐义。
fprintf('\n========== A1b: 完整机身模型 原始 PCA 法线岐义 ==========\n');
try
    if exist('model.ply', 'file')
        pc_full = pcread('model.ply');
        pc_full = pcdownsample(pc_full, 'gridAverage', 1.5);  % 控制规模（约 5-6 万点）
        pts_full = pc_full.Location;
        fprintf('完整模型点数: %d\n', size(pts_full, 1));

        % 原始 PCA 法线（无一致化：每点 KNN 协方差最小特征向量）
        k = 20;
        kd_full = KDTreeSearcher(pts_full);
        [idx, ~] = knnsearch(kd_full, pts_full, 'K', k+1);
        idx = idx(:, 2:end);
        n_pts = size(pts_full, 1);
        nrm_pca = zeros(n_pts, 3);
        for i = 1:n_pts
            nb = pts_full(idx(i,:), :);
            C = cov(nb);
            [V, ~] = eig(C);
            nrm_pca(i,:) = V(:,1)';
        end
        nrm_pca = nrm_pca ./ vecnorm(nrm_pca, 2, 2);

        % 参考方向：质心朝外（近似分区重定向）
        centroid = mean(pts_full, 1);
        ref_dir = pts_full - centroid;
        ref_dir = ref_dir ./ vecnorm(ref_dir, 2, 2);
        flip_full = mean(sum(nrm_pca .* ref_dir, 2) < 0) * 100;
        fprintf('完整模型原始 PCA 法线翻转率: %.2f%%\n', flip_full);
        results(end+1,:) = {'A1','normal_flip_ratio_pct_full_model','raw_PCA',flip_full,NaN};
    else
        fprintf('未找到 model.ply，跳过 A1b\n');
    end
catch ME
    fprintf('A1b 失败: %s\n', ME.message);
end

%% ============ A2:%% ============ A2: 全局距离场过滤 开/关 ============
fprintf('\n========== A2: 全局距离场过滤 ==========\n');
try
    pn = pcread('seg_all/view-normals.ply');
    pa = pcread('seg_all/UP-AIR.pcd');
    target_points = pa.Location;
    kd_tree = KDTreeSearcher(target_points);

    for filt = [true, false]
        cand_pts = pn.Location; cand_nrm = pn.Normal;
        if filt
            [~, dist] = knnsearch(kd_tree, cand_pts);
            keep = dist >= min_safe_distance;
            cand_pts = cand_pts(keep, :); cand_nrm = cand_nrm(keep, :);
        end
        % 候选视点抽样（控制计算量）
        n_all = size(cand_pts, 1);
        if n_all > max_candidates
            rng(3);
            sel = randperm(n_all, max_candidates);
            cand_pts = cand_pts(sel, :); cand_nrm = cand_nrm(sel, :);
        end
        n_cand = size(cand_pts, 1);

        % 覆盖矩阵（FOV + Z-Buffer，与 Code2 一致）
        n_pts = size(target_points, 1);
        cov_mat = false(n_pts, n_cand);
        tic;
        for i = 1:n_cand
            cov_mat(:, i) = computeCoverageWithOcclusion(cand_pts(i,:), -cand_nrm(i,:), ...
                target_points, near_plane, far_plane, fov_h, fov_v, [], []);
        end
        t_cov = toc;

        % 贪心选择（τ=95%）
        [sel_vp, ~, covered, ~] = greedy_select(cov_mat, cand_pts, cand_nrm, coverage_threshold, 500);

        % 安全违规：选中视点中距离 < min_safe_distance 的数量
        [~, sel_dist] = knnsearch(kd_tree, sel_vp);
        n_violation = sum(sel_dist < min_safe_distance);

        fprintf('过滤%s: 候选 %d -> 视点 %d, 覆盖率 %.2f%%, 安全违规 %d, 覆盖矩阵 %.1fs\n', ...
            string(filt), n_cand, size(sel_vp,1), mean(covered)*100, n_violation, t_cov);
        results(end+1,:) = {'A2',sprintf('filter_%s_num_viewpoints', string(filt)), 'proposed', size(sel_vp,1), NaN}; %#ok<SAGROW>
        results(end+1,:) = {'A2',sprintf('filter_%s_coverage_pct', string(filt)), 'proposed', mean(covered)*100, NaN}; %#ok<SAGROW>
        results(end+1,:) = {'A2',sprintf('filter_%s_safety_violations', string(filt)), 'proposed', n_violation, NaN}; %#ok<SAGROW>
    end
catch ME
    fprintf('A2 失败: %s\n', ME.message);
end

%% ============ A3: 覆盖率阈值截断（95% vs 100%） ============
fprintf('\n========== A3: 覆盖率阈值截断 ==========\n');
for tau = [0.95, 1.00]
    folder = fullfile('coverage_threshold_results', sprintf('coverage_%.2f', tau));
    if exist(fullfile(folder, 'summary.txt'), 'file')
        s_txt = fileread(fullfile(folder, 'summary.txt'));
        n_vp = regexp(s_txt, '最终视点数量:\s*(\d+)', 'tokens');
        cov  = regexp(s_txt, '总覆盖率:\s*([0-9.]+)%', 'tokens');
        fprintf('τ=%.2f (已有结果): 视点 %s, 覆盖率 %s%%\n', tau, n_vp{1}{1}, cov{1}{1});
        results(end+1,:) = {'A3',sprintf('tau_%d_num_viewpoints', round(tau*100)), 'proposed', str2double(n_vp{1}{1}), NaN}; %#ok<SAGROW>
        results(end+1,:) = {'A3',sprintf('tau_%d_coverage_pct', round(tau*100)), 'proposed', str2double(cov{1}{1}), NaN}; %#ok<SAGROW>
    else
        fprintf('τ=%.2f: 结果文件夹不存在，跳过（请先运行 MAIN_viewpoint.m）\n', tau);
    end
end

%% ============ A4 & A5: 平滑与 TSP（共用视点数据） ============
fprintf('\n========== A4: 轨迹平滑 & A5: TSP 排序 ==========\n');
try
    folder = fullfile('coverage_threshold_results', 'coverage_0.95');
    vp_data = load(fullfile(folder, 'optimal_viewpoints.mat'));
    viewpoints = vp_data.safe_selected_viewpoints;

    dist_matrix = pdist2(viewpoints, viewpoints);

    % --- A5: NN vs 随机顺序 ---
    tic;
    order_nn = solveTSP(dist_matrix);
    t_nn = toc;
    rng(7);
    order_rnd = randperm(size(viewpoints, 1));
    len_nn = calculatePathCost(order_nn, dist_matrix);
    len_rnd = calculatePathCost(order_rnd, dist_matrix);
    fprintf('A5: NN 路径 %.1f m (%.4fs) vs 随机顺序 %.1f m, 改善 %.2f%%\n', ...
        len_nn, t_nn, len_rnd, (1-len_nn/len_rnd)*100);
    results(end+1,:) = {'A5','path_length_m','NN',len_nn,t_nn}; %#ok<SAGROW>
    results(end+1,:) = {'A5','path_length_m','random_order',len_rnd,NaN}; %#ok<SAGROW>

    % --- A4: 折线 vs 样条平滑（用 NN 顺序） ---
    ordered = viewpoints(order_nn, :);
    polyline = ordered;
    smooth_path = spline_smooth_path(ordered, 0.1, 500);

    [len_poly, turn_poly, curv_viol_poly] = path_metrics(polyline, min_turn_radius);
    [len_sm, turn_sm, curv_viol_sm] = path_metrics(smooth_path, min_turn_radius);
    fprintf('A4: 折线 长度 %.1f m, 转弯角 %.1f°, 曲率违例 %d | 样条 长度 %.1f m, 转弯角 %.1f°, 曲率违例 %d\n', ...
        len_poly, turn_poly, curv_viol_poly, len_sm, turn_sm, curv_viol_sm);
    results(end+1,:) = {'A4','path_length_m','polyline',len_poly,NaN}; %#ok<SAGROW>
    results(end+1,:) = {'A4','path_length_m','spline',len_sm,NaN}; %#ok<SAGROW>
    results(end+1,:) = {'A4','turn_angle_deg','polyline',turn_poly,NaN}; %#ok<SAGROW>
    results(end+1,:) = {'A4','turn_angle_deg','spline',turn_sm,NaN}; %#ok<SAGROW>
    results(end+1,:) = {'A4','curvature_violations','polyline',curv_viol_poly,NaN}; %#ok<SAGROW>
    results(end+1,:) = {'A4','curvature_violations','spline',curv_viol_sm,NaN}; %#ok<SAGROW>
catch ME
    fprintf('A4/A5 失败: %s\n', ME.message);
end

%% === 保存消融结果 ===
if ~isempty(results)
    R = cell2table(results, 'VariableNames', {'ablation','metric','variant','value','extra'});
    writetable(R, 'ablation_results.csv');
    fprintf('\n消融结果表已保存: ablation_results.csv\n');
    disp(R);
end

fprintf('\n任务2 完成。\n');

%% ================== 局部函数（与 Code2 一致） ==================

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

function coverage = computeCoverageWithOcclusion(viewpoint, direction, points, near, far, fov_h, fov_v, ~, ~)
    % FOV 视锥剔除 + Z-Buffer 深度缓冲遮挡剔除（与 Code2 MAIN_viewpoint.m 一致）
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

function path_order = solveTSP(dist_matrix)
    % 最近邻 TSP（与 Code2 MAIN_path_one.m 一致）
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

function smooth_path = spline_smooth_path(waypoints, smoothness, max_points)
    % 与 Code2 MAIN_path_one.m 的样条插值 + 移动平均一致
    if size(waypoints, 1) < 2, smooth_path = waypoints; return; end
    t = 1:size(waypoints, 1);
    tt = linspace(1, size(waypoints, 1), max_points);
    smooth_path = zeros(length(tt), 3);
    for dim = 1:3
        if size(waypoints, 1) < 4
            smooth_path(:, dim) = interp1(t, waypoints(:, dim), tt, 'linear');
        else
            smooth_path(:, dim) = interp1(t, waypoints(:, dim), tt, 'spline');
        end
    end
    if smoothness > 0 && size(smooth_path, 1) > 10
        for dim = 1:3
            smooth_path(:, dim) = smooth(smooth_path(:, dim), round(smoothness * 10));
        end
    end
end

function [len, turn_deg, curv_viol] = path_metrics(path, min_turn_radius)
    len = 0; turn_deg = 0; curv_viol = 0;
    n = size(path, 1);
    if n < 3, len = sum(vecnorm(diff(path), 2, 2)); return; end
    seg = diff(path);
    seglen = vecnorm(seg, 2, 2);
    len = sum(seglen);
    dirs = seg ./ max(seglen, eps);
    for i = 2:size(dirs, 1)
        ang = acosd(max(-1, min(1, dot(dirs(i-1,:), dirs(i,:)))));
        turn_deg = turn_deg + ang;
        kappa = 2 * sind(ang/2) / max(max(seglen(i-1), seglen(i)), eps);
        if kappa > 1 / min_turn_radius, curv_viol = curv_viol + 1; end
    end
end
