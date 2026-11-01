%% 最优覆盖视点生成 — 完整版（Z-Buffer遮挡剔除 + 多阈值 + 插图显示）
% 整合 MAIN_viewpoint.m + MAIN_viewpoint2.m + MAIN_viewpoint_one.m
% 相机配置与 Main_duo.m 一致：FOV-based 视锥剔除 + Z-Buffer 深度缓冲遮挡剔除
% 功能：单阈值/多阈值视点选择 + 高质量插图显示（不保存文件）
clc; clear; close all;
tic

%% ==================== 0. 运行模式与全局配置 ====================
% run_mode 选项:
%   'single' - 单一覆盖率阈值，交互式显示 + 显示插图
%   'multi'  - 多覆盖率阈值循环(0.80:0.01:1.00)，批量保存结果，自动显示收敛图
%   'plot'   - 仅加载已有结果并显示插图（需指定 results_folder）
run_mode = 'single';         % <--- 在此切换模式

% 单一阈值模式的目标覆盖率
target_coverage = 0.95;

% plot模式：已有结果的文件夹路径
results_folder = '';

% 强制重新计算（忽略已有缓存结果true）false
force_recompute = true;

%% ==================== 1. 传感器参数（与 Main_duo.m 完全一致）====================
fov_h = 75;          % 水平视场角（度）90  75
fov_v = 65;          % 垂直视场角（度）60  65
near_plane = 0.5;     % 最近探测距离（米）1
far_plane = 5.46;      % 最远探测距离（米）6
min_safe_distance = 3.5;          % 最小安全距离（米）

% 遮挡剔除参数（Z-Buffer自适应分辨率，以下参数仅保留兼容）
occlusion_angle_threshold = 5;
neighbor_search_radius = 1.0;

% 并行计算设置（需要 Parallel Computing Toolbox，可大幅加速覆盖矩阵预计算）
use_parallel = false;  % 设为 true 启用 parfor 加速

% 贪婪选择参数
max_selected_viewpoints = 500;
max_greedy_iterations = 500;

% 数据路径
pn_file = 'E:/XAX_ws/飞机上表面扫掠 -- 第三篇-第四篇-第五篇/飞机上表面的覆盖检查路径规划研究 -- 论文 -- 第三篇/论文投稿/英文SCI投稿/Robotica -- 投稿  ROB-2026-0389/重投材料/aircraft-upper-surface-ccp/data/B757_segmented/view-normals.ply';
pa_file = 'E:/XAX_ws/飞机上表面扫掠 -- 第三篇-第四篇-第五篇/飞机上表面的覆盖检查路径规划研究 -- 论文 -- 第三篇/论文投稿/英文SCI投稿/Robotica -- 投稿  ROB-2026-0389/重投材料/aircraft-upper-surface-ccp/data/B757_segmented/UP-AIR.pcd';

% 输出图片格式与分辨率（SCI期刊标准）
output_format = '-dpng';
output_resolution = '-r600';  % 600 DPI

% SCI期刊全局字体设置
set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultAxesFontSize', 20);
set(0, 'DefaultTextFontSize', 20);
set(0, 'DefaultLineLineWidth', 1.5);

%% ==================== 2. plot模式：直接加载已有结果 ====================
if strcmp(run_mode, 'plot')
    assert(exist(results_folder, 'dir') == 7, '请指定有效的 results_folder 路径。');
    if ~strcmp(results_folder(end), filesep)
        results_folder = [results_folder, filesep];
    end
    fprintf('加载已有结果: %s\n', results_folder);
    load([results_folder, 'target_points.mat']);
    load([results_folder, 'optimal_viewpoints.mat']);
    load([results_folder, 'coverage_info.mat']);
    if exist([results_folder, 'convergence_history.mat'], 'file')
        load([results_folder, 'convergence_history.mat']);
        has_history = true;
    else
        has_history = false;
        iteration_history = struct();
    end
    fprintf('视点数量: %d，覆盖率: %.2f%%\n', size(safe_selected_viewpoints,1), coverage_percentage);
    generateSCIFigures(target_points, safe_selected_viewpoints, selected_normals, ...
        covered_points, coverage_percentage, iteration_history, has_history, results_folder, ...
        output_format, output_resolution, target_coverage);
    fprintf('插图已显示完毕。\n');
    toc; return;
end

%% ==================== 3. 加载点云数据并过滤危险视点 ====================
fprintf('加载点云数据并过滤危险视点...\n');
try
    pn = pcread(pn_file);
    pa = pcread(pa_file);
catch
    error('请检查 PLY/PCD 文件路径是否正确。');
end
target_points = pa.Location;
num_target = size(target_points, 1);
fprintf('目标点云点数: %d\n', num_target);

% 构建KD树用于快速安全距离检查
kd_tree = createns(target_points, 'NSMethod', 'kdtree');

% 高效安全过滤（使用KD树）
fprintf('过滤距离目标点云过近的候选视点...\n');
num_candidates_initial = size(pn.Location, 1);
valid_idx = true(num_candidates_initial, 1);
for i = 1:num_candidates_initial
    [~, dist] = knnsearch(kd_tree, pn.Location(i, :));
    if dist < min_safe_distance
        valid_idx(i) = false;
    end
end
cand_pts = pn.Location(valid_idx, :);
cand_normals = pn.Normal(valid_idx, :);
num_cand = size(cand_pts, 1);
fprintf('过滤后候选视点数量: %d (原始: %d)\n', num_cand, num_candidates_initial);

%% ==================== 4. 预计算候选视点覆盖矩阵（FOV + Z-Buffer 遮挡剔除）====================
fprintf('预计算候选视点覆盖矩阵（视场角 + Z-Buffer遮挡剔除）...\n');
coverage_matrix = false(num_target, num_cand);
view_dirs_all = -cand_normals;  % 预提取方向向量（parfor 需要连续内存访问）

if use_parallel
    fprintf('  使用并行计算 (parfor)，启动工作进程...\n');
    parfor i = 1:num_cand
        coverage_matrix(:, i) = computeCoverageWithOcclusion(...
            cand_pts(i, :), view_dirs_all(i, :), target_points, ...
            near_plane, far_plane, fov_h, fov_v, ...
            occlusion_angle_threshold, neighbor_search_radius);
    end
else
    for i = 1:num_cand
        if mod(i, 300) == 0
            fprintf('  进度: %d/%d (%.1f%%)\n', i, num_cand, 100*i/num_cand);
        end
        coverage_matrix(:, i) = computeCoverageWithOcclusion(...
            cand_pts(i, :), view_dirs_all(i, :), target_points, ...
            near_plane, far_plane, fov_h, fov_v, ...
            occlusion_angle_threshold, neighbor_search_radius);
    end
end
fprintf('预计算完成。\n');

%% ==================== 5. 确定覆盖率阈值列表 ====================
if strcmp(run_mode, 'single')
    coverage_thresholds = target_coverage;
else  % 'multi'
    coverage_thresholds = 0.80:0.01:1.00;
end
num_thresholds = length(coverage_thresholds);

%% ==================== 6. 创建结果输出目录 ====================
main_result_folder = 'coverage_threshold_results';
if ~exist(main_result_folder, 'dir')
    mkdir(main_result_folder);
end

%% ==================== 7. 按覆盖率阈值循环选择视点 ====================
for t_idx = 1:num_thresholds
    cov_threshold = coverage_thresholds(t_idx);
    fprintf('\n========================================\n');
    fprintf('处理覆盖率阈值: %.2f (%d/%d)\n', cov_threshold, t_idx, num_thresholds);
    fprintf('========================================\n');
    tic_threshold = tic;

    % 子文件夹
    subfolder_name = sprintf('coverage_%.2f', cov_threshold);
    result_folder = fullfile(main_result_folder, subfolder_name);
    if ~exist(result_folder, 'dir')
        mkdir(result_folder);
    end

    % 尝试加载已有结果
    optimal_file = fullfile(result_folder, 'optimal_viewpoints.mat');
    if exist(optimal_file, 'file') && ~force_recompute
        fprintf('检测到已有结果，直接加载...\n');
        load(fullfile(result_folder, 'target_points.mat'));
        load(fullfile(result_folder, 'optimal_viewpoints.mat'));
        load(fullfile(result_folder, 'coverage_info.mat'));
        if exist(fullfile(result_folder, 'convergence_history.mat'), 'file')
            load(fullfile(result_folder, 'convergence_history.mat'));
            has_history = true;
        else
            has_history = false;
            iteration_history = struct();
        end
    else
        % ---------- 贪婪选择视点 ----------
        fprintf('使用贪婪算法选择视点...\n');
        selected_viewpoints = [];
        selected_normals_list = [];
        covered_points = false(num_target, 1);
        remaining = 1:num_cand;

        % 收敛历史记录
        iter_records = [];
        coverage_records = [];
        new_cover_records = [];

        for iter = 1:max_greedy_iterations
            if mean(covered_points) >= cov_threshold
                fprintf('已达到覆盖率阈值 %.1f%%，停止选择\n', cov_threshold * 100);
                break;
            end
            if isempty(remaining), break; end

            % 计算每个剩余视点的新增覆盖点数
            new_counts = zeros(length(remaining), 1);
            if use_parallel && length(remaining) > 200
                parfor k = 1:length(remaining)
                    idx = remaining(k);
                    new_counts(k) = sum(coverage_matrix(:, idx) & ~covered_points);
                end
            else
                for k = 1:length(remaining)
                    idx = remaining(k);
                    new_counts(k) = sum(coverage_matrix(:, idx) & ~covered_points);
                end
            end

            [max_new, best_k] = max(new_counts);
            if max_new == 0
                fprintf('没有视点能提供新的覆盖，停止选择\n');
                break;
            end

            best_idx = remaining(best_k);
            selected_viewpoints = [selected_viewpoints; cand_pts(best_idx, :)];
            selected_normals_list = [selected_normals_list; -cand_normals(best_idx, :)];
            newly_covered = sum(coverage_matrix(:, best_idx) & ~covered_points);
            covered_points = covered_points | coverage_matrix(:, best_idx);
            remaining(best_k) = [];

            % 记录迭代数据
            iter_records = [iter_records; iter];
            coverage_records = [coverage_records; 100 * mean(covered_points)];
            new_cover_records = [new_cover_records; newly_covered];

            if mod(iter, 1) == 0 || iter == 1
                fprintf('迭代 %d: 视点 %d, 新增覆盖: %d, 累计覆盖率: %.2f%%\n', ...
                    iter, best_idx, newly_covered, 100 * mean(covered_points));
            end
        end

        % 限制最大视点数量
        if size(selected_viewpoints, 1) > max_selected_viewpoints
            selected_viewpoints = selected_viewpoints(1:max_selected_viewpoints, :);
            selected_normals_list = selected_normals_list(1:max_selected_viewpoints, :);
        end

        % ---------- 安全距离调整 ----------
        fprintf('应用安全距离策略...\n');
        safe_selected_viewpoints = selected_viewpoints;
        for i = 1:size(safe_selected_viewpoints, 1)
            vp = safe_selected_viewpoints(i, :);
            [~, dist] = knnsearch(kd_tree, vp);
            if dist < min_safe_distance + 0.5
                safe_selected_viewpoints(i, :) = pushViewpointToSafety(vp, target_points, kd_tree, min_safe_distance + 0.5);
            end
        end
        selected_normals = selected_normals_list;
        coverage_percentage = 100 * mean(covered_points);
        has_history = true;
        iteration_history = struct('iter', iter_records, 'coverage', coverage_records, 'new_covered', new_cover_records);

        % ---------- 保存结果 ----------
        fprintf('保存结果到: %s\n', result_folder);
        save(fullfile(result_folder, 'target_points.mat'), 'target_points');
        save(fullfile(result_folder, 'optimal_viewpoints.mat'), 'safe_selected_viewpoints', 'selected_normals', 'covered_points');
        save(fullfile(result_folder, 'coverage_info.mat'), 'coverage_percentage', 'covered_points');
        save(fullfile(result_folder, 'convergence_history.mat'), 'iteration_history');

        params = struct('fov_h', fov_h, 'fov_v', fov_v, ...
            'near_plane', near_plane, 'far_plane', far_plane, ...
            'min_safe_distance', min_safe_distance, ...
            'coverage_threshold', cov_threshold);
        save(fullfile(result_folder, 'parameters.mat'), 'params');

        % 文本摘要
        fid = fopen(fullfile(result_folder, 'summary.txt'), 'w');
        fprintf(fid, '覆盖率阈值: %.2f\n', cov_threshold);
        fprintf(fid, '最终视点数量: %d\n', size(safe_selected_viewpoints, 1));
        fprintf(fid, '总覆盖率: %.2f%%\n', coverage_percentage);
        fprintf(fid, '处理时间: %.2f 秒\n', toc(tic_threshold));
        fclose(fid);
    end

    fprintf('阈值 %.2f 处理完成，耗时: %.2f 秒\n', cov_threshold, toc(tic_threshold));

    % ---------- 8. 生成并显示插图 ----------
    fprintf('生成并显示插图...\n');
    generateSCIFigures(target_points, safe_selected_viewpoints, selected_normals, ...
        covered_points, coverage_percentage, iteration_history, has_history, result_folder, ...
        output_format, output_resolution, cov_threshold);
    fprintf('插图已显示。\n');

    % ---------- 9. 单阈值模式：交互式显示 ----------
    if strcmp(run_mode, 'single')
        fprintf('显示交互式视点分布图...\n');
        % 快速可视化（不保存，仅用于检查）
        figure('Position', [100, 100, 1000, 500], 'Color', 'w', 'Name', '最优覆盖视点集合');
        hold on; grid on; axis equal;
        set(gca, 'Color', 'w');
        covered_idx = find(covered_points);
        not_covered_idx = find(~covered_points);
        if ~isempty(covered_idx)
            scatter3(target_points(covered_idx,1), target_points(covered_idx,2), target_points(covered_idx,3), ...
                10, 'g', 'filled', 'MarkerFaceAlpha', 0.3);
        end
        if ~isempty(not_covered_idx)
            scatter3(target_points(not_covered_idx,1), target_points(not_covered_idx,2), target_points(not_covered_idx,3), ...
                10, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.3);
        end
        scatter3(safe_selected_viewpoints(:,1), safe_selected_viewpoints(:,2), safe_selected_viewpoints(:,3), ...
            60, 'b', 'filled', 'MarkerEdgeColor', 'k');
        arrow_len = 3;
        for i = 1:size(safe_selected_viewpoints, 1)
            vp = safe_selected_viewpoints(i, :);
            n = selected_normals(i, :);
            vd = n / norm(n);
            up = [0, 0, 1];
            if abs(dot(vd, up)) > 0.99, up = [0, 1, 0]; end
            rv = cross(up, vd); rv = rv / norm(rv);
            uv = cross(vd, rv);
            quiver3(vp(1), vp(2), vp(3), rv(1)*arrow_len, rv(2)*arrow_len, rv(3)*arrow_len, ...
                'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
            quiver3(vp(1), vp(2), vp(3), uv(1)*arrow_len, uv(2)*arrow_len, uv(3)*arrow_len, ...
                'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
            quiver3(vp(1), vp(2), vp(3), vd(1)*arrow_len, vd(2)*arrow_len, vd(3)*arrow_len, ...
                'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        end
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title(sprintf('最优覆盖视点集合 (%d视点, 覆盖率: %.2f%%)', ...
            size(safe_selected_viewpoints,1), coverage_percentage));
        legend('已覆盖点', '未覆盖点', '视点位置', 'Location', 'best');
        view(3); rotate3d on;
    end
end

fprintf('\n========================================\n');
fprintf('全部完成！\n');
fprintf('结果保存在: %s\n', main_result_folder);
fprintf('总耗时: %.2f 秒\n', toc);
fprintf('========================================\n');

%% ==================== 辅助函数：Z-Buffer遮挡剔除 ====================

function coverage = computeCoverageWithOcclusion(viewpoint, direction, points, near, far, ...
    fov_h, fov_v, ~, ~)
    % 计算视点覆盖（视锥剔除 + Z-Buffer深度缓冲遮挡剔除）
    % 与 Main_duo.m 完全一致的算法
    N = size(points, 1);
    coverage = false(N, 1);

    % ===== 阶段1: 视锥剔除 =====
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

    % ===== 阶段2: Z-Buffer 遮挡剔除 =====
    f_dists = sqrt(sum((points(frustum_idx, :) - viewpoint).^2, 2));
    f_dist_proj = dist_proj(frustum_idx);
    h_norm = h_comp(frustum_idx) ./ (f_dist_proj * tan_half_h + eps);
    v_norm = v_comp(frustum_idx) ./ (f_dist_proj * tan_half_v + eps);

    % 自适应网格分辨率（缩放后表面分辨率0.03m，对应原始0.3m）
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
    end
end

%% ==================== 辅助函数：安全距离调整 ====================

function safe_vp = pushViewpointToSafety(viewpoint, target_points, kd_tree, min_dist)
    [idx, dist] = knnsearch(kd_tree, viewpoint);
    nearest_pt = target_points(idx, :);
    if dist >= min_dist
        safe_vp = viewpoint;
        return;
    end
    push_dir = viewpoint - nearest_pt;
    if norm(push_dir) < 1e-6
        % 退化情况：视点与最近点重合，使用随机方向推出
        push_dir = rand(1, 3);
    end
    push_dir = push_dir / norm(push_dir);
    safe_vp = nearest_pt + push_dir * (min_dist + 1);
end

%% ==================== 辅助函数：SCI期刊插图生成 ====================

function generateSCIFigures(target_points, safe_viewpoints, selected_normals, ...
    covered_points, coverage_percentage, iteration_history, has_history, result_folder, ...
    output_format, output_resolution, cov_threshold)

    num_vp = size(safe_viewpoints, 1);

    %% ---- Figure A: 视点分布与覆盖三维渲染图 ----
    fprintf('  显示 Figure A: 视点分布与覆盖渲染图...\n');
    figA = figure('Position', [50, 50, 1000, 500], 'Color', 'w', 'Visible', 'on', ...
        'Name', 'Figure A: Viewpoint Distribution');
    hold on; grid on; axis equal;

    % 未覆盖点 — 灰色
    not_cov_idx = find(~covered_points);
    if ~isempty(not_cov_idx)
        scatter3(target_points(not_cov_idx,1), target_points(not_cov_idx,2), target_points(not_cov_idx,3), ...
            4, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.3);
    end

    % 已覆盖点 — 亮绿色
    cov_idx = find(covered_points);
    if ~isempty(cov_idx)
        scatter3(target_points(cov_idx,1), target_points(cov_idx,2), target_points(cov_idx,3), ...
            8, [0.0 0.9 0.1], 'filled', 'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.7);
    end

    % 绘制视点位置及RGB坐标轴
    arrow_len = 3;
    for i = 1:num_vp
        vp = safe_viewpoints(i, :);
        n = selected_normals(i, :);
        scatter3(vp(1), vp(2), vp(3), 70, 'b', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1);
        vd = n / norm(n);
        up = [0, 0, 1];
        if abs(dot(vd, up)) > 0.99, up = [0, 1, 0]; end
        rv = cross(up, vd); rv = rv / norm(rv);
        uv = cross(vd, rv);
        quiver3(vp(1), vp(2), vp(3), rv(1)*arrow_len, rv(2)*arrow_len, rv(3)*arrow_len, ...
            'Color', 'r', 'LineWidth', 1.8, 'MaxHeadSize', 0.6);
        quiver3(vp(1), vp(2), vp(3), uv(1)*arrow_len, uv(2)*arrow_len, uv(3)*arrow_len, ...
            'Color', 'g', 'LineWidth', 1.8, 'MaxHeadSize', 0.6);
        quiver3(vp(1), vp(2), vp(3), vd(1)*arrow_len, vd(2)*arrow_len, vd(3)*arrow_len, ...
            'Color', 'b', 'LineWidth', 1.8, 'MaxHeadSize', 0.6);
    end

    set(gca, 'Color', 'w');
    xlabel('X (m)', 'FontSize', 20);
    ylabel('Y (m)', 'FontSize', 20);
    zlabel('Z (m)', 'FontSize', 20);
    title(sprintf('Aircraft Upper Surface Optimal Viewpoint Distribution\nCoverage: %.1f%% | Viewpoints: %d', ...
        coverage_percentage, num_vp), 'FontSize', 22, 'FontWeight', 'bold');
    view(45, 25);
    light('Position', [1 1 1], 'Style', 'infinite');
    lighting gouraud;
    material dull;

    % 图例
    h1 = scatter3(NaN, NaN, NaN, 20, [0.5 0.5 0.5], 'filled');
    h2 = scatter3(NaN, NaN, NaN, 20, [0.0 0.9 0.1], 'filled');
    h3 = scatter3(NaN, NaN, NaN, 70, 'b', 'filled', 'MarkerEdgeColor', 'k');
    h4 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'Color', 'r', 'LineWidth', 1.8);
    h5 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'Color', 'g', 'LineWidth', 1.8);
    h6 = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'Color', 'b', 'LineWidth', 1.8);
    legend([h1, h2, h3, h4, h5, h6], ...
        {'Uncovered area', 'Covered area', 'Optimal viewpoint', 'Right axis', 'Up axis', 'View direction'}, ...
        'Location', 'northeastoutside', 'FontSize', 20);

    % print(figA, fullfile(result_folder, 'Figure_A_Viewpoint_Distribution'), output_format, output_resolution);
    % close(figA);
    drawnow;  % 强制渲染显示

    %% ---- Figure B: 视点选择收敛过程 ----
    if has_history
        fprintf('  显示 Figure B: 视点选择收敛过程...\n');
        figB = figure('Position', [100, 100, 1400, 550], 'Color', 'w', 'Visible', 'on', ...
            'Name', 'Figure B: Convergence');

        subplot(1, 2, 1);
        set(gca, 'Color', 'w', 'LineWidth', 1.2, 'TickDir', 'out', 'Box', 'on');
        plot(iteration_history.iter, iteration_history.coverage, 'b-o', ...
            'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
        grid on;
        xlabel('Number of Viewpoints', 'FontSize', 20);
        ylabel('Cumulative Coverage (%)', 'FontSize', 20);
        title(sprintf('Coverage Convergence (threshold: %.2f)', cov_threshold), 'FontSize', 20, 'FontWeight', 'bold');
        xlim([1, max(iteration_history.iter)]);
        ylim([0, 105]);
        % 标注覆盖率阈值线
        hold on;
        yline(cov_threshold * 100, 'r--', 'LineWidth', 2);
        text(max(iteration_history.iter)*0.5, cov_threshold*100 + 2, ...
            sprintf('Target: %.0f%%', cov_threshold*100), 'Color', 'r', 'FontSize', 18);
        
        subplot(1, 2, 2);
        set(gca, 'Color', 'w', 'LineWidth', 1.2, 'TickDir', 'out', 'Box', 'on');
        bar(iteration_history.iter, iteration_history.new_covered, ...
            'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'none');
        grid on;
        xlabel('Number of Viewpoints', 'FontSize', 20);
        ylabel('Newly Covered Points', 'FontSize', 20);
        title('Marginal Coverage Gain per Viewpoint', 'FontSize', 20, 'FontWeight', 'bold');
        xlim([0.5, max(iteration_history.iter) + 0.5]);
        
        sgtitle('Greedy Viewpoint Selection Convergence Behavior', 'FontSize', 22, 'FontWeight', 'bold');
        % print(figB, fullfile(result_folder, 'Figure_B_Convergence'), output_format, output_resolution);
        % close(figB);
        drawnow;
    end

    %% ---- Figure C: 视点方向详细视图（前N个视点的RGB坐标轴）----
    fprintf('  显示 Figure C: 视点方向详细视图...\n');
    num_show = min(100, num_vp);
    figC = figure('Position', [100, 100, 1200, 600], 'Color', 'w', 'Visible', 'on', ...
        'Name', 'Figure C: Viewpoint Directions');
    hold on; grid on; axis equal;

    % 未覆盖点 — 灰色背景
    not_cov_idx = find(~covered_points);
    if ~isempty(not_cov_idx)
        scatter3(target_points(not_cov_idx,1), target_points(not_cov_idx,2), target_points(not_cov_idx,3), ...
            3, [0.5 0.5 0.5], 'filled', 'MarkerFaceAlpha', 0.15);
    end

    % 已覆盖点 — 亮绿色
    cov_idx = find(covered_points);
    if ~isempty(cov_idx)
        scatter3(target_points(cov_idx,1), target_points(cov_idx,2), target_points(cov_idx,3), ...
            6, [0.0 0.9 0.1], 'filled', 'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.4);
    end

    vp_colors = lines(num_show);
    long_arrow = 3;
    for i = 1:num_show
        vp = safe_viewpoints(i, :);
        n = selected_normals(i, :);
        scatter3(vp(1), vp(2), vp(3), 100, vp_colors(i,:), 'filled', ...
            'MarkerEdgeColor', 'k', 'LineWidth', 1.2);

        vd = n / norm(n);
        up = [0, 0, 1];
        if abs(dot(vd, up)) > 0.99, up = [0, 1, 0]; end
        rv = cross(up, vd); rv = rv / norm(rv);
        uv = cross(vd, rv);

        quiver3(vp(1), vp(2), vp(3), rv(1)*long_arrow, rv(2)*long_arrow, rv(3)*long_arrow, ...
            'Color', 'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.8);
        quiver3(vp(1), vp(2), vp(3), uv(1)*long_arrow, uv(2)*long_arrow, uv(3)*long_arrow, ...
            'Color', 'g', 'LineWidth', 2.5, 'MaxHeadSize', 0.8);
        quiver3(vp(1), vp(2), vp(3), vd(1)*long_arrow, vd(2)*long_arrow, vd(3)*long_arrow, ...
            'Color', 'b', 'LineWidth', 2.5, 'MaxHeadSize', 0.8);

        text(vp(1), vp(2), vp(3) + 3, sprintf('%d', i), ...
            'FontSize', 16, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'Color', vp_colors(i,:));
    end

    xlabel('X (m)', 'FontSize', 20);
    ylabel('Y (m)', 'FontSize', 20);
    zlabel('Z (m)', 'FontSize', 20);
    title(sprintf('Viewpoint Orientation Detail (First %d Viewpoints)\nCoverage: %.1f%%', ...
        num_show, coverage_percentage), 'FontSize', 22, 'FontWeight', 'bold');
    set(gca, 'Color', 'w', 'LineWidth', 1.2, 'TickDir', 'out', 'Box', 'on');

    % 图例
    h_red = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'Color', 'r', 'LineWidth', 2);
    h_green = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'Color', 'g', 'LineWidth', 2);
    h_blue = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'Color', 'b', 'LineWidth', 2);
    legend([h_red, h_green, h_blue], ...
        {'Right axis (X)', 'Up axis (Y)', 'View direction (Z)'}, ...
        'Location', 'northeastoutside', 'FontSize', 18);
    view(45, 25);

    % print(figC, fullfile(result_folder, 'Figure_C_Viewpoint_Directions'), output_format, output_resolution);
    % close(figC);
    drawnow;
end
