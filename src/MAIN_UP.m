%% 预处理流水线：上表面9分区 → 缩放10倍 → 候选视点 → 下采样子样带法向量
% 功能：
%   1. 读取 seg_all/ 下 1~9.pcd 共 9 个上表面分区点云
%   2. 坐标统一缩小 10 倍，保存为 seg_all/UP-AIR.pcd（待检点云）
%   3. 每个点沿法线方向移动最优视距 d，获得候选视点位置 & 法线方向
%   4. 合并全部候选视点 → seg_all/zong.ply
%   5. 对 zong.ply 下采样 + 表面法向量映射 → seg_all/view-normals.ply
%   6. 生成高质量论文插图
clc; clear; close all;
tic

%% ==================== 1. 参数设置 ====================
% 最优视距（与点云缩放匹配：原 40 → 缩放 10 倍后 = 4）
optimal_dist = 4;   %4  3
% 法向量估计邻域点数
normal_k = 10;
% 下采样体素大小（与缩放后尺度匹配）
grid_step = 1; % 0.5

fprintf('========================================\n');
fprintf('上表面点云预处理流水线\n');
fprintf('  最优视距 d = %.1f\n', optimal_dist);
fprintf('  下采样步长 = %.1f\n', grid_step);
fprintf('========================================\n\n');

%% ==================== 2. 读取 9 个分区点云并缩小 10 倍 ====================
fprintf('第 1 步：读取 9 个分区点云...\n');
all_pts = [];
for i = 1:9
    cloud_i = pcread(sprintf('E:/XAX_ws/飞机上表面扫掠 -- 第三篇-第四篇-第五篇/飞机上表面的覆盖检查路径规划研究 -- 论文 -- 第三篇/论文投稿/英文SCI投稿/Robotica -- 投稿  ROB-2026-0389/重投材料/aircraft-upper-surface-ccp/data/B757_segmented/%d.pcd', i));
    all_pts = [all_pts; cloud_i.Location];
    fprintf('  分区 %d: %d 点\n', i, size(cloud_i.Location, 1));
end
pts_orig = all_pts;
fprintf('  合计: %d 点\n\n', size(all_pts, 1));

% 坐标统一缩小 10 倍（X/Y/Z 均 /10）
fprintf('第 2 步：坐标缩小 10 倍并保存 UP-AIR.pcd...\n');
pts_scaled = pts_orig / 1;    % 10
pcwrite(pointCloud(pts_scaled), 'E:/XAX_ws/飞机上表面扫掠 -- 第三篇-第四篇-第五篇/飞机上表面的覆盖检查路径规划研究 -- 论文 -- 第三篇/论文投稿/英文SCI投稿/Robotica -- 投稿  ROB-2026-0389/重投材料/aircraft-upper-surface-ccp/data/B757_segmented/UP-AIR.pcd', 'Encoding', 'ascii');
fprintf('  UP-AIR.pcd 保存完成，%d 点\n\n', size(pts_scaled, 1));

%% ==================== 3. 逐分区生成候选视点 ====================
fprintf('第 3 步：逐分区生成候选视点（沿法线移动最优视距）...\n');
all_viewpoints = [];       % 视点位置
all_surface_normals = [];  % 每个视点对应的表面法向量（指向机外）

for i = 1:9
    cloud_i = pcread(sprintf('E:/XAX_ws/飞机上表面扫掠 -- 第三篇-第四篇-第五篇/飞机上表面的覆盖检查路径规划研究 -- 论文 -- 第三篇/论文投稿/英文SCI投稿/Robotica -- 投稿  ROB-2026-0389/重投材料/aircraft-upper-surface-ccp/data/B757_segmented/%d.pcd', i));
    loc = cloud_i.Location;
    loc_scaled = loc / 1;   % 使用缩放后坐标 10

    % 法向量估计（MATLAB pcnormals 符号不确定，需后续修正方向）
    normals = pcnormals(pointCloud(loc_scaled), normal_k);

    % 沿法线方向移动最优视距，同时确定正确的指向
    % 上表面朝外的法向量必须 Z>0（朝上），利用此几何约束校正 pcnormals 的符号歧义
    M = zeros(size(loc_scaled, 1), 3);
    corrected_normals = zeros(size(loc_scaled, 1), 3);
    for j = 1:size(loc_scaled, 1)
        if normals(j, 3) >= 0
            % normal 指向朝上（即外侧），视点沿法线向外移动
            M(j, :) = loc_scaled(j, :) + optimal_dist * normals(j, :);
            corrected_normals(j, :) = normals(j, :);
        else
            % normal 指向朝下（内侧），取反后沿法线反向（朝外）移动
            M(j, :) = loc_scaled(j, :) - optimal_dist * normals(j, :);
            corrected_normals(j, :) = -normals(j, :);
        end
    end
    all_viewpoints = [all_viewpoints; M];
    all_surface_normals = [all_surface_normals; corrected_normals];

    % 可视化（缩放后的坐标空间）
    figure('Position', [100, 100, 800, 600], ...
        'Color', 'w', 'Name', sprintf('分区 %d', i));
    pcshow(loc_scaled, [0, 0, 1], 'MarkerSize', 30);  % 蓝色：原始点
    set(gca, 'Color', 'w');
    hold on;
    pcshow(M, [0, 1, 0], 'MarkerSize', 30);            % 绿色：候选视点
    hold off;
    title(sprintf('Partition %d: Surface (blue) → Viewpoints (green)', i));
    xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on; axis equal;

    fprintf('  分区 %d: %d 个候选视点\n', i, size(M, 1));
end
fprintf('  合计候选视点: %d\n\n', size(all_viewpoints, 1));

%% ==================== 4. 合并全部候选视点 → zong.ply ====================
fprintf('第 4 步：合并全部候选视点 → seg_all/zong.ply...\n');
zong_cloud = pointCloud(all_viewpoints);
pcwrite(zong_cloud, 'E:/XAX_ws/飞机上表面扫掠 -- 第三篇-第四篇-第五篇/飞机上表面的覆盖检查路径规划研究 -- 论文 -- 第三篇/论文投稿/英文SCI投稿/Robotica -- 投稿  ROB-2026-0389/重投材料/aircraft-upper-surface-ccp/data/B757_segmented/zong.ply', 'Encoding', 'ascii');
fprintf('  zong.ply 保存完成，%d 点\n\n', size(all_viewpoints, 1));

%% ==================== 5. 下采样 + 表面法向量映射 → view-normals.ply ====================
% 注意：view-normals.ply 的法向量存储的是指向机外的表面法向量
% MAIN_viewpoint.m 中会用 -cand_normals 得到指向机内的视线方向
fprintf('第 5 步：下采样并映射表面法向量 → view-normals.ply...\n');
fprintf('  加载 zong.ply...\n');
zong = pcread('E:/XAX_ws/飞机上表面扫掠 -- 第三篇-第四篇-第五篇/飞机上表面的覆盖检查路径规划研究 -- 论文 -- 第三篇/论文投稿/英文SCI投稿/Robotica -- 投稿  ROB-2026-0389/重投材料/aircraft-upper-surface-ccp/data/B757_segmented/zong.ply');
fprintf('  原始点数: %d\n', zong.Count);

fprintf('  体素下采样 (gridAverage, step=%.1f)...\n', grid_step);
downsampled = pcdownsample(zong, 'gridAverage', grid_step);
fprintf('  下采样后点数: %d\n', downsampled.Count);

% 法向量映射：在下采样后的每个视点位置，从原始视点集中找到最近邻，
% 将该最近邻对应的表面法向量赋给下采样后的视点
fprintf('  表面法向量映射（KNN最近邻 %d → %d）...\n', ...
    size(downsampled.Location, 1), size(all_viewpoints, 1));
[indices] = knnsearch(all_viewpoints, downsampled.Location);
downsampled.Normal = all_surface_normals(indices, :);

fprintf('  保存到 seg_all/view-normals.ply...\n');
pcwrite(downsampled, 'E:/XAX_ws/飞机上表面扫掠 -- 第三篇-第四篇-第五篇/飞机上表面的覆盖检查路径规划研究 -- 论文 -- 第三篇/论文投稿/英文SCI投稿/Robotica -- 投稿  ROB-2026-0389/重投材料/aircraft-upper-surface-ccp/data/B757_segmented/view-normals.ply');
fprintf('  view-normals.ply 保存完成，%d 个候选视点（带法向量）\n\n', downsampled.Count);

%% ==================== 6. 高质量论文插图 ====================
fprintf('第 6 步：生成高质量论文插图...\n');

% SCI 期刊全局字体设置（高分辨率出版标准）
set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultAxesFontSize', 20);
set(0, 'DefaultTextFontSize', 20);
set(0, 'DefaultLineLineWidth', 2);
set(0, 'DefaultAxesLineWidth', 1.5);
set(0, 'DefaultAxesTickDir', 'out');
set(0, 'DefaultAxesBox', 'on');

% ---- Figure D: 全部候选视点 + 待检点云 全景图 ----
fprintf('  显示 Figure D: 候选视点全景分布...\n');
figD = figure('Position', [50, 50, 1000, 500], 'Color', 'w', ...
    'Name', 'Figure D: Candidate Viewpoints Overview');
hold on; grid on; axis equal;

% 待检点云 — 灰色半透明背景
pcshow(pts_scaled, [0.5 0.5 0.5], 'MarkerSize', 8);
set(gca, 'Color', [1 1 1]);  % 白底
set(gca, 'LineWidth', 1.2, 'TickDir', 'out', 'Box', 'on');

% 候选视点
vp_pt = downsampled.Location;
vp_norm = downsampled.Normal;
num_vp = size(vp_pt, 1);

% 用 jet 颜色映射显示高度层次
vp_z = vp_pt(:, 3);
scatter3(vp_pt(:,1), vp_pt(:,2), vp_pt(:,3), 24, ...
    vp_z, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.6);

% 视点方向箭头
arrow_len = 3;
arrow_step = max(1, floor(num_vp / 30));
for i = 1:arrow_step:num_vp
    vp = vp_pt(i, :);
    vd = vp_norm(i, :) / norm(vp_norm(i, :));
    quiver3(vp(1), vp(2), vp(3), ...
        -vd(1)*arrow_len, -vd(2)*arrow_len, -vd(3)*arrow_len, ...
        'Color', [0.0 0.0 0.6], 'LineWidth', 2, 'MaxHeadSize', 0.5, ...
        'AutoScale', 'off');
end

xlabel('X (m)', 'FontSize', 20);
ylabel('Y (m)', 'FontSize', 20);
zlabel('Z (m)', 'FontSize', 20);
title('Aircraft Upper Surface: Target Point Cloud and Candidate Viewpoints', ...
    'FontSize', 22, 'FontWeight', 'bold');
view(45, 25);
light('Position', [1 1 1], 'Style', 'infinite');
lighting gouraud;

% 图例
h_surf = scatter3(NaN, NaN, NaN, 20, [0.6 0.6 0.6], 'filled');
h_vp = scatter3(NaN, NaN, NaN, 20, 'b', 'filled', 'MarkerEdgeColor', 'k');
h_dir = quiver3(NaN, NaN, NaN, NaN, NaN, NaN, 'Color', [0.0 0.0 0.6], 'LineWidth', 2);
legend([h_surf, h_vp, h_dir], ...
    {'Target surface (UP-AIR.pcd)', 'Candidate viewpoints (view-normals.ply)', ...
    'Viewing direction'}, ...
    'Location', 'northeastoutside', 'FontSize', 18);
colormap(jet);
h_cb = colorbar('Location', 'eastoutside');
set(h_cb, 'FontSize', 16, 'FontName', 'Times New Roman');
ylabel(h_cb, 'Viewpoint Z (m)', 'FontSize', 18);
drawnow;

% ---- Figure E: 视点方向玫瑰图/极坐标统计 ----
fprintf('  显示 Figure E: 视点方向统计...\n');
figE = figure('Position', [50, 50, 1000, 500], 'Color', 'w', ...
    'Name', 'Figure E: View Direction Statistics');

subplot(1, 2, 1);
vp_dirs = -vp_norm;  % 视线方向（指向机内）
vp_dirs = vp_dirs ./ vecnorm(vp_dirs, 2, 2);
% 方位角(azimuth)和俯仰角(elevation)
az = atan2d(vp_dirs(:,2), vp_dirs(:,1));
el = asind(vp_dirs(:,3));
polarhistogram(deg2rad(az), 36, 'FaceColor', [0.2 0.4 0.8], ...
    'EdgeColor', 'w', 'LineWidth', 1);
set(gca, 'Color', 'w');
set(gca, 'LineWidth', 1.2, 'TickDir', 'out', 'Box', 'on');
title('Azimuth Distribution of Viewing Directions', ...
    'FontSize', 18, 'FontWeight', 'bold');

subplot(1, 2, 2);
histogram(el, 20, 'FaceColor', [0.2 0.6 0.2], 'EdgeColor', 'w', ...
    'LineWidth', 1);
set(gca, 'Color', 'w');
set(gca, 'LineWidth', 1.2, 'TickDir', 'out', 'Box', 'on');
xlabel('Elevation Angle (deg)', 'FontSize', 18);
ylabel('Count', 'FontSize', 18);
title('Elevation Distribution of Viewing Directions', ...
    'FontSize', 18, 'FontWeight', 'bold');
grid on;
xlim([-90, 90]);
drawnow;

%% ==================== 7. 最终汇总 ====================
fprintf('========================================\n');
fprintf('预处理流水线全部完成！\n');
fprintf('  待检点云: seg_all/UP-AIR.pcd (%d 点)\n', size(pts_scaled, 1));
fprintf('  候选视点: seg_all/zong.ply (%d 点)\n', size(all_viewpoints, 1));
fprintf('  最终视点: seg_all/view-normals.ply (%d 点，带法向量)\n', downsampled.Count);
fprintf('  总耗时: %.2f 秒\n', toc);
fprintf('========================================\n');
