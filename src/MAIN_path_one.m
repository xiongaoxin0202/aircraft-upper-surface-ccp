%% 合并版：多阈值无人机覆盖路径规划 + SCI插图（参数已缩放1/10，与MAIN_viewpoint.m一致）
clc; clear; close all;
tic

%% ==================== 0. 运行模式 ====================
run_mode = 'single';  % 'single'=单阈值(0.99), 'multi'=多阈值循环(0.80:0.01:1.00)

%% ==================== 1. 参数设置（缩放后1/10）====================
% 路径安全参数
min_safe_distance = 3.5;           % 与MAIN_viewpoint.m一致
collision_check_resolution = 0.05;
segment_safety_resolution = 0.5;   % 线段安全检查分辨率
safety_margin = 0.1;
max_safety_iterations = 50;

% 路径规划参数
path_smoothness = 0.1;
max_path_points = 100;

% 上方覆盖参数
min_height_above_cloud = 1.0;
preferred_height_above_cloud = 3.0;

% 相机参数（与MAIN_viewpoint.m一致：FOV-based视锥剔除）
fov_h = 75;          % 水平视场角（度）
fov_v = 65;          % 垂直视场角（度）
near_plane = 0.5;     % 最近探测距离（米）
far_plane = 6;      % 最远探测距离（米）

path_coverage_sampling_interval = 1;

% 路径中间点采样参数
path_sub_segments = 0;  % 每段路径中间采样点数，加密

% 覆盖率阈值（与MAIN_viewpoint.m一致）
if strcmp(run_mode, 'single')
    coverage_thresholds = 0.95;
else
    coverage_thresholds = 0.80:0.01:1.00;
end

% 可视化参数
frustum_visualization_interval = 80;
frustum_length = 6;
frustum_alpha = 0.05;
coordinate_axis_length = 3;
coordinate_visualization_interval = 20;

% SCI图片全局设置
set(0, 'DefaultAxesFontName', 'Times New Roman');
set(0, 'DefaultTextFontName', 'Times New Roman');
set(0, 'DefaultAxesFontSize', 20);
set(0, 'DefaultTextFontSize', 20);
set(0, 'DefaultLineLineWidth', 2);
set(0, 'DefaultAxesLineWidth', 1.5);
set(0, 'DefaultAxesTickDir', 'out');
set(0, 'DefaultAxesBox', 'on');
set(0, 'DefaultAxesGridAlpha', 0.3);
set(0, 'DefaultAxesGridLineStyle', ':');

fprintf('========================================\n');
fprintf('多阈值无人机覆盖路径规划\n');
fprintf('========================================\n');

%% ==================== 2. 循环处理每个覆盖率阈值 ====================
for t_idx = 1:length(coverage_thresholds)
    cov_thr = coverage_thresholds(t_idx);
    fprintf('\n========================================\n');
    fprintf('覆盖率阈值: %.2f (%d/%d)\n', cov_thr, t_idx, length(coverage_thresholds));
    fprintf('========================================\n');
    tic_thr = tic;

    subfolder = sprintf('coverage_%.2f', cov_thr);
    result_folder = fullfile('coverage_threshold_results', subfolder);
    if ~exist(result_folder, 'dir')
        fprintf('文件夹不存在: %s，跳过\n', result_folder);
        continue;
    end

    % ---------- 加载数据 ----------
    fprintf('加载数据...\n');
    tp_file = fullfile(result_folder, 'target_points.mat');
    ov_file = fullfile(result_folder, 'optimal_viewpoints.mat');
    ci_file = fullfile(result_folder, 'coverage_info.mat');
    if ~exist(tp_file, 'file') || ~exist(ov_file, 'file')
        fprintf('必要文件不存在，跳过\n'); continue;
    end
    load(tp_file); load(ov_file);
    if exist(ci_file, 'file'), load(ci_file); end

    fprintf('目标点云: %d 点, 最优视点: %d\n', size(target_points,1), size(safe_selected_viewpoints,1));
    cloud_max_z = max(target_points(:,3));
    cloud_mean_z = mean(target_points(:,3));

    % KD树
    kd_tree = createns(target_points, 'NSMethod', 'kdtree');

    % ---------- 单无人机路径规划 ----------
    fprintf('\n--- 路径规划和安全优化 ---\n');
    tic_path = tic;
    [path, dirs] = generateSingleDronePathWithDirections(...
        safe_selected_viewpoints, selected_normals, path_smoothness, ...
        max_path_points, target_points, kd_tree);

    if ~isempty(path)
        [path, dirs] = accuratePathSafetyWithDirections(path, dirs, ...
            target_points, kd_tree, min_safe_distance, ...
            collision_check_resolution, segment_safety_resolution, ...
            safety_margin, max_safety_iterations, ...
            min_height_above_cloud, preferred_height_above_cloud);
    end

    path_plan_time = toc(tic_path);
    fprintf('  路径规划和安全优化耗时: %.2f 秒\n', path_plan_time);
    % ---------- 路径覆盖率计算 ----------
    fprintf('\n--- 路径覆盖率计算 ---\n');
    if ~isempty(path)
        [path_cov, covered_pts] = computeAccuratePathCoverageWithPoints(...
            path, dirs, target_points, path_coverage_sampling_interval, ...
            near_plane, far_plane, fov_h, fov_v, path_sub_segments);
        path_cov_pct = 100 * mean(path_cov);
        fprintf('路径覆盖率: %.2f%%\n', path_cov_pct);
    else
        path_cov_pct = 0;
        covered_pts = false(size(target_points,1), 1);
    end

    % ---------- 安全性验证 ----------
    fprintf('\n--- 安全性验证 ---\n');
    if ~isempty(path)
        [min_pt, min_seg] = accuratePathSafetyCheck(path, target_points, kd_tree, segment_safety_resolution);
        fprintf('  点距离=%.2f, 线段距离=%.2f (要求>=%.1f)\n', min_pt, min_seg, min_safe_distance);
        if min_pt >= min_safe_distance && min_seg >= min_safe_distance
            fprintf('  ✓ 安全\n');
        else
            fprintf('  ✗ 不安全!\n');
        end
    end

    % ---------- SCI期刊插图（仅显示，不保存文件）----------
    fprintf('\n--- 生成SCI插图 ---\n');
    generatePathSCIFigures(path, dirs, safe_selected_viewpoints, ...
        selected_normals, target_points, covered_pts, path_cov_pct, ...
        kd_tree, min_safe_distance, cov_thr);

    % ---------- 覆盖可视化 ----------
    if ~isempty(path)
        visualizeSingleDroneCoverage(path, dirs, safe_selected_viewpoints, ...
            selected_normals, target_points, covered_pts, path_cov_pct, ...
            fov_h, fov_v, near_plane, far_plane, ...
            frustum_visualization_interval, frustum_length, ...
            frustum_alpha, coordinate_visualization_interval, coordinate_axis_length);
    end

    % ---------- 保存结果 ----------
    fprintf('\n--- 保存结果 ---\n');
    savePathResults(path, dirs, path_cov_pct, cov_thr, result_folder);
    vp_cov = 100*mean(covered_pts);
    if exist('coverage_percentage','var'), vp_cov = coverage_percentage; end
    saveCoverageSummary(result_folder, vp_cov, path_cov_pct, cov_thr, toc(tic_thr), path_plan_time);

    fprintf('阈值 %.2f 完成, 耗时: %.2f 秒\n', cov_thr, toc(tic_thr));
    clear target_points safe_selected_viewpoints selected_normals covered_points;
    clear kd_tree path dirs;
end

fprintf('\n========================================\n');
fprintf('所有阈值处理完成！总耗时: %.2f 秒\n', toc);
fprintf('========================================\n');

%% ==================== 辅助函数 ====================

%% --- 路径生成 ---
function [path, dirs] = generateSingleDronePathWithDirections(vps, vnorms, smoothness, max_pts, target, kd_tree)
    if isempty(vps), path=[]; dirs=[]; return; end
    if size(vps,1) > 1
        dist_m = pdist2(vps, vps);
        order = solveTSP(dist_m);
        vps = vps(order,:); vnorms = vnorms(order,:);
    end
    [path, dirs] = generateSmoothPathWithDirections(vps, vnorms, smoothness, max_pts, target, kd_tree);
end

function [smooth_path, smooth_dirs] = generateSmoothPathWithDirections(wpts, wdirs, smoothness, max_pts, target, kd_tree)
    if size(wpts,1) < 2, smooth_path=wpts; smooth_dirs=wdirs; return; end
    t = 1:size(wpts,1);
    tt = linspace(1, size(wpts,1), max_pts);
    smooth_path = zeros(length(tt),3);
    for dim=1:3
        if size(wpts,1) < 4
            smooth_path(:,dim) = interp1(t, wpts(:,dim), tt, 'linear');
        else
            smooth_path(:,dim) = interp1(t, wpts(:,dim), tt, 'spline');
        end
    end
    smooth_dirs = interpolateDirectionsAccurate(wpts, wdirs, tt);
    if smoothness>0 && size(smooth_path,1)>10
        for dim=1:3
            smooth_path(:,dim) = smooth(smooth_path(:,dim), round(smoothness*10));
        end
        smooth_dirs = smoothDirections(smooth_dirs, round(smoothness*5));
    end
    smooth_dirs = adjustDirectionsToSurface(smooth_path, smooth_dirs, target, kd_tree);
end

function order = solveTSP(dist_m)
    n = size(dist_m,1);
    if n<=1, order=1:n; return; end
    visited = false(1,n); order = zeros(1,n);
    curr = 1; visited(1)=true; order(1)=1;
    for i=2:n
        unv = find(~visited);
        [~,idx] = min(dist_m(curr, unv));
        order(i) = unv(idx); visited(unv(idx))=true; curr = unv(idx);
    end
end

function dirs = interpolateDirectionsAccurate(wpts, wdirs, tt)
    np = length(tt); dirs = zeros(np,3);
    to = 1:size(wpts,1);
    for i=1:np
        tc = tt(i);
        if tc<=to(1), dirs(i,:)=wdirs(1,:);
        elseif tc>=to(end), dirs(i,:)=wdirs(end,:);
        else
            i1 = find(to<=tc,1,'last'); i2 = i1+1;
            if i2>size(wpts,1), dirs(i,:)=wdirs(end,:); continue; end
            alpha = (tc-to(i1))/(to(i2)-to(i1));
            q1 = wdirs(i1,:)/norm(wdirs(i1,:)+eps);
            q2 = wdirs(i2,:)/norm(wdirs(i2,:)+eps);
            dotv = max(-1, min(1, dot(q1,q2)));
            theta = acos(dotv);
            if theta<1e-6, dir= q1;
            else, dir = (sin((1-alpha)*theta)*q1 + sin(alpha*theta)*q2)/sin(theta); end
            dirs(i,:) = dir/(norm(dir)+eps);
        end
    end
end

function sd = smoothDirections(dirs, ws)
    if size(dirs,1)<=ws, sd=dirs; return; end
    sd = zeros(size(dirs));
    for i=1:size(dirs,1)
        s = max(1,i-floor(ws/2)); e = min(size(dirs,1),i+floor(ws/2));
        avg = mean(dirs(s:e,:),1);
        sd(i,:) = avg/(norm(avg)+eps);
    end
end

function adj = adjustDirectionsToSurface(pts, dirs, target, kd)
    adj = dirs;
    for i=1:size(pts,1)
        [idx, dist] = knnsearch(kd, pts(i,:));
        if dist < 5
            to_surf = target(idx,:) - pts(i,:);
            if norm(to_surf)>0
                ideal = to_surf/norm(to_surf);
                blend = 0.3*(1 - dist/5);
                blended = (1-blend)*dirs(i,:)/norm(dirs(i,:)+eps) + blend*ideal;
                adj(i,:) = blended/(norm(blended)+eps);
            end
        end
    end
end

%% --- 安全优化（来自两个文件的相同版本）---
function [safe_path, safe_dirs] = accuratePathSafetyWithDirections(path, dirs, target, kd, ...
    min_sd, pt_res, seg_res, saf_margin, max_iter, min_h, pref_h)
    if isempty(path) || size(path,1)<2, safe_path=path; safe_dirs=dirs; return; end
    orig_n = size(path,1);
    [sampled_p, sampled_d] = adaptiveSamplePathWithDirections(path, dirs, pt_res);
    elevated = applyUpperCoverageOptimization(sampled_p, target, kd, min_h, pref_h);
    pt_safe = ensureAccuratePointSafety(elevated, target, kd, min_sd + saf_margin);
    seg_safe_p = pt_safe; seg_safe_d = sampled_d;
    for iter=1:max_iter
        [seg_safe_p, seg_safe_d, improved, all_safe] = improveSegmentSafetyAccurate(...
            seg_safe_p, seg_safe_d, target, kd, min_sd + saf_margin, seg_res);
        if all_safe || ~improved || size(seg_safe_p,1) > orig_n*10, break; end
    end
    final_p = finalSafetyVerificationAccurate(seg_safe_p, target, kd, min_sd);
    [safe_path, safe_dirs] = safetyPreservingSmoothingAccurate(final_p, seg_safe_d, target, kd, min_sd);
    [fmp, fms] = accuratePathSafetyCheck(safe_path, target, kd, seg_res);
    if fmp<min_sd || fms<min_sd
        [safe_path, safe_dirs] = applyEnhancedSafetyMeasures(safe_path, safe_dirs, target, kd, min_sd, saf_margin);
    end
    ml = min(size(safe_path,1), size(safe_dirs,1));
    safe_path = safe_path(1:ml,:); safe_dirs = safe_dirs(1:ml,:);
end

function elevated = applyUpperCoverageOptimization(path, target, kd, min_h, pref_h)
    elevated = path; cmax = max(target(:,3)); cmean = mean(target(:,3));
    for i=1:size(elevated,1)
        pt = elevated(i,:);
        [idx, ~] = rangesearch(kd, [pt(1:2), cmean], 2);
        if ~isempty(idx{1})
            local_max = max(target(idx{1},3));
            target_z = max(local_max + pref_h, cmean + min_h);
            if pt(3) < target_z, elevated(i,3) = pt(3) + (target_z-pt(3))*0.7; end
            max_allowed = cmax + 5;
            if elevated(i,3) > max_allowed, elevated(i,3) = max_allowed; end
        elseif pt(3) < cmean + min_h
            elevated(i,3) = cmean + min_h;
        end
    end
end

function [sp, sd] = adaptiveSamplePathWithDirections(path, dirs, res)
    if size(path,1)<2, sp=path; sd=dirs; return; end
    total = sum(sqrt(sum(diff(path).^2,2)));
    ns = max(50, min(2000, ceil(total/res)));
    t = 1:size(path,1); tt = linspace(1,size(path,1),ns);
    sp = zeros(length(tt),3);
    for d=1:3
        if size(path,1)>=4, sp(:,d)=interp1(t,path(:,d),tt,'spline');
        else, sp(:,d)=interp1(t,path(:,d),tt,'linear'); end
    end
    sd = interpolateDirectionsAccurate(path, dirs, tt);
end

function safe = ensureAccuratePointSafety(path, target, kd, req)
    safe = path;
    for i=1:size(safe,1)
        [~, d] = knnsearch(kd, safe(i,:));
        if d < req, safe(i,:) = accuratePushToSafety(safe(i,:), target, kd, req); end
    end
end

function [np, nd, imp, all_safe] = improveSegmentSafetyAccurate(path, dirs, target, kd, req, res)
    np = path; nd = dirs; imp = false; all_safe = true; i = 1;
    while i < size(np,1)
        s = np(i,:); e = np(i+1,:); seg_len = norm(e-s);
        if seg_len>0
            nc = max(5, ceil(seg_len/res));
            unsafe = false; worst_t = 0.5; worst_d = inf;
            for j=1:nc
                t = (j-1)/(nc-1);
                pt = s + t*(e-s);
                [~, d] = knnsearch(kd, pt);
                if d<worst_d, worst_d=d; worst_t=t; end
                if d<req, unsafe=true; end
            end
            if unsafe
                all_safe = false; imp = true;
                unsafe_pt = s + worst_t*(e-s);
                safe_pt = accuratePushToSafety(unsafe_pt, target, kd, req+0.2);
                np = [np(1:i,:); safe_pt; np(i+1:end,:)];
                interp_d = interpolateDirectionAtPosition(dirs, i, worst_t);
                nd = [nd(1:i,:); interp_d; nd(i+1:end,:)];
            else, i=i+1; end
        else, i=i+1; end
    end
end

function safe = accuratePushToSafety(pt, target, kd, req)
    [idx, d] = knnsearch(kd, pt);
    nearest = target(idx,:);
    if d>=req, safe=pt; return; end
    push = pt - nearest + [0,0,0.15];
    if norm(push)<1e-6
        [ns,~] = rangesearch(kd, pt, 1.5);
        if length(ns{1})>3
            [~,~,V] = pca(target(ns{1},:));
            push = V(:,3)' + [0,0,0.15];
            if dot(push, pt-mean(target(ns{1},:)))<0, push=-push; end
        else, push = [randn(1,2),0.2]; end
    end
    push = push/(norm(push)+eps);
    safe = pt + push*(req-d+0.3);
    [~, nd] = knnsearch(kd, safe);
    if nd<req, safe = accuratePushToSafety(safe, target, kd, req); end
end

function d = interpolateDirectionAtPosition(dirs, idx, t)
    if idx>=size(dirs,1), d=dirs(end,:);
    else
        d1=dirs(idx,:)/norm(dirs(idx,:)+eps);
        d2=dirs(idx+1,:)/norm(dirs(idx+1,:)+eps);
        dotv = max(-1,min(1,dot(d1,d2))); th=acos(dotv);
        if th<1e-6, d=d1; else, d=(sin((1-t)*th)*d1+sin(t*th)*d2)/sin(th); end
        d=d/(norm(d)+eps);
    end
end

function safe = finalSafetyVerificationAccurate(path, target, kd, min_sd)
    safe = path;
    for i=1:size(safe,1)
        [~,d]=knnsearch(kd,safe(i,:));
        if d<min_sd, safe(i,:)=accuratePushToSafety(safe(i,:),target,kd,min_sd+0.2); end
    end
end

function [sp, sd] = safetyPreservingSmoothingAccurate(path, dirs, target, kd, min_sd)
    if size(path,1)<3, sp=path; sd=dirs; return; end
    sp = path;
    for si=1:2
        for i=2:size(sp,1)-1
            orig = sp(i,:); ws = min(3, floor(size(sp,1)/10));
            s = max(1,i-ws); e = min(size(sp,1),i+ws);
            w = gausswin(e-s+1); w = w/sum(w);
            sm = zeros(1,3);
            for j=1:length(w), sm = sm + w(j)*sp(s+j-1,:); end
            [~,sm_d]=knnsearch(kd,sm); [~,orig_d]=knnsearch(kd,orig);
            if sm_d>=min_sd && sm_d>=orig_d*0.9, sp(i,:)=sm; end
        end
    end
    for i=1:size(sp,1)
        [~,d]=knnsearch(kd,sp(i,:));
        if d<min_sd+0.1, sp(i,:)=accuratePushToSafety(sp(i,:),target,kd,min_sd+0.2); end
    end
    sd = dirs;
end

function [sp, sd] = applyEnhancedSafetyMeasures(path, dirs, target, kd, min_sd, sm)
    sp = path; sd = dirs;
    for i=1:size(sp,1)
        [~,d]=knnsearch(kd,sp(i,:));
        if d<min_sd+sm, sp(i,:)=accuratePushToSafety(sp(i,:),target,kd,min_sd+sm+0.5); end
    end
    i=1;
    while i<size(sp,1)
        seg_len = norm(sp(i+1,:)-sp(i,:));
        if seg_len>0.5
            nc = max(10,ceil(seg_len/0.05));
            for j=1:nc
                t=j/nc; pt = sp(i,:)+t*(sp(i+1,:)-sp(i,:));
                [~,d]=knnsearch(kd,pt);
                if d<min_sd
                    safe = accuratePushToSafety(pt,target,kd,min_sd+sm+0.3);
                    sp=[sp(1:i,:);safe;sp(i+1:end,:)];
                    sd=[sd(1:i,:);sd(i,:);sd(i+1:end,:)];
                    break;
                end
            end
        end
        i=i+1;
    end
end

%% --- 覆盖计算（Z-Buffer遮挡剔除 + 中间点采样）---
function [path_cov, covered] = computeAccuratePathCoverageWithPoints(path, dirs, target, interval, ...
        near, far, fov_h, fov_v, sub_segments)
    % 计算路径整体覆盖率，包含路径点及中间点
    nt = size(target,1); path_cov = false(nt,1); covered = false(nt,1);
    valid = 1:max(1,interval):size(path,1);
    nv = length(valid);

    % 收集所有采样点位置和方向（含中间插值点）
    all_pos = []; all_dir = [];
    for vi = 1:nv
        i = valid(vi);
        if i>size(path,1)||i>size(dirs,1), continue; end
        all_pos = [all_pos; path(i,:)];
        all_dir = [all_dir; dirs(i,:)];

        % 在当前段与下一段之间插入中间点
        if vi < nv
            next_i = valid(vi+1);
            if next_i > i
                d1 = dirs(i,:)/(norm(dirs(i,:))+eps);
                d2 = dirs(next_i,:)/(norm(dirs(next_i,:))+eps);
                dotv = max(-1, min(1, dot(d1,d2)));
                theta = acos(dotv);
                for s = 1:sub_segments
                    t = s / (sub_segments + 1);
                    % 位置线性插值
                    interp_pos = path(i,:) + t * (path(next_i,:) - path(i,:));
                    % 方向球面插值
                    if theta < 1e-6
                        interp_dir = d1;
                    else
                        interp_dir = (sin((1-t)*theta)*d1 + sin(t*theta)*d2) / sin(theta);
                    end
                    interp_dir = interp_dir/(norm(interp_dir)+eps);
                    all_pos = [all_pos; interp_pos];
                    all_dir = [all_dir; interp_dir];
                end
            end
        end
    end

    % 批量计算覆盖（Z-Buffer遮挡剔除）
    npos = size(all_pos,1);
    batch_size = 50;
    fprintf('  路径覆盖计算: %d个采样点（含%d个中间点）\n', npos, npos-nv);
    for b = 1:ceil(npos/batch_size)
        s = (b-1)*batch_size+1; e = min(b*batch_size, npos);
        for ii = s:e
            cc = computeCoverageWithOcclusion(all_pos(ii,:), all_dir(ii,:), target, ...
                near, far, fov_h, fov_v, 0, 0);
            path_cov = path_cov | cc;
            covered = covered | cc;
        end
        if mod(b,10)==0
            fprintf('  批次 %d/%d, 当前覆盖: %.2f%%\n', b, ceil(npos/batch_size), 100*mean(path_cov));
        end
    end
end

%% --- Z-Buffer遮挡剔除覆盖计算（与MAIN_viewpoint.m一致）---
function coverage = computeCoverageWithOcclusion(viewpoint, direction, points, near, far, ...
    fov_h, fov_v, ~, ~)
    % Z-Buffer深度缓冲遮挡剔除覆盖计算
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

    % 自适应网格分辨率
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

%% --- 安全检查 ---
function [min_pt, min_seg] = accuratePathSafetyCheck(path, target, kd, res)
    if isempty(path), min_pt=inf; min_seg=inf; return; end
    ds = zeros(size(path,1),1);
    for i=1:size(path,1), [~,ds(i)]=knnsearch(kd,path(i,:)); end
    min_pt = min(ds); min_seg = inf;
    for i=1:size(path,1)-1
        s=path(i,:); e=path(i+1,:); l=norm(e-s);
        if l>0
            for j=1:max(3,ceil(l/res))
                t=(j-1)/(max(3,ceil(l/res))-1);
                [~,d]=knnsearch(kd,s+t*(e-s));
                if d<min_seg, min_seg=d; end
            end
        end
    end
end

%% --- SCI插图（仅显示，不保存）---
function generatePathSCIFigures(path, dirs, vps, vnorms, target, covered, cov_pct, kd, min_sd, cov_thr)
    if isempty(path), return; end
    % Figure A: 三维视图
    figA = figure('Position',[50,50,1200,550],'Color','w','Name','Figure A: 3D Rendered Path');
    hold on; grid on; axis equal;
    % 点云（覆盖状态着色）
    uncov_i = find(~covered);
    cov_i = find(covered);
    if ~isempty(uncov_i)
        scatter3(target(uncov_i,1),target(uncov_i,2),target(uncov_i,3),5,[0.7,0.7,0.7],'filled','MarkerFaceAlpha',0.3);
    end
    if ~isempty(cov_i)
        scatter3(target(cov_i,1),target(cov_i,2),target(cov_i,3),8,'g','filled','MarkerFaceAlpha',0.6);
    end
    scatter3(vps(:,1),vps(:,2),vps(:,3),60,'b','filled','MarkerEdgeColor','k','LineWidth',1);
    % 路径
    plot3(path(:,1),path(:,2),path(:,3),'r-','LineWidth',3);
    scatter3(path(1,1),path(1,2),path(1,3),150,'go','filled','MarkerEdgeColor','k','LineWidth',2);
    scatter3(path(end,1),path(end,2),path(end,3),150,'ro','filled','MarkerEdgeColor','k','LineWidth',2);
    % % 安全包络
    % step = max(1,floor(size(path,1)/30));
    % for i=1:step:size(path,1)
    %     [X,Y,Z]=sphere(12);
    %     surf(X*min_sd+path(i,1), Y*min_sd+path(i,2), Z*min_sd+path(i,3), ...
    %         'FaceAlpha',0.04,'EdgeColor','none','FaceColor',[0.5 0.5 0.5]);
    % end
    xlabel('X (m)','FontSize',20); ylabel('Y (m)','FontSize',20); zlabel('Z (m)','FontSize',20);
    path_len = sum(sqrt(sum(diff(path).^2,2)));
    title(sprintf('Aircraft Upper Surface Inspection Path | Coverage: %.1f%% | Path: %.1f m',cov_pct,path_len),...
        'FontSize',22,'FontWeight','bold');
    legend({'Uncovered','Covered','Viewpoints','Path','Start','End','Safety envelope'},...
        'Location','northeastoutside','FontSize',18);
    view(45,25); light('Position',[1,1,1],'Style','infinite'); lighting gouraud; material dull;
    set(gca,'Color','w','LineWidth',1.2,'TickDir','out','Box','on'); drawnow;

    % Figure B: 高度剖面 + 安全距离
    np = size(path,1); dist_surf = zeros(np,1); height_above = zeros(np,1);
    for i=1:np
        [idx,d]=knnsearch(kd,path(i,:)); dist_surf(i)=d; height_above(i)=path(i,3)-target(idx,3);
    end
    cum_len = [0;cumsum(sqrt(sum(diff(path).^2,2)))];
    figB = figure('Position',[100,100,1100,500],'Color','w','Name','Figure B: Height and Safety Profile');
    subplot(1,2,1); hold on; grid on; set(gca,'Color','w','LineWidth',1.2,'TickDir','out','Box','on');
    plot(cum_len,path(:,3),'b-','LineWidth',2.5);
    surf_z = zeros(np,1);
    for i=1:np, [~,d]=knnsearch(kd,path(i,:)); surf_z(i)=target(idx,3); end
    plot(cum_len,surf_z,'k--','LineWidth',1.8);
    xlabel('Cumulative Path Length (m)','FontSize',18); ylabel('Z (m)','FontSize',18);
    title('Path Height Profile','FontSize',20,'FontWeight','bold');
    legend({'UAV altitude','Surface altitude'},'Location','best','FontSize',16);
    subplot(1,2,2); hold on; grid on; set(gca,'Color','w','LineWidth',1.2,'TickDir','out','Box','on');
    plot(cum_len,dist_surf,'r-','LineWidth',2.5);
    yline(min_sd,'g--','LineWidth',2,'Label',sprintf('Min safe = %.1f m',min_sd),'FontSize',16);
    xlabel('Cumulative Path Length (m)','FontSize',18); ylabel('Distance to Surface (m)','FontSize',18);
    title('Safety Distance','FontSize',20,'FontWeight','bold');
    ylim([0,max(dist_surf)*1.1]);
    sgtitle(sprintf('Path Analysis | Min Clearance: %.2f m | Avg: %.2f m',min(dist_surf),mean(dist_surf)),...
        'FontSize',22,'FontWeight','bold');
    drawnow;
end

%% --- 可视化 ---
function visualizeSingleDroneCoverage(path, dirs, vps, vnorms, target, covered, cov_pct, ...
        fov_h, fov_v, near, far, interval, fl, alpha, coord_i, coord_l)
    fig = figure('Position',[100,100,1200,550],'Color','w','Name','Path Coverage');
    subplot(1,2,1); hold on; grid on; axis equal; set(gca,'Color','w');
    cov_i = find(covered); uncov_i = find(~covered);
    if ~isempty(uncov_i)
        scatter3(target(uncov_i,1),target(uncov_i,2),target(uncov_i,3),8,[0.7,0.7,0.7],'filled','MarkerFaceAlpha',0.4);
    end
    if ~isempty(cov_i)
        scatter3(target(cov_i,1),target(cov_i,2),target(cov_i,3),12,'g','filled','MarkerFaceAlpha',0.7);
    end
    scatter3(vps(:,1),vps(:,2),vps(:,3),60,'b','filled','MarkerEdgeColor','k');
    if ~isempty(path)
        plot3(path(:,1),path(:,2),path(:,3),'r-','LineWidth',3);
        scatter3(path(1,1),path(1,2),path(1,3),120,'go','filled','MarkerEdgeColor','k','LineWidth',2);
        scatter3(path(end,1),path(end,2),path(end,3),120,'ro','filled','MarkerEdgeColor','k','LineWidth',2);
        drawPathFrustumCoverage(path,dirs,fov_h,fov_v,near,far,interval,fl,alpha,'cyan');
    end
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title(sprintf('Coverage: %.2f%%',cov_pct));
    legend('Uncovered','Covered','Viewpoints','Path','Start','End','FOV','Location','best'); view(3);

    subplot(1,2,2); hold on; grid on; axis equal; set(gca,'Color','w');
    if ~isempty(path)
        plot3(path(:,1),path(:,2),path(:,3),'r-','LineWidth',4);
        scatter3(path(1,1),path(1,2),path(1,3),120,'go','filled','MarkerEdgeColor','k','LineWidth',3);
        scatter3(path(end,1),path(end,2),path(end,3),120,'ro','filled','MarkerEdgeColor','k','LineWidth',3);
        drawPathCoordinateSystems(path,dirs,coord_i,coord_l);
        if ~isempty(uncov_i)
            s_uncov = uncov_i(1:min(500,length(uncov_i)));
            scatter3(target(s_uncov,1),target(s_uncov,2),target(s_uncov,3),8,[0.7,0.7,0.7],'filled','MarkerFaceAlpha',0.3);
        end
        if ~isempty(cov_i)
            s = cov_i(1:min(500,length(cov_i)));
            scatter3(target(s,1),target(s,2),target(s,3),12,'g','filled','MarkerFaceAlpha',0.6);
        end
    end
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Path Detail & Coordinate Axes');
    legend('Path','Start','End','Uncovered','Covered','Location','best'); view(3);
    drawnow;
end

function drawPathFrustumCoverage(path, dirs, fov_h, fov_v, near, far, interval, fl, alpha, color)
    if isempty(path)||isempty(dirs), return; end
    fov_x = deg2rad(fov_h); fov_y = deg2rad(fov_v);
    valid = 1:interval:size(path,1);
    for ii=1:length(valid)
        i=valid(ii); pos=path(i,:); d=dirs(i,:);
        if norm(d)>0, d=d/(norm(d)+eps); drawFrustum(pos,d,fov_x,fov_y,near,far,fl,alpha,color); end
    end
end

function drawFrustum(pos, dir, fov_x, fov_y, near, far, fl, alpha, color)
    nhw = near*tan(fov_x/2); nhh = near*tan(fov_y/2);
    fhw = far*tan(fov_x/2); fhh = far*tan(fov_y/2);
    z = dir/(norm(dir)+eps); up = [0,0,1];
    if abs(dot(z,up))>0.99, up=[0,1,0]; end
    x = cross(up,z)/(norm(cross(up,z))+eps); y = cross(z,x);
    nc = pos+near*z; fc = pos+fl*z;
    ntl = nc+nhh*y-nhw*x; ntr = nc+nhh*y+nhw*x;
    nbl = nc-nhh*y-nhw*x; nbr = nc-nhh*y+nhw*x;
    ftl = fc+fhh*y-fhw*x; ftr = fc+fhh*y+fhw*x;
    fbl = fc-fhh*y-fhw*x; fbr = fc-fhh*y+fhw*x;
    verts = [ntl;ntr;ftr;ftl;nbl;nbr;fbr;fbl;ntl;nbl;fbl;ftl;ntr;nbr;fbr;ftr];
    faces = [1,2,3,4;5,6,7,8;9,10,11,12;13,14,15,16];
    patch('Vertices',verts,'Faces',faces,'FaceColor',color,'FaceAlpha',alpha,...
        'EdgeColor',color,'EdgeAlpha',0.3);
    pts = [ftl;ftr;fbr;fbl];
    for k=1:4
        plot3([pos(1),pts(k,1)],[pos(2),pts(k,2)],[pos(3),pts(k,3)],...
            'Color',color,'LineStyle','--','LineWidth',0.5);
    end
end

function drawPathCoordinateSystems(path, dirs, interval, al)
    if isempty(path)||isempty(dirs), return; end
    valid = 1:interval:size(path,1);
    for ii=1:length(valid)
        i=valid(ii); pos=path(i,:); d=dirs(i,:);
        if norm(d)>0
            d=d/(norm(d)+eps); up=[0,0,1];
            if abs(dot(d,up))>0.99, up=[0,1,0]; end
            r=cross(up,d)/(norm(cross(up,d))+eps); u=cross(d,r);
            quiver3(pos(1),pos(2),pos(3),r(1)*al,r(2)*al,r(3)*al,'Color','r','LineWidth',2,'MaxHeadSize',0.8);
            quiver3(pos(1),pos(2),pos(3),u(1)*al,u(2)*al,u(3)*al,'Color','g','LineWidth',2,'MaxHeadSize',0.8);
            quiver3(pos(1),pos(2),pos(3),d(1)*al,d(2)*al,d(3)*al,'Color','b','LineWidth',2,'MaxHeadSize',0.8);
        end
    end
end

%% --- 保存结果 ---
function savePathResults(path, dirs, cov_pct, cov_thr, folder)
    if isempty(path), return; end
    % 删除同一覆盖率阈值下已有的路径数据文件，确保覆盖旧结果
    delete_pattern = fullfile(folder, sprintf('path_viewpoint_%.0f_*', cov_thr*100));
    old_files = dir(delete_pattern);
    for k = 1:length(old_files)
        delete(fullfile(folder, old_files(k).name));
    end
    fn = sprintf('path_viewpoint_%.0f_path_%.0f', cov_thr*100, cov_pct);
    pd = [];
    if size(path,1)==size(dirs,1)
        pd = [path, dirs];
    end
    csvwrite(fullfile(folder,[fn '.csv']), pd);
    save(fullfile(folder,[fn '.mat']),'path','dirs','cov_pct','cov_thr');
    fprintf('  保存路径: %s\n', [fn '.csv']);
end

function saveCoverageSummary(folder, vp_cov, path_cov, cov_thr, total_time, path_plan_time)
    % 删除同一覆盖率阈值下已有的摘要文件，确保覆盖旧结果
    delete_pattern = fullfile(folder, sprintf('summary_viewpoint_%.0f_*', cov_thr*100));
    old_files = dir(delete_pattern);
    for k = 1:length(old_files)
        delete(fullfile(folder, old_files(k).name));
    end
    fn = sprintf('summary_viewpoint_%.0f_path_%.0f.txt', cov_thr*100, path_cov);
    fid = fopen(fullfile(folder,fn),'w');
    fprintf(fid,'覆盖率阈值: %.2f\n',cov_thr);
    fprintf(fid,'视点覆盖率: %.2f%%\n',vp_cov);
    fprintf(fid,'路径覆盖率: %.2f%%\n',path_cov);
    fprintf(fid,'覆盖率差值: %.2f%%\n',path_cov-vp_cov);
    fprintf(fid,'总处理时间: %.2f 秒\n',total_time);
    fprintf(fid,'路径规划时间: %.2f 秒\n',path_plan_time);
    fclose(fid);
    fprintf('  保存摘要: %s\n',fn);
end
