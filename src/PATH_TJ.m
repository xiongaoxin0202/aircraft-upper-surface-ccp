%% PATH_TJ.m - 路径规划结果综合分析（中文版 + 英文版）
% 整合自: lujing_tj.m, tj.m, tj_yingwen.m, tj2.m, tj2_yingwen.m
% 功能: 从coverage_threshold_results读取数据，生成中英文双版学术论文图
% 输出: analysis_comprehensive_results/（含Fig_Chinese/ Fig_English/ Data/）
clc; clear; close all;

% SCI 期刊全局字体设置
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
fprintf('PATH_TJ: 路径规划结果综合分析\n');
fprintf('========================================\n');

%% ========== [1] 参数设置 ==========
DATA_FOLDER = 'coverage_threshold_results';
OUT_ROOT    = 'analysis_comprehensive_results';
OUT_CN      = fullfile(OUT_ROOT, 'Fig_Chinese');
OUT_EN      = fullfile(OUT_ROOT, 'Fig_English');
OUT_DATA    = fullfile(OUT_ROOT, 'Data');

% 覆盖率阈值范围设置（可修改）
% 例如：想统计80%~100%所有数据，设 COVERAGE_MIN=0.80, COVERAGE_MAX=1.00
%       想统计90%~99%，设 COVERAGE_MIN=0.90, COVERAGE_MAX=0.99
COVERAGE_MIN = 0.80;   % 最小覆盖率阈值（含）
COVERAGE_MAX = 1.00;   % 最大覆盖率阈值（含）

if exist(OUT_ROOT, 'dir')
    try
        rmdir(OUT_ROOT, 's');
    catch
        warning('无法删除输出目录，将覆盖保存文件');
    end
end
mkdir(OUT_CN); mkdir(OUT_EN); mkdir(OUT_DATA);
fprintf('输出目录: %s\n', OUT_ROOT);

subfolders = dir(fullfile(DATA_FOLDER, 'coverage_*'));
subfolders = subfolders([subfolders.isdir]);
[~, idx] = sort({subfolders.name});
subfolders = subfolders(idx);
fprintf('找到 %d 个覆盖率文件夹\n', length(subfolders));

%% ========== [2] 数据采集与指标计算 ==========
results = struct();
valid_idx = 0;
conv_data = {};  % 收敛历史数据

for fi = 1:length(subfolders)
    sf = subfolders(fi);
    sp = fullfile(DATA_FOLDER, sf.name);
    
    tok = regexp(sf.name, '(\d+\.\d+)', 'tokens');
    if isempty(tok), continue; end
    threshold = str2double(tok{1}{1});
    if threshold < COVERAGE_MIN || threshold > COVERAGE_MAX, continue; end  % 受COVERAGE_MIN/MAX控制
    
    % ---- 读取summary.txt ----
    sum_file = fullfile(sp, 'summary.txt');
    if ~exist(sum_file, 'file'), continue; end
    txt = fileread(sum_file);
    vp_count = NaN; vp_cov_pr = NaN; vp_time = NaN;
    m = regexp(txt, '最终视点数量:\s*(\d+)', 'tokens');
    if ~isempty(m), vp_count = str2double(m{1}{1}); end
    m = regexp(txt, '总覆盖率:\s*([\d\.]+)%', 'tokens');
    if ~isempty(m), vp_cov_pr = str2double(m{1}{1}); end
    m = regexp(txt, '处理时间:\s*([\d\.]+)', 'tokens');
    if ~isempty(m), vp_time = str2double(m{1}{1}); end
    
    % ---- 读取summary_viewpoint_*.txt ----
    psf = dir(fullfile(sp, 'summary_viewpoint_*.txt'));
    path_cov_pr = NaN; path_time = NaN;
    if ~isempty(psf)
        txt2 = fileread(fullfile(sp, psf(1).name));
        m = regexp(txt2, '路径覆盖率:\s*([\d\.]+)', 'tokens');
        if ~isempty(m), path_cov_pr = str2double(m{1}{1}); end
        m = regexp(txt2, '路径规划时间:\s*([\d\.]+)', 'tokens');
        if ~isempty(m), path_time = str2double(m{1}{1}); end
    end
    
    % ---- 读取path_viewpoint_*.csv 计算路径指标 ----
    csvf = dir(fullfile(sp, 'path_viewpoint_*.csv'));
    path_len = NaN; total_turn = NaN; npts = NaN;
    
    if ~isempty(csvf)
        try
            raw = readmatrix(fullfile(sp, csvf(1).name));
            if size(raw,2) >= 6
                pos = raw(:,1:3); dir0 = raw(:,4:6); npts = size(pos,1);
                % 路径长度
                dxyz = diff(pos,1,1);
                seg_len = sqrt(sum(dxyz.^2,2));
                path_len = sum(seg_len);
                % 转弯角度
                nd = vecnorm(dir0,2,2); nd(nd<1e-12)=1;
                ndir = dir0 ./ nd;
                ta = zeros(npts-1,1);
                for k = 1:npts-1
                    dp = dot(ndir(k,:), ndir(k+1,:));
                    dp = max(min(dp,1),-1);
                    ta(k) = acosd(dp);
                end
                ta(isnan(ta)) = 0;
                total_turn = sum(ta);
            end
        catch
        end
    end
    
    % ---- 读取收敛历史数据 ----
    conv_file = fullfile(sp, 'convergence_history.mat');
    if exist(conv_file, 'file')
        try
            cdata = load(conv_file);
            conv_data{end+1} = struct(...
                'threshold', threshold, ...
                'iter', cdata.iteration_history.iter, ...
                'coverage', cdata.iteration_history.coverage);
        catch
        end
    end
    
    % ---- 记录 ----
    valid_idx = valid_idx + 1;
    results(valid_idx).coverage_threshold   = threshold;
    results(valid_idx).viewpoint_coverage   = vp_cov_pr;
    results(valid_idx).path_coverage        = path_cov_pr;
    results(valid_idx).num_viewpoints       = vp_count;
    results(valid_idx).viewpoint_time       = vp_time;
    results(valid_idx).path_time            = path_time;
    results(valid_idx).total_time           = sum([vp_time, path_time],'omitnan');
    results(valid_idx).path_length          = path_len;
    results(valid_idx).total_turn_angle     = total_turn;
    results(valid_idx).num_path_points      = npts;
    
    fprintf('  %s: vp_cov=%.1f%%, path_cov=%.1f%%, len=%.1fm, turn=%.1f°\n',...
        sf.name, vp_cov_pr, path_cov_pr, path_len, total_turn);
end

fprintf('有效数据: %d 组\n', valid_idx);
if valid_idx < 2, error('有效数据不足!'); end

%% ========== [3] 数据向量化 ==========
R = results;
xv = [R.coverage_threshold]*100; % 阈值 (%) 90..100
vc = [R.viewpoint_coverage];
pc = [R.path_coverage];
nv = [R.num_viewpoints];
pl = [R.path_length];
tt = [R.total_turn_angle];
vt = [R.viewpoint_time];
pt_ = [R.path_time];
tot = [R.total_time];
np = [R.num_path_points];

% X轴显示范围（基于实际数据动态计算）
xMin = min(xv); xMax = max(xv);
xRange = xMax - xMin;
xStep = max(1, ceil(xRange / 8));   % 约6~10个刻度
xTicks = xMin:xStep:xMax;
if xTicks(end) < xMax, xTicks = [xTicks, xMax]; end
xLimLine = [xMin - 1, xMax + 1];   % 折线图用
xLimBar  = [xMin - 2, xMax + 2];   % 柱状图用

% 导出统计表
T = table(xv', vc', pc', nv', pl', tt', vt', pt_', tot',...
    'VariableNames',{'Threshold','ViewpointCov','PathCov','NumVP','PathLen',...
    'TotalTurn','VPTime','PathTime','TotalTime'});
writetable(T, fullfile(OUT_DATA, 'analysis_data.csv'));
fprintf('数据表已保存\n');

%% ========== [4] 统计分析 ==========
fprintf('\n===== 统计分析 =====\n');
fprintf('视点覆盖率: mean=%.2f%%, min=%.2f%%, max=%.2f%%\n', mean(vc), min(vc), max(vc));
fprintf('路径覆盖率: mean=%.2f%%, min=%.2f%%, max=%.2f%%\n', mean(pc), min(pc), max(pc));
fprintf('路径长度: mean=%.1fm, min=%.1fm, max=%.1fm\n', mean(pl), min(pl), max(pl));
fprintf('总转弯角度: mean=%.1f°, min=%.1f°, max=%.1f°\n',...
    mean(tt,'omitnan'), min(tt,[],'omitnan'), max(tt,[],'omitnan'));
fprintf('视点数量: mean=%.1f, min=%d, max=%d\n', mean(nv), min(nv), max(nv));
fprintf('计算时间: mean=%.1fs, min=%.1fs, max=%.1fs\n', mean(tot), min(tot), max(tot));

function r = calcCorr(x,y)
    ok = ~isnan(x) & ~isnan(y);
    xv = x(ok); yv = y(ok);
    if sum(ok) < 3, r = NaN; return; end
    xm = mean(xv); ym = mean(yv);
    num = sum((xv-xm).*(yv-ym));
    den = sqrt(sum((xv-xm).^2)*sum((yv-ym).^2));
    if den==0, r=0; else r=num/den; end
end

r1 = calcCorr(xv, pl);
fprintf('覆盖率阈值 vs 路径长度: r=%.4f\n', r1);
r2 = calcCorr(nv, pl);
fprintf('视点数量 vs 路径长度: r=%.4f\n', r2);
r3 = calcCorr(vc, pc);
fprintf('视点覆盖率 vs 路径覆盖率: r=%.4f\n', r3);
r4 = calcCorr(pl, tot);
fprintf('路径长度 vs 计算时间: r=%.4f\n', r4);

%% ========== [5] 辅助函数 ==========
function y = norm01(x)
    x = double(x); v = x(~isnan(x));
    if isempty(v), y = NaN(size(x)); return; end
    mn = min(v); mx = max(v);
    if mx==mn, y = zeros(size(x)); return; end
    y = (x-mn)/(mx-mn);
end

function pi = findPareto(len, cov)
    n = length(len); pi = [];
    for i=1:n
        isP = true;
        for j=1:n
            if i~=j && len(j)<=len(i)+1e-9 && cov(j)>=cov(i)-1e-9 && ...
               (len(j)<len(i)-1e-9 || cov(j)>cov(i)+1e-9)
                isP = false; break;
            end
        end
        if isP, pi = [pi, i]; end
    end
end

function [C, names] = corrMatrix(T, vars)
    n = length(vars); names = vars;
    C = zeros(n); 
    for i=1:n
        for j=1:n
            a = double(T.(vars{i})); b = double(T.(vars{j}));
            ok = ~isnan(a) & ~isnan(b);
            if sum(ok)>=3, C(i,j) = calcCorr(a,b); else C(i,j)=0; end
        end
    end
end

function setCNFont()
    if ismember('SimSun', listfonts)
        cnF = 'SimSun'; enF = 'Times New Roman';
    elseif ismember('宋体', listfonts)
        cnF = '宋体'; enF = 'Times New Roman';
    else
        cnF = 'SimHei'; enF = 'Arial';
    end
    set(gca, 'FontName', enF, 'FontSize',20);
    ax = gca;
    for lab = {'XLabel','YLabel','ZLabel','Title'}
        hh = get(ax, lab{1});
        if ~isempty(hh)
            s = get(hh, 'String');
            if ischar(s) && any(s>127)
                set(hh, 'FontName', cnF);
            elseif iscell(s)
                hasCN = false;
                for ci = 1:length(s)
                    if ischar(s{ci}) && any(s{ci}>127), hasCN=true; break; end
                end
                if hasCN, set(hh, 'FontName', cnF); end
            end
        end
    end
    lg = findobj(gcf, 'Type', 'legend');
    if ~isempty(lg)
        ls = get(lg, 'String');
        if iscell(ls)
            hasCN = false;
            for ci = 1:length(ls)
                if ischar(ls{ci}) && any(ls{ci}>127), hasCN=true; break; end
            end
            if hasCN, set(lg, 'FontName', cnF); end
        end
    end
    % 学术格式：坐标框线粗、刻度向外、网格虚线淡显
    set(gca, 'LineWidth', 1.2, 'TickDir', 'out');
    set(gca, 'GridAlpha', 0.3, 'GridLineStyle', ':');
    set(gca, 'Layer', 'top');
end

function setENFont()
    set(gca, 'FontName', 'Times New Roman', 'FontSize',20);
    ax = gca;
    for lab = {'XLabel','YLabel','ZLabel','Title'}
        hh = get(ax, lab{1});
        if ~isempty(hh), set(hh, 'FontName', 'Times New Roman'); end
    end
    lg = findobj(gcf, 'Type', 'legend');
    if ~isempty(lg), set(lg, 'FontName', 'Times New Roman'); end
    % 学术格式：坐标框线粗、刻度向外、网格虚线淡显
    set(gca, 'LineWidth', 1.2, 'TickDir', 'out');
    set(gca, 'GridAlpha', 0.3, 'GridLineStyle', ':');
    set(gca, 'Layer', 'top');
end

%% ========== [6] 生成中文图 (19个) ==========
fprintf('\n===== 生成中文图 =====\n');
N_FIGS = 19;

fprintf('(1/%d) Fig_C01 覆盖性能对比\n', N_FIGS);
h1 = figure('Position',[50,50,800,600],'Color','w','Name','C01','NumberTitle','off');
hold on; grid on; box on;
p1a = plot(xv, vc, 'o-','Color',[0,0.447,0.741],'LineWidth',2.5,'MarkerSize',9,'MarkerFaceColor',[0,0.447,0.741]);
p1b = plot(xv, pc, 's-','Color',[0.85,0.325,0.098],'LineWidth',2.5,'MarkerSize',9,'MarkerFaceColor',[0.85,0.325,0.098]);
plot(xlim, xlim, 'k--','LineWidth',1,'HandleVisibility','off');
xlabel('覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
ylabel('覆盖率 (%)','FontSize',20,'FontWeight','bold');
title('视点覆盖率与路径覆盖率对比','FontSize',20,'FontWeight','bold');
legend([p1a,p1b],{'视点覆盖率','路径覆盖率'},'Location','southeast','FontSize',20,'Box','off');
xlim(xLimLine); setCNFont(); saveas(h1,fullfile(OUT_CN,'Fig_C01_Coverage_Comparison.fig'));drawnow;

fprintf('(2/%d) Fig_C02 覆盖率差值\n', N_FIGS);
covDiff = pc - vc;
h2 = figure('Position',[100,100,800,600],'Color','w','Name','C02','NumberTitle','off');
hold on; grid on; box on;
bar(xv, covDiff, 'FaceColor',[0.466,0.674,0.188],'EdgeColor','k','LineWidth',1.5,'BarWidth',0.8);
plot(xlim,[0,0],'k-','LineWidth',1);
xlabel('覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
ylabel('覆盖率差值 (%)','FontSize',20,'FontWeight','bold');
title('路径覆盖率相对视点覆盖率的提升','FontSize',20,'FontWeight','bold');
[maxD,idxD] = max(covDiff);
text(xv(idxD),maxD+0.3,sprintf('最大:%.1f%%',maxD),'HorizontalAlignment','center','FontSize',20,'FontWeight','bold');
xlim(xLimBar); set(gca,'XTick',xTicks); setCNFont(); saveas(h2,fullfile(OUT_CN,'Fig_C02_Coverage_Diff.fig'));drawnow;

fprintf('(3/%d) Fig_C03 覆盖效率\n', N_FIGS);
ve = vc./nv;
h3 = figure('Position',[150,150,800,600],'Color','w','Name','C03','NumberTitle','off');
hold on; grid on; box on;
yyaxis left;
p3a = plot(xv, vc, 'o-','Color',[0,0.447,0.741],'LineWidth',2.5,'MarkerSize',8,'MarkerFaceColor',[0,0.447,0.741]);
ylabel('视点覆盖率 (%)','FontSize',20,'FontWeight','bold');
yyaxis right;
p3b = plot(xv, ve, 's-','Color',[0.494,0.184,0.556],'LineWidth',2.5,'MarkerSize',8,'MarkerFaceColor',[0.494,0.184,0.556]);
ylabel('覆盖效率 (%/个)','FontSize',20,'FontWeight','bold');
xlabel('覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
title('覆盖效率分析','FontSize',20,'FontWeight','bold');
legend('视点覆盖率','覆盖效率','Location','northwest','FontSize',20,'Box','off');
xlim(xLimLine); setCNFont(); saveas(h3,fullfile(OUT_CN,'Fig_C03_Coverage_Efficiency.fig'));drawnow;

fprintf('(4/%d) Fig_C04 增量成本\n', N_FIGS);
dx_ = diff(xv); dp_ = diff(pl);
if any(dx_~=0)
    cost = dp_./dx_;
    h4 = figure('Position',[50,200,800,600],'Color','w','Name','C04','NumberTitle','off');
    hold on; grid on; box on;
    bar(xv(2:end), cost, 'FaceColor',[0.929,0.694,0.125],'EdgeColor','k','LineWidth',1.5,'BarWidth',0.6);
    mc = mean(cost);
    plot(xlim,[mc,mc],'r--','LineWidth',2);
    text(max(xlim)*0.95,mc*1.05,sprintf('平均:%.1f',mc),'HorizontalAlignment','right','FontSize',20,'FontWeight','bold','Color','r');
    xlabel('覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
    ylabel('路径长度增量 (m/1%)','FontSize',20,'FontWeight','bold');
    title('覆盖率增量成本分析','FontSize',20,'FontWeight','bold');
    xlim(xLimBar); set(gca,'XTick',xTicks); setCNFont(); saveas(h4,fullfile(OUT_CN,'Fig_C04_Incremental_Cost.fig'));
end
drawnow;

fprintf('(5/%d) Fig_C05 视点数量与路径长度\n', N_FIGS);
h5 = figure('Position',[50,50,800,600],'Color','w','Name','C05','NumberTitle','off');
hold on; grid on; box on;
yyaxis left;
bar(xv, nv, 'FaceColor',[0,0.447,0.741],'EdgeColor','k','FaceAlpha',0.7,'BarWidth',0.6);
ylabel('视点数量 (个)','FontSize',20,'FontWeight','bold');
yyaxis right;
p5 = plot(xv, pl, 'o-','Color',[0.85,0.325,0.098],'LineWidth',2.5,'MarkerSize',8,'MarkerFaceColor',[0.85,0.325,0.098]);
ylabel('路径长度 (m)','FontSize',20,'FontWeight','bold');
xlabel('覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
title('视点数量与路径长度对比','FontSize',20,'FontWeight','bold');
legend('视点数量','路径长度','Location','northwest','FontSize',20,'Box','off');
xlim(xLimBar); set(gca,'XTick',xTicks); setCNFont(); saveas(h5,fullfile(OUT_CN,'Fig_C05_VPvsPathLen.fig'));drawnow;

fprintf('(6/%d) Fig_C06 路径长度与转弯角度\n', N_FIGS);
h6 = figure('Position',[100,100,800,600],'Color','w','Name','C06','NumberTitle','off');
hold on; grid on; box on;
yyaxis left;
bar(xv, pl, 'FaceColor',[0,0.447,0.741],'EdgeColor','k','FaceAlpha',0.7,'BarWidth',0.6);
ylabel('路径长度 (m)','FontSize',20,'FontWeight','bold');
yyaxis right;
p6 = plot(xv, tt, 's-','Color',[0.85,0.325,0.098],'LineWidth',2.5,'MarkerSize',8,'MarkerFaceColor',[0.85,0.325,0.098]);
ylabel('总转弯角度 (度)','FontSize',20,'FontWeight','bold');
xlabel('覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
title('路径长度与总转弯角度','FontSize',20,'FontWeight','bold');
legend('路径长度','总转弯角度','Location','northwest','FontSize',20,'Box','off');
xlim(xLimBar); set(gca,'XTick',xTicks); setCNFont(); saveas(h6,fullfile(OUT_CN,'Fig_C06_Length_TurnAngle.fig'));drawnow;

fprintf('(7/%d) Fig_C07 计算时间分解\n', N_FIGS);
h7 = figure('Position',[100,100,800,600],'Color','w','Name','C07','NumberTitle','off');
hold on; grid on; box on;
b = bar(xv, [vt(:), pt_(:)], 'stacked', 'BarWidth',0.6);
b(1).FaceColor = [0,0.447,0.741]; b(1).EdgeColor = 'k';
b(2).FaceColor = [0.85,0.325,0.098]; b(2).EdgeColor = 'k';
xlabel('覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
ylabel('计算时间 (秒)','FontSize',20,'FontWeight','bold');
title('计算时间分解','FontSize',20,'FontWeight','bold');
legend('视点规划','路径生成','Location','northwest','FontSize',20,'Box','off');
xlim(xLimBar); set(gca,'XTick',xTicks); setCNFont(); saveas(h7,fullfile(OUT_CN,'Fig_C07_Time_Breakdown.fig'));drawnow;

fprintf('(8/%d) Fig_C08 路径长度与计算时间\n', N_FIGS);
h8 = figure('Position',[50,50,800,600],'Color','w','Name','C08','NumberTitle','off');
hold on; grid on; box on;
scatter(pl, tot, 100, xv, 'filled','MarkerEdgeColor','k','LineWidth',1);
colormap(jet); cb = colorbar; ylabel(cb,'覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
v = ~isnan(pl) & ~isnan(tot);
if sum(v) > 2
    p = polyfit(pl(v), tot(v), 1);  % 一次线性拟合
    xf = linspace(min(pl(v)), max(pl(v)), 100);
    plot(xf, polyval(p, xf), 'r--', 'LineWidth', 2);
    yp = polyval(p, pl(v));
    ssr = sum((tot(v)-yp).^2);
    sst = sum((tot(v)-mean(tot(v))).^2);
    r2 = 1 - ssr/sst;
    text(0.05,0.95,sprintf('y=%.4fx%+.2f\n拟合优度 R^2=%.4f',p(1),p(2),r2),...
        'Units','normalized','FontSize',20,'FontWeight','bold','Color','r','BackgroundColor','w');
end
xlabel('路径长度 (m)','FontSize',20,'FontWeight','bold');
ylabel('总计算时间 (秒)','FontSize',20,'FontWeight','bold');
title('路径长度与计算时间关系','FontSize',20,'FontWeight','bold');
setCNFont(); saveas(h8,fullfile(OUT_CN,'Fig_C08_Length_Time.fig'));drawnow;

fprintf('(9/%d) Fig_C09 收敛历史曲线\n', N_FIGS);
h9 = figure('Position',[100,100,800,600],'Color','w','Name','C09','NumberTitle','off');
hold on; grid on; box on;
nConv = length(conv_data);
cmC = jet(min(nConv, 21));
legStr = {};
for ci = 1:min(nConv, 21)
    cd2 = conv_data{ci};
    if isempty(cd2.iter), continue; end
    thrVal = round(cd2.threshold * 100);
    hCov = plot(cd2.iter, cd2.coverage, 'o-', 'Color', cmC(ci,:), ...
        'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', cmC(ci,:));
    legStr{end+1} = sprintf('阈值 %.0f%%', thrVal);
end
if nConv > 21
    fprintf('  收敛历史曲线超过21条，仅显示前21条\n');
end
xlabel('迭代次数','FontSize',20,'FontWeight','bold');
ylabel('覆盖率 (%)','FontSize',20,'FontWeight','bold');
title('各阈值下覆盖率迭代收敛过程','FontSize',20,'FontWeight','bold');
legend(legStr, 'Location', 'southeast', 'FontSize',20, 'Box', 'off', 'NumColumns', 3);
setCNFont(); saveas(h9,fullfile(OUT_CN,'Fig_C09_Convergence_History.fig'));drawnow;

fprintf('(10/%d) Fig_C10 平行坐标图\n', N_FIGS);
indic = {'路径覆盖率',pc; '视点数量',nv; '路径长度',pl; '总转弯角度',tt; '总时间',tot};
ni = size(indic,1); nd = length(xv);
nD = zeros(nd,ni);
for k=1:ni, nD(:,k)=norm01(indic{k,2}); end
h10 = figure('Position',[50,50,1000,600],'Color','w','Name','C10','NumberTitle','off');
hold on; grid on; box on;
cmP = jet(nd);
for i=1:nd
    if all(~isnan(nD(i,:)))
        plot(1:ni, nD(i,:), 'o-','Color',cmP(i,:),'LineWidth',1.5,'MarkerSize',6,'MarkerFaceColor',cmP(i,:));
    end
end
colormap(jet); caxis([min(xv), max(xv)]);
cb = colorbar; ylabel(cb,'覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
set(gca,'XTick',1:ni,'XTickLabel',indic(:,1),'XTickLabelRotation',45,'FontSize',20);
ylabel('归一化值','FontSize',20,'FontWeight','bold'); ylim([0,1.1]);
title('综合性能平行坐标图','FontSize',20,'FontWeight','bold');
setCNFont(); saveas(h10,fullfile(OUT_CN,'Fig_C10_Parallel_Coords.fig'));drawnow;

fprintf('(11/%d) Fig_C11 性能气泡图\n', N_FIGS);
h11 = figure('Position',[100,100,800,600],'Color','w','Name','C11','NumberTitle','off');
hold on; grid on; box on;
scatter(pl, pc, sqrt(tot/max(tot))*100, xv, 'filled','MarkerEdgeColor','k','LineWidth',1);
colormap(jet); cb = colorbar; ylabel(cb,'覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
% 路径覆盖率~路径长度一次线性拟合
v = ~isnan(pl) & ~isnan(pc);
if sum(v) > 3
    p = polyfit(pl(v), pc(v), 1);
    xf = linspace(min(pl(v)), max(pl(v)), 100);
    plot(xf, polyval(p, xf), 'r--', 'LineWidth', 2);
    yp = polyval(p, pl(v));
    ssr = sum((pc(v)-yp).^2);
    sst = sum((pc(v)-mean(pc(v))).^2);
    r2 = 1 - ssr/sst;
    text(0.05,0.05,sprintf('线性拟合 R^2=%.4f',r2),...
        'Units','normalized','FontSize',20,'FontWeight','bold','Color','r','BackgroundColor','w');
end
xlabel('路径长度 (m)','FontSize',20,'FontWeight','bold');
ylabel('路径覆盖率 (%)','FontSize',20,'FontWeight','bold');
title('性能气泡图 (气泡大小=计算时间)','FontSize',20,'FontWeight','bold');
setCNFont(); saveas(h11,fullfile(OUT_CN,'Fig_C11_Bubble_Chart.fig'));drawnow;

fprintf('(12/%d) Fig_C12 综合得分排名\n', N_FIGS);
wt = [0.4,0.3,0.3];
nrm_pc = norm01(pc); nrm_pl = 1-norm01(pl); nrm_tot = 1-norm01(tot);
score = wt(1)*nrm_pc + wt(2)*nrm_pl + wt(3)*nrm_tot;
[ss, si] = sort(score,'descend');
h12 = figure('Position',[50,50,800,600],'Color','w','Name','C12','NumberTitle','off');
hold on; grid on; box on;
bar(xv(si), ss, 'FaceColor',[0,0.447,0.741],'EdgeColor','k','BarWidth',0.6);
xlabel('覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
ylabel('综合得分','FontSize',20,'FontWeight','bold');
title('综合性能得分排名','FontSize',20,'FontWeight','bold');
for k=1:min(3,length(ss))
    text(xv(si(k)), ss(k)+0.02, sprintf('第%d名',k),'HorizontalAlignment','center','FontSize',20,'FontWeight','bold');
end
text(0.02,0.98,sprintf('权重:覆盖率%.1f 长度%.1f 时间%.1f',wt(1),wt(2),wt(3)),...
    'Units','normalized','FontSize',20,'FontWeight','bold','BackgroundColor','w');
setCNFont(); saveas(h12,fullfile(OUT_CN,'Fig_C12_Score_Ranking.fig'));drawnow;

fprintf('(13/%d) Fig_C13 帕累托前沿分析\n', N_FIGS);
pi = findPareto(pl, pc);
h13 = figure('Position',[150,150,800,600],'Color','w','Name','C13','NumberTitle','off');
hold on; grid on; box on;
scatter(pl, pc, 80, xv, 'filled','MarkerEdgeColor','k','LineWidth',1);
scatter(pl(pi), pc(pi), 120, 'r','filled','MarkerEdgeColor','k','LineWidth',2);
if length(pi)>1
    [~,o] = sort(pl(pi));
    plot(pl(pi(o)), pc(pi(o)), 'r--','LineWidth',2);
end
colormap(jet); cb = colorbar; ylabel(cb,'覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
xlabel('路径长度 (m)','FontSize',20,'FontWeight','bold');
ylabel('路径覆盖率 (%)','FontSize',20,'FontWeight','bold');
title('帕累托前沿分析','FontSize',20,'FontWeight','bold');
legend('所有解','帕累托最优解','前沿曲线','Location','southeast','FontSize',20);
setCNFont(); saveas(h13,fullfile(OUT_CN,'Fig_C13_Pareto_Front.fig'));drawnow;

fprintf('(14/%d) Fig_C14 三维关系图\n', N_FIGS);
h14 = figure('Position',[100,100,900,700],'Color','w','Name','C14','NumberTitle','off');
hold on; grid on; box on;
scatter3(xv, pl, nv, 80, xv, 'filled','MarkerEdgeColor','k');
colormap(jet); cb = colorbar; ylabel(cb,'覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
xlabel('覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
ylabel('路径长度 (m)','FontSize',20,'FontWeight','bold');
zlabel('视点数量 (个)','FontSize',20,'FontWeight','bold');
title('覆盖率阈值、路径长度与视点数量的三维关系','FontSize',20,'FontWeight','bold');
view(45,30); setCNFont(); saveas(h14,fullfile(OUT_CN,'Fig_C14_3D_Relation.fig'));drawnow;

fprintf('(15/%d) Fig_C15 相关系数热图\n', N_FIGS);
vars = {'Threshold','ViewpointCov','PathCov','NumVP','PathLen','TotalTurn','TotalTime','VPTime','PathTime'};
cnames = {'阈值','视点覆盖率','路径覆盖率','视点数量','路径长度','总转弯角度','总时间','视点规划时间','路径生成时间'};
[C,~] = corrMatrix(T, vars); nv2 = length(vars);
h15 = figure('Position',[100,100,850,650],'Color','w','Name','C15','NumberTitle','off');
imagesc(C,[-1,1]); colormap(jet); cb = colorbar; ylabel(cb,'相关系数','FontSize',20,'FontWeight','bold');
hold on;
set(gca,'XTick',1:nv2,'XTickLabel',cnames,'YTick',1:nv2,'YTickLabel',cnames,'FontSize',20);
xtickangle(45);
for i=1:nv2
    for j=1:nv2
        text(j,i,sprintf('%.2f',C(i,j)),'HorizontalAlignment','center','FontSize',20,'Color','w','FontWeight','bold');
    end
end
title('路径规划指标相关系数矩阵','FontSize',20,'FontWeight','bold');
setCNFont(); saveas(h15,fullfile(OUT_CN,'Fig_C15_Correlation_Heatmap.fig'));drawnow;

fprintf('(16/%d) Fig_C16 最优方案推荐\n', N_FIGS);
[~, bi] = max(score);
h16 = figure('Position',[50,50,600,800],'Color','w','Name','C16','NumberTitle','off');
rt = {sprintf('======= 最优方案推荐 ======='); sprintf('最优覆盖率阈值: %.0f%%',xv(bi));
    sprintf('综合得分: %.3f',score(bi)); ''; '======= 性能指标 =======';
    sprintf('路径覆盖率: %.1f%%',pc(bi)); sprintf('视点数量: %d个',nv(bi));
    sprintf('路径长度: %.1fm',pl(bi)); sprintf('总转弯角度: %.1f°',tt(bi));
    sprintf('计算时间: %.1fs',tot(bi)); ''; '======= 效率指标 =======';
    sprintf('单位覆盖率路径: %.2fm/1%%',pl(bi)/pc(bi));
    sprintf('单位覆盖率时间: %.2fs/1%%',tot(bi)/pc(bi)); '';
    '======= 推荐理由 ======='; '在覆盖率、路径长度和计算时间之间'; '取得了最佳平衡'};
text(0.05,0.95,rt,'VerticalAlignment','top','FontSize',20,'FontWeight','bold','Interpreter','none','BackgroundColor',[1,1,0.9]);
rectangle('Position',[0.02,0.02,0.96,0.96],'EdgeColor','r','LineWidth',2); axis off;
setCNFont(); saveas(h16,fullfile(OUT_CN,'Fig_C16_Optimal_Solution.fig'));drawnow;
fprintf('中文图完成! (%d个)\n', N_FIGS);

fprintf('(17/%d) Fig_C17 视点数量随覆盖率阈值变化\n', N_FIGS);
h17 = figure('Position',[50,50,800,600],'Color','w','Name','C17','NumberTitle','off');
hold on; grid on; box on;
yyaxis left;
hb17 = bar(xv, nv, 'FaceColor',[0,0.447,0.741],'EdgeColor','k','FaceAlpha',0.6,'BarWidth',0.7);
ylabel('视点数量 (个)','FontSize',20,'FontWeight','bold','Color',[0,0.447,0.741]);
yyaxis right;
hp17 = plot(xv, nv, 'o-','Color',[0.85,0.325,0.098],'LineWidth',2.5,'MarkerSize',9,'MarkerFaceColor',[0.85,0.325,0.098]);
ylabel('视点数量 (个)','FontSize',20,'FontWeight','bold','Color',[0.85,0.325,0.098]);
xlabel('覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
title('视点数量随覆盖率阈值的变化','FontSize',20,'FontWeight','bold');
legend([hb17,hp17],{'各阈值视点数量','变化趋势'},'Location','northwest','FontSize',20,'Box','off');
% 线性拟合
v17 = ~isnan(xv) & ~isnan(nv);
if sum(v17) > 3
    p17 = polyfit(xv(v17), nv(v17), 1);
    xf17 = linspace(min(xv(v17)), max(xv(v17)), 100);
    plot(xf17, polyval(p17, xf17), '--','Color',[0.5,0.5,0.5],'LineWidth',1.5);
    yp17 = polyval(p17, xv(v17));
    ssr17 = sum((nv(v17)-yp17).^2);
    sst17 = sum((nv(v17)-mean(nv(v17))).^2);
    r2_17 = 1 - ssr17/sst17;
    text(0.05,0.95,sprintf('线性拟合 R^2=%.4f',r2_17),'Units','normalized','FontSize',20,'FontWeight','bold',...
        'BackgroundColor','w','EdgeColor','k');
end
xlim(xLimBar); set(gca,'XTick',xTicks);
setCNFont(); saveas(h17,fullfile(OUT_CN,'Fig_C17_VPvsThreshold.fig'));drawnow;

fprintf('(18/%d) Fig_C18 视点数量与路径长度的关系\n', N_FIGS);
h18 = figure('Position',[100,100,800,600],'Color','w','Name','C18','NumberTitle','off');
hold on; grid on; box on;
scatter(pl, nv, 100, xv, 'filled','MarkerEdgeColor','k','LineWidth',1);
colormap(jet); cb18 = colorbar; ylabel(cb18,'覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
v18 = ~isnan(pl) & ~isnan(nv);
if sum(v18) > 2
    p18 = polyfit(pl(v18), nv(v18), 1);
    xf18 = linspace(min(pl(v18)), max(pl(v18)), 100);
    plot(xf18, polyval(p18, xf18), 'r--','LineWidth',2);
    yp18 = polyval(p18, pl(v18));
    r_pl_nv = calcCorr(pl(v18), nv(v18));
    text(0.05,0.95,sprintf('线性拟合: y=%.2fx%+.1f\n相关系数 r=%.4f',p18(1),p18(2),r_pl_nv),...
        'Units','normalized','FontSize',20,'FontWeight','bold','Color','r','BackgroundColor','w');
end
xlabel('路径长度 (m)','FontSize',20,'FontWeight','bold');
ylabel('视点数量 (个)','FontSize',20,'FontWeight','bold');
title('不同覆盖率阈值下视点数量与路径长度关系','FontSize',20,'FontWeight','bold');
setCNFont(); saveas(h18,fullfile(OUT_CN,'Fig_C18_VPvsPathLen.fig'));drawnow;

fprintf('(19/%d) Fig_C19 覆盖率与代价的帕累托前沿\n', N_FIGS);
% 定义综合代价 = 归一化路径长度 + 归一化转弯角度
cost19 = norm01(pl) + norm01(tt);
pi19 = findPareto(cost19, pc);
h19 = figure('Position',[150,150,800,600],'Color','w','Name','C19','NumberTitle','off');
hold on; grid on; box on;
scatter(cost19, pc, 100, xv, 'filled','MarkerEdgeColor','k','LineWidth',1.2);
colormap(jet); cb19 = colorbar; ylabel(cb19,'覆盖率阈值 (%)','FontSize',20,'FontWeight','bold');
if ~isempty(pi19)
    scatter(cost19(pi19), pc(pi19), 150, 'r','filled','MarkerEdgeColor','k','LineWidth',2);
    if length(pi19) > 1
        [~, o19] = sort(cost19(pi19));
        plot(cost19(pi19(o19)), pc(pi19(o19)), 'r-','LineWidth',2.5);
    end
    % 标注帕累托最优解
    for k19 = 1:length(pi19)
        text(cost19(pi19(k19)), pc(pi19(k19))+0.3, sprintf('%.0f%%',xv(pi19(k19))),...
            'HorizontalAlignment','center','FontSize',20,'FontWeight','bold','Color','r');
    end
end
xlabel('综合代价 (归一化路径长度+归一化转弯角度)','FontSize',20,'FontWeight','bold');
ylabel('路径覆盖率 (%)','FontSize',20,'FontWeight','bold');
title('覆盖率与综合代价的帕累托前沿分析','FontSize',20,'FontWeight','bold');
legend('所有方案','帕累托最优解','前沿曲线','Location','southeast','FontSize',20,'Box','off');
setCNFont(); saveas(h19,fullfile(OUT_CN,'Fig_C19_Pareto_Coverage_vs_Cost.fig'));drawnow;
fprintf('中文图完成! (%d个)\n', N_FIGS);

%% ========== [7] 生成英文图 (19个) ==========
fprintf('\n===== 生成英文图 =====\n');

fprintf('(1/%d) Fig_E01 Coverage Comparison\n', N_FIGS);
he1 = figure('Position',[50,50,800,600],'Color','w','Name','E01','NumberTitle','off');
hold on; grid on; box on;
pe1 = plot(xv, vc, 'o-','Color',[0,0.447,0.741],'LineWidth',2.5,'MarkerSize',9,'MarkerFaceColor',[0,0.447,0.741]);
pe2 = plot(xv, pc, 's-','Color',[0.85,0.325,0.098],'LineWidth',2.5,'MarkerSize',9,'MarkerFaceColor',[0.85,0.325,0.098]);
plot(xlim, xlim, 'k--','LineWidth',1,'HandleVisibility','off');
xlabel('Coverage threshold (%)','FontSize',20,'FontWeight','bold');
ylabel('Coverage (%)','FontSize',20,'FontWeight','bold');
title('Viewpoint vs. Path Coverage','FontSize',20,'FontWeight','bold');
legend([pe1,pe2],{'Viewpoint coverage','Path coverage'},'Location','southeast','FontSize',20,'Box','off');
xlim(xLimLine); setENFont(); saveas(he1,fullfile(OUT_EN,'Fig_E01_Coverage_Comparison.fig'));drawnow;

fprintf('(2/%d) Fig_E02 Coverage Improvement\n', N_FIGS);
he2 = figure('Position',[100,100,800,600],'Color','w','Name','E02','NumberTitle','off');
hold on; grid on; box on;
bar(xv, covDiff, 'FaceColor',[0.466,0.674,0.188],'EdgeColor','k','LineWidth',1.5,'BarWidth',0.8);
plot(xlim,[0,0],'k-','LineWidth',1);
xlabel('Coverage threshold (%)','FontSize',20,'FontWeight','bold');
ylabel('Coverage improvement (%)','FontSize',20,'FontWeight','bold');
title('Path Coverage Improvement','FontSize',20,'FontWeight','bold');
text(xv(idxD),maxD+0.3,sprintf('Max:%.1f%%',maxD),'HorizontalAlignment','center','FontSize',20,'FontWeight','bold');
xlim(xLimBar); set(gca,'XTick',xTicks); setENFont(); saveas(he2,fullfile(OUT_EN,'Fig_E02_Coverage_Improvement.fig'));drawnow;

fprintf('(3/%d) Fig_E03 Coverage Efficiency\n', N_FIGS);
he3 = figure('Position',[150,150,800,600],'Color','w','Name','E03','NumberTitle','off');
hold on; grid on; box on;
yyaxis left;
plot(xv, vc, 'o-','Color',[0,0.447,0.741],'LineWidth',2.5,'MarkerSize',8,'MarkerFaceColor',[0,0.447,0.741]);
ylabel('Viewpoint coverage (%)','FontSize',20,'FontWeight','bold');
yyaxis right;
plot(xv, ve, 's-','Color',[0.494,0.184,0.556],'LineWidth',2.5,'MarkerSize',8,'MarkerFaceColor',[0.494,0.184,0.556]);
ylabel('Efficiency (%/viewpoint)','FontSize',20,'FontWeight','bold');
xlabel('Coverage threshold (%)','FontSize',20,'FontWeight','bold');
title('Coverage Efficiency Analysis','FontSize',20,'FontWeight','bold');
legend('Viewpoint coverage','Efficiency','Location','northwest','FontSize',20,'Box','off');
xlim(xLimLine); setENFont(); saveas(he3,fullfile(OUT_EN,'Fig_E03_Coverage_Efficiency.fig'));drawnow;

fprintf('(4/%d) Fig_E04 Incremental Cost\n', N_FIGS);
if any(dx_~=0)
    he4 = figure('Position',[50,200,800,600],'Color','w','Name','E04','NumberTitle','off');
    hold on; grid on; box on;
    bar(xv(2:end), cost, 'FaceColor',[0.929,0.694,0.125],'EdgeColor','k','LineWidth',1.5,'BarWidth',0.6);
    plot(xlim,[mc,mc],'r--','LineWidth',2);
    text(max(xlim)*0.95,mc*1.05,sprintf('Mean:%.1f',mc),'HorizontalAlignment','right','FontSize',20,'FontWeight','bold','Color','r');
    xlabel('Coverage threshold (%)','FontSize',20,'FontWeight','bold');
    ylabel('Path length increment (m/1%)','FontSize',20,'FontWeight','bold');
    title('Incremental Cost Analysis','FontSize',20,'FontWeight','bold');
    xlim(xLimBar); set(gca,'XTick',xTicks); setENFont(); saveas(he4,fullfile(OUT_EN,'Fig_E04_Incremental_Cost.fig'));
end
drawnow;

fprintf('(5/%d) Fig_E05 Viewpoints vs Path Length\n', N_FIGS);
he5 = figure('Position',[50,50,800,600],'Color','w','Name','E05','NumberTitle','off');
hold on; grid on; box on;
yyaxis left;
bar(xv, nv, 'FaceColor',[0,0.447,0.741],'EdgeColor','k','FaceAlpha',0.7,'BarWidth',0.6);
ylabel('Number of viewpoints','FontSize',20,'FontWeight','bold');
yyaxis right;
plot(xv, pl, 'o-','Color',[0.85,0.325,0.098],'LineWidth',2.5,'MarkerSize',8,'MarkerFaceColor',[0.85,0.325,0.098]);
ylabel('Path length (m)','FontSize',20,'FontWeight','bold');
xlabel('Coverage threshold (%)','FontSize',20,'FontWeight','bold');
title('Viewpoints and Path Length','FontSize',20,'FontWeight','bold');
legend('Viewpoints','Path length','Location','northwest','FontSize',20,'Box','off');
xlim(xLimBar); set(gca,'XTick',xTicks); setENFont(); saveas(he5,fullfile(OUT_EN,'Fig_E05_VPvsPathLen.fig'));drawnow;

fprintf('(6/%d) Fig_E06 Path Length vs Turn Angle\n', N_FIGS);
he6 = figure('Position',[100,100,800,600],'Color','w','Name','E06','NumberTitle','off');
hold on; grid on; box on;
yyaxis left;
bar(xv, pl, 'FaceColor',[0,0.447,0.741],'EdgeColor','k','FaceAlpha',0.7,'BarWidth',0.6);
ylabel('Path length (m)','FontSize',20,'FontWeight','bold');
yyaxis right;
plot(xv, tt, 's-','Color',[0.85,0.325,0.098],'LineWidth',2.5,'MarkerSize',8,'MarkerFaceColor',[0.85,0.325,0.098]);
ylabel('Total turn angle (deg)','FontSize',20,'FontWeight','bold');
xlabel('Coverage threshold (%)','FontSize',20,'FontWeight','bold');
title('Path Length and Turn Angle','FontSize',20,'FontWeight','bold');
legend('Path length','Total turn angle','Location','northwest','FontSize',20,'Box','off');
xlim(xLimBar); set(gca,'XTick',xTicks); setENFont(); saveas(he6,fullfile(OUT_EN,'Fig_E06_Length_TurnAngle.fig'));drawnow;

fprintf('(7/%d) Fig_E07 Time Breakdown\n', N_FIGS);
he7 = figure('Position',[100,100,800,600],'Color','w','Name','E07','NumberTitle','off');
hold on; grid on; box on;
b = bar(xv, [vt(:), pt_(:)], 'stacked', 'BarWidth',0.6);
b(1).FaceColor = [0,0.447,0.741]; b(1).EdgeColor = 'k';
b(2).FaceColor = [0.85,0.325,0.098]; b(2).EdgeColor = 'k';
xlabel('Coverage threshold (%)','FontSize',20,'FontWeight','bold');
ylabel('Time (s)','FontSize',20,'FontWeight','bold');
title('Computation Time Breakdown','FontSize',20,'FontWeight','bold');
legend('Viewpoint planning','Path generation','Location','northwest','FontSize',20,'Box','off');
xlim(xLimBar); set(gca,'XTick',xTicks); setENFont(); saveas(he7,fullfile(OUT_EN,'Fig_E07_Time_Breakdown.fig'));drawnow;

fprintf('(8/%d) Fig_E08 Path Length vs Time\n', N_FIGS);
he8 = figure('Position',[50,50,800,600],'Color','w','Name','E08','NumberTitle','off');
hold on; grid on; box on;
scatter(pl, tot, 100, xv, 'filled','MarkerEdgeColor','k','LineWidth',1);
colormap(jet); cb = colorbar; ylabel(cb,'Coverage threshold (%)','FontSize',20,'FontWeight','bold');
v = ~isnan(pl) & ~isnan(tot);
if sum(v) > 2
    p = polyfit(pl(v), tot(v), 1);
    xf = linspace(min(pl(v)), max(pl(v)), 100);
    plot(xf, polyval(p, xf), 'r--', 'LineWidth', 2);
    yp = polyval(p, pl(v));
    ssr = sum((tot(v)-yp).^2);
    sst = sum((tot(v)-mean(tot(v))).^2);
    r2 = 1 - ssr/sst;
    text(0.05,0.95,sprintf('y=%.4fx%+.2f\nR^2=%.4f',p(1),p(2),r2),...
        'Units','normalized','FontSize',20,'FontWeight','bold','Color','r','BackgroundColor','w');
end
xlabel('Path length (m)','FontSize',20,'FontWeight','bold');
ylabel('Total time (s)','FontSize',20,'FontWeight','bold');
title('Computation Time vs. Path Length','FontSize',20,'FontWeight','bold');
setENFont(); saveas(he8,fullfile(OUT_EN,'Fig_E08_Length_Time.fig'));drawnow;

fprintf('(9/%d) Fig_E09 Convergence History\n', N_FIGS);
he9 = figure('Position',[100,100,800,600],'Color','w','Name','E09','NumberTitle','off');
hold on; grid on; box on;
nConv = length(conv_data);
cmE = jet(min(nConv, 21));
legStrE = {};
for ci = 1:min(nConv, 21)
    cd2 = conv_data{ci};
    if isempty(cd2.iter), continue; end
    plot(cd2.iter, cd2.coverage, 'o-', 'Color', cmE(ci,:), ...
        'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', cmE(ci,:));
    thrVal = round(cd2.threshold * 100);
    legStrE{end+1} = sprintf('%.0f%%', thrVal);
end
if nConv > 21
    fprintf('  Convergence history exceeds 21 curves, showing first 21\n');
end
xlabel('Iteration','FontSize',20,'FontWeight','bold');
ylabel('Coverage (%)','FontSize',20,'FontWeight','bold');
title('Convergence History by Threshold','FontSize',20,'FontWeight','bold');
legend(legStrE, 'Location', 'southeast', 'FontSize',20, 'Box', 'off', 'NumColumns', 3);
setENFont(); saveas(he9,fullfile(OUT_EN,'Fig_E09_Convergence_History.fig'));drawnow;

fprintf('(10/%d) Fig_E10 Parallel Coordinates\n', N_FIGS);
ind_en = {'Coverage',pc; 'Viewpoints',nv; 'PathLen',pl; 'TurnAng',tt; 'Time',tot};
ni = size(ind_en,1); nD2 = zeros(nd,ni);
for k=1:ni, nD2(:,k)=norm01(ind_en{k,2}); end
he10 = figure('Position',[50,50,1000,600],'Color','w','Name','E10','NumberTitle','off');
hold on; grid on; box on;
cmP2 = jet(nd);
for i=1:nd
    if all(~isnan(nD2(i,:)))
        plot(1:ni, nD2(i,:), 'o-','Color',cmP2(i,:),'LineWidth',1.5,'MarkerSize',6,'MarkerFaceColor',cmP2(i,:));
    end
end
colormap(jet); caxis([min(xv), max(xv)]);
cb = colorbar; ylabel(cb,'Coverage threshold (%)','FontSize',20,'FontWeight','bold');
set(gca,'XTick',1:ni,'XTickLabel',{'Coverage','Viewpoints','PathLen','TurnAng','Time'},'XTickLabelRotation',45,'FontSize',20);
ylabel('Normalized value','FontSize',20,'FontWeight','bold'); ylim([0,1.1]);
title('Parallel Coordinates of Performance Metrics','FontSize',20,'FontWeight','bold');
setENFont(); saveas(he10,fullfile(OUT_EN,'Fig_E10_Parallel_Coords.fig'));drawnow;

fprintf('(11/%d) Fig_E11 Bubble Chart\n', N_FIGS);
he11 = figure('Position',[100,100,800,600],'Color','w','Name','E11','NumberTitle','off');
hold on; grid on; box on;
scatter(pl, pc, sqrt(tot/max(tot))*100, xv, 'filled','MarkerEdgeColor','k','LineWidth',1);
colormap(jet); cb = colorbar; ylabel(cb,'Coverage threshold (%)','FontSize',20,'FontWeight','bold');
% Linear fit: path coverage ~ path length
v = ~isnan(pl) & ~isnan(pc);
if sum(v) > 3
    p = polyfit(pl(v), pc(v), 1);
    xf = linspace(min(pl(v)), max(pl(v)), 100);
    plot(xf, polyval(p, xf), 'r--', 'LineWidth', 2);
    yp = polyval(p, pl(v));
    ssr = sum((pc(v)-yp).^2);
    sst = sum((pc(v)-mean(pc(v))).^2);
    r2 = 1 - ssr/sst;
    text(0.05,0.05,sprintf('Linear fit R^2=%.4f',r2),...
        'Units','normalized','FontSize',20,'FontWeight','bold','Color','r','BackgroundColor','w');
end
xlabel('Path length (m)','FontSize',20,'FontWeight','bold');
ylabel('Path coverage (%)','FontSize',20,'FontWeight','bold');
title('Performance Bubble Chart (Size=Time)','FontSize',20,'FontWeight','bold');
setENFont(); saveas(he11,fullfile(OUT_EN,'Fig_E11_Bubble_Chart.fig'));drawnow;

fprintf('(12/%d) Fig_E12 Score Ranking\n', N_FIGS);
he12 = figure('Position',[50,50,800,600],'Color','w','Name','E12','NumberTitle','off');
hold on; grid on; box on;
bar(xv(si), ss, 'FaceColor',[0,0.447,0.741],'EdgeColor','k','BarWidth',0.6);
xlabel('Coverage threshold (%)','FontSize',20,'FontWeight','bold');
ylabel('Composite score','FontSize',20,'FontWeight','bold');
title('Composite Performance Ranking','FontSize',20,'FontWeight','bold');
text(0.05,0.95,sprintf('Weights: Cov=%.1f, Len=%.1f, Time=%.1f',wt(1),wt(2),wt(3)),...
    'Units','normalized','FontSize',20,'FontWeight','bold','BackgroundColor','w');
setENFont(); saveas(he12,fullfile(OUT_EN,'Fig_E12_Score_Ranking.fig'));drawnow;

fprintf('(13/%d) Fig_E13 Pareto Front\n', N_FIGS);
he13 = figure('Position',[150,150,800,600],'Color','w','Name','E13','NumberTitle','off');
hold on; grid on; box on;
scatter(pl, pc, 80, xv, 'filled','MarkerEdgeColor','k','LineWidth',1);
scatter(pl(pi), pc(pi), 120, 'r','filled','MarkerEdgeColor','k','LineWidth',2);
if length(pi)>1, [~,o]=sort(pl(pi)); plot(pl(pi(o)),pc(pi(o)),'r--','LineWidth',2); end
colormap(jet); cb = colorbar; ylabel(cb,'Coverage threshold (%)','FontSize',20,'FontWeight','bold');
xlabel('Path length (m)','FontSize',20,'FontWeight','bold');
ylabel('Path coverage (%)','FontSize',20,'FontWeight','bold');
title('Pareto Front Analysis','FontSize',20,'FontWeight','bold');
legend('All solutions','Pareto optimal','Pareto front','Location','southeast','FontSize',20);
setENFont(); saveas(he13,fullfile(OUT_EN,'Fig_E13_Pareto_Front.fig'));drawnow;

fprintf('(14/%d) Fig_E14 3D Relationship\n', N_FIGS);
he14 = figure('Position',[100,100,900,700],'Color','w','Name','E14','NumberTitle','off');
hold on; grid on; box on;
scatter3(xv, pl, nv, 80, xv, 'filled','MarkerEdgeColor','k');
colormap(jet); cb = colorbar; ylabel(cb,'Coverage threshold (%)','FontSize',20,'FontWeight','bold');
xlabel('Coverage threshold (%)','FontSize',20,'FontWeight','bold');
ylabel('Path length (m)','FontSize',20,'FontWeight','bold');
zlabel('Number of viewpoints','FontSize',20,'FontWeight','bold');
title('3D: Coverage, Path Length and Viewpoints','FontSize',20,'FontWeight','bold');
view(45,30); setENFont(); saveas(he14,fullfile(OUT_EN,'Fig_E14_3D_Relation.fig'));drawnow;

fprintf('(15/%d) Fig_E15 Correlation Heatmap\n', N_FIGS);
cnames_en = {'Thrs','VP_Cov','Path_Cov','NumVP','PathLen','TurnAng','Time','VP_Time','Path_Time'};
he15 = figure('Position',[100,100,850,650],'Color','w','Name','E15','NumberTitle','off');
imagesc(C,[-1,1]); colormap(jet); cb = colorbar; ylabel(cb,'Correlation coefficient','FontSize',20,'FontWeight','bold');
hold on;
set(gca,'XTick',1:nv2,'XTickLabel',cnames_en,'YTick',1:nv2,'YTickLabel',cnames_en,'FontSize',20);
xtickangle(45);
for i=1:nv2
    for j=1:nv2
        text(j,i,sprintf('%.2f',C(i,j)),'HorizontalAlignment','center','FontSize',20,'Color','w','FontWeight','bold');
    end
end
title('Correlation Matrix of Path Planning Metrics','FontSize',20,'FontWeight','bold');
setENFont(); saveas(he15,fullfile(OUT_EN,'Fig_E15_Correlation_Heatmap.fig'));drawnow;

fprintf('(16/%d) Fig_E16 Optimal Solution\n', N_FIGS);
he16 = figure('Position',[50,50,600,800],'Color','w','Name','E16','NumberTitle','off');
rt_en = {sprintf('======= Optimal Solution ======='); sprintf('Optimal threshold: %.0f%%',xv(bi));
    sprintf('Composite score: %.3f',score(bi)); ''; '======= Performance =======';
    sprintf('Path coverage: %.1f%%',pc(bi)); sprintf('Viewpoints: %d',nv(bi));
    sprintf('Path length: %.1f m',pl(bi)); sprintf('Total turn: %.1f deg',tt(bi));
    sprintf('Computation time: %.1f s',tot(bi)); ''; '======= Efficiency =======';
    sprintf('Path length/coverage: %.2f m/1%%',pl(bi)/pc(bi));
    sprintf('Time/coverage: %.2f s/1%%',tot(bi)/pc(bi)); '';
    '======= Rationale ======='; 'Best balance among coverage,'; 'path length, and computation time.'};
text(0.05,0.95,rt_en,'VerticalAlignment','top','FontSize',20,'FontWeight','bold','Interpreter','none','BackgroundColor',[1,1,0.9]);
rectangle('Position',[0.02,0.02,0.96,0.96],'EdgeColor','r','LineWidth',2); axis off;
setENFont(); saveas(he16,fullfile(OUT_EN,'Fig_E16_Optimal_Solution.fig'));drawnow;

% --- 增补图 E17~E19 ---
fprintf('(17/%d) Fig_E17 Viewpoints vs. Coverage Threshold\n', N_FIGS);
he17 = figure('Position',[50,50,800,600],'Color','w','Name','E17','NumberTitle','off');
hold on; grid on; box on;
yyaxis left;
heb17 = bar(xv, nv, 'FaceColor',[0,0.447,0.741],'EdgeColor','k','FaceAlpha',0.6,'BarWidth',0.7);
ylabel('Number of viewpoints','FontSize',20,'FontWeight','bold','Color',[0,0.447,0.741]);
yyaxis right;
hep17 = plot(xv, nv, 'o-','Color',[0.85,0.325,0.098],'LineWidth',2.5,'MarkerSize',9,'MarkerFaceColor',[0.85,0.325,0.098]);
ylabel('Number of viewpoints','FontSize',20,'FontWeight','bold','Color',[0.85,0.325,0.098]);
xlabel('Coverage threshold (%)','FontSize',20,'FontWeight','bold');
title('Variation in the Number of Viewpoints\nwith Respect to the Coverage Threshold','FontSize',20,'FontWeight','bold');
legend([heb17,hep17],{'Viewpoints per threshold','Trend'},'Location','northwest','FontSize',20,'Box','off');
% Linear fit
v17e = ~isnan(xv) & ~isnan(nv);
if sum(v17e) > 3
    p17e = polyfit(xv(v17e), nv(v17e), 1);
    xf17e = linspace(min(xv(v17e)), max(xv(v17e)), 100);
    plot(xf17e, polyval(p17e, xf17e), '--','Color',[0.5,0.5,0.5],'LineWidth',1.5);
    yp17e = polyval(p17e, xv(v17e));
    ssr17e = sum((nv(v17e)-yp17e).^2);
    sst17e = sum((nv(v17e)-mean(nv(v17e))).^2);
    r2_17e = 1 - ssr17e/sst17e;
    text(0.05,0.95,sprintf('Linear fit R^2=%.4f',r2_17e),'Units','normalized','FontSize',20,'FontWeight','bold',...
        'BackgroundColor','w','EdgeColor','k');
end
xlim(xLimBar); set(gca,'XTick',xTicks);
setENFont(); saveas(he17,fullfile(OUT_EN,'Fig_E17_VPvsThreshold.fig'));drawnow;

fprintf('(18/%d) Fig_E18 Viewpoints vs. Path Length\n', N_FIGS);
he18 = figure('Position',[100,100,800,600],'Color','w','Name','E18','NumberTitle','off');
hold on; grid on; box on;
scatter(pl, nv, 100, xv, 'filled','MarkerEdgeColor','k','LineWidth',1);
colormap(jet); cbe18 = colorbar; ylabel(cbe18,'Coverage threshold (%)','FontSize',20,'FontWeight','bold');
v18e = ~isnan(pl) & ~isnan(nv);
if sum(v18e) > 2
    p18e = polyfit(pl(v18e), nv(v18e), 1);
    xf18e = linspace(min(pl(v18e)), max(pl(v18e)), 100);
    plot(xf18e, polyval(p18e, xf18e), 'r--','LineWidth',2);
    yp18e = polyval(p18e, pl(v18e));
    r_pl_nv_e = calcCorr(pl(v18e), nv(v18e));
    text(0.05,0.95,sprintf('Linear fit: y=%.2fx%+.1f\nCorrelation r=%.4f',p18e(1),p18e(2),r_pl_nv_e),...
        'Units','normalized','FontSize',20,'FontWeight','bold','Color','r','BackgroundColor','w');
end
xlabel('Path length (m)','FontSize',20,'FontWeight','bold');
ylabel('Number of viewpoints','FontSize',20,'FontWeight','bold');
title('Relationship Between Number of Viewpoints\nand Path Length Under Different Thresholds','FontSize',20,'FontWeight','bold');
setENFont(); saveas(he18,fullfile(OUT_EN,'Fig_E18_VPvsPathLen.fig'));drawnow;

fprintf('(19/%d) Fig_E19 Pareto Frontier: Coverage vs. Cost\n', N_FIGS);
cost19e = norm01(pl) + norm01(tt);
pi19e = findPareto(cost19e, pc);
he19 = figure('Position',[150,150,800,600],'Color','w','Name','E19','NumberTitle','off');
hold on; grid on; box on;
scatter(cost19e, pc, 100, xv, 'filled','MarkerEdgeColor','k','LineWidth',1.2);
colormap(jet); cbe19 = colorbar; ylabel(cbe19,'Coverage threshold (%)','FontSize',20,'FontWeight','bold');
if ~isempty(pi19e)
    scatter(cost19e(pi19e), pc(pi19e), 150, 'r','filled','MarkerEdgeColor','k','LineWidth',2);
    if length(pi19e) > 1
        [~, o19e] = sort(cost19e(pi19e));
        plot(cost19e(pi19e(o19e)), pc(pi19e(o19e)), 'r-','LineWidth',2.5);
    end
    for k19e = 1:length(pi19e)
        text(cost19e(pi19e(k19e)), pc(pi19e(k19e))+0.3, sprintf('%.0f%%',xv(pi19e(k19e))),...
            'HorizontalAlignment','center','FontSize',20,'FontWeight','bold','Color','r');
    end
end
xlabel('Composite cost (norm. path length + norm. turn angle)','FontSize',20,'FontWeight','bold');
ylabel('Path coverage (%)','FontSize',20,'FontWeight','bold');
title('Pareto Frontier Analysis of Coverage versus Cost','FontSize',20,'FontWeight','bold');
legend('All solutions','Pareto optimal','Pareto front','Location','southeast','FontSize',20,'Box','off');
setENFont(); saveas(he19,fullfile(OUT_EN,'Fig_E19_Pareto_Coverage_vs_Cost.fig'));drawnow;
fprintf('英文图完成! (%d个)\n', N_FIGS);

%% ========== [8] 保存数据 & 报告 ==========
save(fullfile(OUT_DATA, 'analysis_results.mat'), 'results', 'T', 'score', 'C', 'xv', 'conv_data');
fprintf('数据已保存到 %s\n', OUT_DATA);

fr = fopen(fullfile(OUT_DATA,'analysis_report.txt'),'w');
fprintf(fr,'=== 路径规划结果分析报告 ===\n');
fprintf(fr,'生成时间: %s\n\n',datestr(now));
fprintf(fr,'数据概况:\n');
fprintf(fr,'  有效数据点: %d\n',length(xv));
fprintf(fr,'  覆盖率范围: %.0f%%-%.0f%%\n',min(xv),max(xv));
fprintf(fr,'\n核心统计:\n');
fprintf(fr,'  平均视点覆盖率: %.2f%%\n',mean(vc));
fprintf(fr,'  平均路径覆盖率: %.2f%%\n',mean(pc));
fprintf(fr,'  平均视点数量: %.1f\n',mean(nv));
fprintf(fr,'  平均路径长度: %.1f m\n',mean(pl));
fprintf(fr,'  平均总转弯角度: %.1f°\n',mean(tt,'omitnan'));
fprintf(fr,'  平均计算时间: %.2f s\n',mean(tot));
fprintf(fr,'\n最优方案: 阈值 %.0f%%\n',xv(bi));
fprintf(fr,'  路径覆盖率: %.1f%%\n',pc(bi));
fprintf(fr,'  路径长度: %.1f m\n',pl(bi));
fprintf(fr,'  计算时间: %.1f s\n',tot(bi));
fprintf(fr,'\n相关系统计:\n');
fprintf(fr,'  覆盖率阈值 vs 路径长度: r=%.4f\n',r1);
fprintf(fr,'  视点数量 vs 路径长度: r=%.4f\n',r2);
fprintf(fr,'  视点覆盖率 vs 路径覆盖率: r=%.4f\n',r3);
fprintf(fr,'  路径长度 vs 计算时间: r=%.4f\n',r4);
fprintf(fr,'\n生成文件:\n');
fprintf(fr,'  Fig_Chinese/ - %d 个中文图\n',N_FIGS);
fprintf(fr,'  Fig_English/ - %d 个英文图\n',N_FIGS);
fprintf(fr,'  Data/ - 数据与报告\n');
fclose(fr);
fprintf('报告已保存\n');

fprintf('\n========================================\n');
fprintf('PATH_TJ 完成! 共生成 %d 中文 + %d 英文 = %d 个图表\n', N_FIGS, N_FIGS, 2*N_FIGS);
fprintf('输出目录: %s\n', OUT_ROOT);
fprintf('========================================\n');
