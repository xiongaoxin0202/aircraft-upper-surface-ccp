# -*- coding: utf-8 -*-
"""generate_boxplots.py: 从 boxplot_data.csv 生成论文 Figure 19 箱线图（SCI 标准）"""
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({
    'font.family': 'Times New Roman',
    'font.size': 14,
    'axes.linewidth': 1.2,
    'xtick.direction': 'out',
    'ytick.direction': 'out',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'grid.linestyle': ':',
})

# 读取数据
rows = {}
with open('boxplot_data.csv', 'r', encoding='utf-8') as f:
    for r in csv.DictReader(f):
        rows.setdefault(r['metric'], {}).setdefault(r['method'], []).append(float(r['value']))

metrics = ['Viewpoint count', 'Path length (m)', 'Turning angle (deg)', 'Coverage (%)', 'Planning time (s)']
method_names = ['A (SCP+MMAS)', 'B (MCTS)', 'C (ASSCPP)', 'D (Proposed)']
colors = ['#4DBEEE', '#7E2F8E', '#EDB120', '#D95319']

fig, axes = plt.subplots(2, 3, figsize=(15, 8.5))
axes_flat = axes.flatten()
for k, met in enumerate(metrics):
    ax = axes_flat[k]
    data = [rows.get(met, {}).get(chr(65+i), []) for i in range(4)]
    bp = ax.boxplot(data, labels=[m.split(' ')[0] for m in method_names],
                    patch_artist=True, widths=0.55,
                    medianprops=dict(color='black', linewidth=1.5),
                    flierprops=dict(marker='o', markersize=4, markerfacecolor='gray'))
    for patch, c in zip(bp['boxes'], colors):
        patch.set_facecolor(c)
        patch.set_alpha(0.55)
        patch.set_edgecolor('black')
        patch.set_linewidth(1.2)
    ax.set_title(met, fontsize=14)
    ax.set_xlabel('Method')
    ax.grid(True, which='major', axis='y')
    # y 轴小标签
    ax.tick_params(labelsize=12)
axes_flat[5].axis('off')
plt.tight_layout()
plt.savefig('statistical_significance_boxplots.png', dpi=400, bbox_inches='tight', facecolor='white')
print('boxplot figure saved (400 dpi)')
