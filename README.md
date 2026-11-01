# Structure-Aware Coverage Path Planning for Autonomous Inspection of Aircraft Upper Surfaces

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2024a%2B-orange.svg)](https://www.mathworks.com/products/matlab.html)

Code and data for the study:

> **Structure-Aware Coverage Path Planning for Autonomous Inspection of Aircraft Upper Surfaces**
> Wei Zhang and Aoxin Xiong, *Robotica* (2026).

This repository provides the complete, runnable pipeline and the data used for multi-stage coverage
path planning (CPP) of autonomous UAV inspection over aircraft upper surfaces:

1. **Structure-aware point cloud processing** — ACNS-based segmentation of the aircraft into
   functional regions, region-wise normal reorientation (eliminating the 47.96 % normal-flip
   ambiguity of raw PCA), upper-surface extraction and voxel downsampling;
2. **Viewpoint generation** — normal-offset sampling at a 3.5 m standoff distance, with FOV-frustum
   (75° × 65°) and Z-buffer occlusion culling;
3. **Coverage path planning** — greedy set-cover viewpoint selection with coverage-threshold
   truncation, nearest-neighbour TSP sequencing over a symmetric Euclidean distance matrix;
4. **Trajectory generation** — cubic spline interpolation with moving-average smoothing, curvature
   constraints and iterative safety correction;
5. **Validation** — benchmark comparison (SCP+MMAS, MCTS, ASSCPP, Bircher et al.), statistical
   tests, five-module ablation, cross-aircraft generalization (A340, B757) and voxel-resolution /
   visibility sensitivity analysis.

## Repository structure

```
aircraft-upper-surface-ccp/
├── src/                        # Main pipeline programs (paper data generator)
│   ├── MAIN_UP.m               # Segmentation, upper-surface extraction, viewpoint sampling
│   ├── MAIN_viewpoint.m        # FOV + Z-buffer coverage matrix, greedy set-cover, truncation
│   ├── MAIN_path_one.m         # NN-TSP sequencing, cubic spline smoothing, safety correction
│   └── PATH_TJ.m               # Analysis and figure generation (Fig. E01–E19)
├── experiments/                # Supplementary experiment scripts
│   ├── collect_baseline_data.m # Baseline confirmation
│   ├── statistical_tests.m     # Statistical comparisons
│   ├── ablation_study.m        # Five-module ablation
│   ├── generalization_experiments.m  # Cross-aircraft experiments
│   ├── resolution_sensitivity.m      # Voxel-resolution / visibility sensitivity
│   └── generate_boxplots.py    # Boxplot of the five metrics
├── data/
│   ├── A350_segmented/         # Segmented A350-900 regions and upper-surface cloud
│   │   ├── 1.pcd … 7.pcd       # Seven functional regions (ACNS output)
│   │   ├── UP-AIR.pcd          # Upper-surface target point cloud (70,018 pts)
│   │   └── view-normals.ply    # Candidate viewpoints with outward normals
│   ├── A350_full_model_downsampled.ply   # Full A350 model (downsampled, ablation A1)
│   ├── A340.pcd                # Generalization model
│   ├── B757.ply                # Generalization model (release asset)
│   ├── A340_segmented/         # Auxiliary region-level segmentation of A340
│   │   └── zong.ply            # Combined cloud (release asset)
│   └── B757_segmented/         # Auxiliary region-level segmentation of B757
│       └── zong.ply            # Combined cloud (release asset)
├── results/
│   ├── analysis_data.csv       # Aggregated per-threshold metrics
│   ├── statistical_test_results.csv     # Statistical comparisons
│   ├── welch_anova_baselines.csv        # Welch ANOVA over the baselines
│   ├── ablation_results.csv             # Ablation metrics
│   ├── generalization_results.csv       # Cross-aircraft metrics
│   ├── resolution_sensitivity_results.csv  # Sensitivity metrics
│   ├── baseline_summary.csv             # Baseline confirmation output
│   ├── boxplot_data.csv                 # Samples for the five-metric boxplot
│   └── figures/
│       └── statistical_significance_boxplots.png
├── analysis_comprehensive_results/
│   ├── Data/                   # Aggregated analysis output
│   └── Fig_Chinese/ Fig_English/  # MATLAB .fig files (Fig. E01–E19)
├── coverage_threshold_results/ # Per-threshold runs, τ = 0.80–1.00
├── supplementary/              # Revision experiment notes; S1 ZIP is a release asset
├── tools/
│   └── fetch_release_assets.py # Restore and verify the release assets
├── release_assets.json         # Release asset manifest (paths, sizes, SHA-256)
├── data/README.md              # Data provenance and licensing
├── CITATION.cff
├── LICENSE                     # MIT License
└── README.md
```

## Large files (release assets)

Four binaries are attached to the [`r1` release](https://github.com/xiongaoxin0202/aircraft-upper-surface-ccp/releases/tag/r1)
rather than committed to the Git tree:

| Path | File |
|---|---|
| `data/B757.ply` | B757 generalization model |
| `data/A340_segmented/zong.ply` | Auxiliary combined A340 segmented cloud |
| `data/B757_segmented/zong.ply` | Auxiliary combined B757 segmented cloud |
| `supplementary/ROB-2026-0584_Supplementary_S1_Revision_Experiments.zip` | Revision experiment archive |

After cloning, restore them to their canonical paths with:

```bash
python tools/fetch_release_assets.py
```

The script verifies the SHA-256 digest and size of every file (see
[`release_assets.json`](release_assets.json)).

## Requirements

- MATLAB R2024a or newer (developed and verified on R2026a)
  - Statistics and Machine Learning Toolbox (`KDTreeSearcher`, `pcnormals`, `boxplot`)
  - Curve Fitting Toolbox (`smooth`, used by the path script)
- Python ≥ 3.8 with numpy and matplotlib (only for `generate_boxplots.py`)
- (Optional) Parallel Computing Toolbox — the released code runs without it

## Quick start

After a fresh clone, first restore the large files from the release:

```bash
python tools/fetch_release_assets.py
```

The MATLAB scripts expect to run from a working directory that contains `seg_all/` (segmented
regions and upper-surface cloud), `coverage_threshold_results/` and
`analysis_comprehensive_results/` (all generated automatically). The published data under
`data/A350_segmented/` already contains the segmentation output, so you can start from the
viewpoint-selection stage directly:

```cmd
:: Windows
mkdir work && cd work
mkdir seg_all
copy ..\data\A350_segmented\* seg_all\          :: 1.pcd-7.pcd, UP-AIR.pcd, view-normals.ply
copy ..\src\*.m .
:: Linux/macOS
:: mkdir -p work/seg_all && cp data/A350_segmented/* work/seg_all/ && cp src/*.m work/
```

```matlab
% In MATLAB, cd to the work folder, then:

MAIN_viewpoint      % Stage I-II: FOV+Z-buffer visibility matrix, greedy set-cover with
                    %             tau-truncation -> coverage_threshold_results/
MAIN_path_one       % Stage II: NN-TSP + cubic spline + safety correction -> per-threshold paths
PATH_TJ             % Aggregates per-threshold metrics -> analysis_comprehensive_results/
```

To re-run segmentation from scratch, start earlier with `MAIN_UP` (it reads `seg_all/1.pcd … 7.pcd`
and regenerates `UP-AIR.pcd`/`view-normals.ply`).

Reproduction of the reported results:

| Result | Command | Output |
|---|---|---|
| Threshold sweep (proposed method) | `MAIN_viewpoint` + `MAIN_path_one` + `PATH_TJ` | `results/analysis_data.csv` (copy from `analysis_comprehensive_results/Data/`) |
| Benchmark comparison | Deterministic pipeline; baseline runs | `results/baseline_summary.csv` |
| Statistical comparisons | `experiments/statistical_tests.m` (run in `work/`) | `results/statistical_test_results.csv` |
| Ablation | `experiments/ablation_study.m` (run in `work/`) | `results/ablation_results.csv` |
| Cross-aircraft generalization | `experiments/generalization_experiments.m` (run in `work/`, needs `../data/A340.pcd` and `B757.ply` copied to `work/3D_dataset/`) | `results/generalization_results.csv` |
| Sensitivity study | `experiments/resolution_sensitivity.m` (run in `work/`) | `results/resolution_sensitivity_results.csv` |
| Five-metric boxplot | `python generate_boxplots.py` (run in `work/` after the export above) | `results/figures/statistical_significance_boxplots.png` |
| Baseline confirmation | `experiments/collect_baseline_data.m` (run in `work/`) | `results/baseline_vs_manuscript.csv` |

The ablation (A1) full-model flip-ratio experiment additionally needs
`data/A350_full_model_downsampled.ply` placed in the working directory as `model.ply`.

For a headless/CI run use: `matlab -softwareopengl -batch "MAIN_viewpoint"` (the `-softwareopengl`
flag avoids graphics-driver crashes on some Windows setups).

## Key parameters

All parameters are identical to those used in the study:

| Parameter | Value | Notes |
|---|---|---|
| Horizontal / vertical FOV | 75° / 65° | Camera frustum definition |
| Near / far plane | 0.5 m / 5.46 m | Depth-of-field interval |
| Standoff distance d_safe | 3.5 m | Derived from the GSD bound |
| Coverage threshold τ | 0.80–1.00 | Truncation of greedy set-cover |
| Occlusion culling | Z-buffer | Adaptive angular grid (100–800 × 67–600) |
| TSP solver | Nearest-neighbour | Deterministic, no random restarts |
| Smoothing | Cubic spline + moving average | max_path_points = 100 |

## Data provenance and licensing

See [`data/README.md`](data/README.md) for full provenance details. The full-resolution A350-900
laser scan and the full-resolution source models are proprietary; the repository distributes
**downsampled releases** at the exact voxel grids used in the study, so all reported results remain
reproducible from the released files.

The code is released under the **MIT License** (see [`LICENSE`](LICENSE)). The point-cloud data are
released for research reproduction only; commercial use requires written permission from the
authors (Civil Aviation University of China).

## Citation

If you use this code or data in your research, please cite the study:

```bibtex
@article{zhang2026structureaware,
  title   = {Structure-Aware Coverage Path Planning for Autonomous Inspection of Aircraft Upper Surfaces},
  author  = {Zhang, Wei and Xiong, Aoxin},
  journal = {Robotica},
  year    = {2026}
}
```

A [`CITATION.cff`](CITATION.cff) file is also provided.

## Contact

Aoxin Xiong — 2021012200@cauc.edu.cn
College of Safety Science and Engineering, Civil Aviation University of China, Tianjin 300300, China.
