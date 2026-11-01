# Supplementary material

This directory accompanies the code and data release. The revision experiment archive is attached
to the [`r1` release](https://github.com/xiongaoxin0202/aircraft-upper-surface-ccp/releases/tag/r1).

## S1 — Revision experiments

[`ROB-2026-0584_Supplementary_S1_Revision_Experiments.zip`](https://github.com/xiongaoxin0202/aircraft-upper-surface-ccp/releases/download/r1/ROB-2026-0584_Supplementary_S1_Revision_Experiments.zip)
(233 entries). Restore it to this directory from the repository root with
`python tools/fetch_release_assets.py`.

| Folder | Content |
|---|---|
| `bircher_baseline/` | Re-implementation of the Bircher et al. iterative viewpoint resampling baseline, the unified Z-buffer variant, sweep and ten-run timing data, logs and evaluation scripts |
| `sensitivity/` | Ten sensitivity configurations: per-configuration `MAIN_viewpoint.m` / `MAIN_path_one.m`, run logs, summaries, executed paths and `sens_summary.csv` |
| `timing_reexecution/` | End-to-end re-execution of the proposed pipeline on A350-900, A340 and B757 at τ = 95 % |
| `statistics/` | Confidence intervals and effect sizes, plus figure-source files |
| `three_algorithms/` | MCTS baseline implementations (including the 382-candidate / 6-nearest-neighbour / 20,000-iteration protocol), SCP distribution runs and their result tables |
| `generalization/` | A340 and B757 summary outputs |

The MCTS baseline data are under `three_algorithms/results_paper/`; the 382-candidate ten-seed run
is `fair_mcts_382vox_95_70k_seeds.csv` with its selection/path tables and summary.

## S2 — Code and data

The code and data are provided in extracted, browsable form at the repository root: `src/`,
`experiments/`, `data/`, `results/`, `analysis_comprehensive_results/` and
`coverage_threshold_results/`, together with the data-provenance README and the MIT license.
