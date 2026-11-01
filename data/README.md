# Data Provenance

All point clouds in this repository are distributed in the **exact voxel-grid state** used by the
scripts, so that the reported results can be reproduced directly from the released files.

## A350-900 (main test model)

| File | Content | Points | Source & licensing |
|---|---|---|---|
| `A350_segmented/1.pcd … 7.pcd` | Seven functional regions produced by the ACNS segmentation (fuselage, wings, horizontal stabilisers, vertical stabiliser sides) | — | Derived from the A350-900 laser scan; distribution approved for this repository |
| `A350_segmented/UP-AIR.pcd` | Extracted upper-surface target point cloud, 0.1 m voxel grid | 70,018 | Same as above |
| `A350_segmented/view-normals.ply` | Candidate viewpoints (normal-offset sampling at 3.5 m) with outward normals | — | Same as above |
| `A350_full_model_downsampled.ply` | Full airframe at the 1.5 m voxel grid used by the ablation A1 (raw-PCA flip-ratio experiment) | 63,633 | Downsampled release of the proprietary laser scan; the full-resolution model is available from the authors on request |

The original laser-scanned model (~670 k vertices) was created with CloudCompare; it is **not**
redistributed in full resolution. The 1.5 m release preserves the geometry required to reproduce
the ablation A1 normal-flip result.

## Generalization models

| File | Aircraft | Voxel grid | Points | Notes |
|---|---|---|---|---|
| `A340.pcd` | A340 wide-body airliner | model scale (see paper) | — | Public/open model; see original source |
| `B757.ply` | B757 narrow-body airliner | original density | 175,687 | Public/open model; distributed as a release asset |

To keep the Git tree small, `B757.ply` and the two auxiliary `zong.ply` clouds are attached to the
repository's `r1` release. Restore them with `python tools/fetch_release_assets.py`; the script
verifies size and SHA-256 (see `release_assets.json`).

**Important**: the point-cloud models are provided at varying physical scales; the pipeline scales
the standoff and safety parameters by each model's bounding-box length, so only the *relative*
geometry matters for reproduction.

## Additional segmentation outputs

`A340_segmented/` (1–10.pcd) and `B757_segmented/` (1–9.pcd) contain the region-level
segmentation partitions of the A340 and B757 models (ACNS output). They are provided as auxiliary
data for users who wish to inspect or re-use the per-region segmentation of the generalization
models; the coverage experiments operate directly on the whole models under `A340.pcd` /
`B757.ply`.

## Results

`results/*.csv` are the outputs of the corresponding scripts from the reference run (2026-09-04).
The pipeline is deterministic, so the geometric and coverage metrics reproduce exactly; planning
times depend on the machine and load.

## Licensing

The code is MIT-licensed (see `../LICENSE`). The point-cloud data are released for research
reproduction purposes only; commercial use requires written permission from the authors
(Civil Aviation University of China). If you use the A350-900 model, please cite the study.
