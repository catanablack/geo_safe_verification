# Conservative Voxel-Space Trajectory Verification: Prototype Results

**Version:** 0.1

**Date:** 2026-11-07

**Scope:** NumPy/SciPy core prototype; not a production safety assessment

## Abstract

This report documents a first executable version of a geometric trajectory
verifier. The implementation converts a probabilistic occupancy grid into a
Euclidean distance field, samples an extruded vehicle footprint along
interpolated poses, and subtracts explicit voxel, interpolation, and sampling
error allowances before returning a safety verdict. Static occupancy and
deterministic, linearly interpolated agent-center forecasts are supported.

Analytic box and sphere tests and 128 deterministic comparisons against exact
axis-aligned rectangle/voxel overlap geometry produced zero false accepts in
those test cases. A local latency sweep measured p50/p99 between 1.43/1.66 ms
and 24.07/24.82 ms across the tested grid and agent-count ranges. A first
nuScenes mini evaluation covered 285 candidates from 10 scenes; the verifier
had zero false accepts and a 39.8% false-reject rate on this sampled set.
These results are a small prototype evaluation, not a population-level
performance claim or a production safety assessment. A nuScenes train/validation
metadata run evaluated 300 candidates across 100 evenly spaced scenes; only 21
candidates were labeled collisions, so its false-accept rates have wide
uncertainty and should not be treated as certification evidence.

## Method

### Occupancy distance field

Occupancy values at or above the configured threshold are treated as occupied
closed voxels. A SciPy Euclidean distance transform computes distances between
voxel centers. For free samples, the field subtracts the voxel half-diagonal,
\(\sqrt{3}r/2\), where \(r\) is the voxel edge length. This converts distance
to an occupied voxel center into a conservative clearance to its cube. Query
values are trilinearly interpolated. The safety-query path subtracts another
\(\sqrt{3}r/2\) to bound interpolation error between voxel centers.

Queries outside the mapped volume are assigned a negative clearance. In an
empty map, the map boundary is treated as unknown and limits the reported
clearance. Thus, absence of occupied voxels does not imply unbounded safe
space.

### Swept footprint and certification

The ego footprint is filled with a regular planar sample lattice and edge
samples, then extruded between configured vertical bounds. Waypoint positions
are linearly interpolated; headings follow the shortest angular interpolation.
Time samples are subdivided according to translation plus the footprint's
maximum rotational arc length. With the sampling step capped at half a voxel,
the implementation subtracts a conservative \(2s\) sweep allowance, where
\(s\) is the sampling step.

Static clearance is the minimum SDF lower bound over the sampled swept volume,
less the sweep allowance. Dynamic-agent checks compare sampled ego-volume
points against forecast centers, subtract a configured disc radius, and add an
allowance for ego sampling and agent motion between forecast points. A portion
of a trajectory outside an agent forecast's horizon is rejected. A safe verdict
means the lower bound is strictly greater than the requested margin.

These bounds are intended to prevent discretization-induced false accepts for
the modeled geometry. They do not cover perception errors, probabilistic
occupancy uncertainty, incorrect footprints, unmodeled objects, or deviations
from the deterministic linear forecasts.

## Correctness checks

The focused test suite includes:

1. **Box field:** sampled clearances are no greater than analytic point-to-box
   distances, and points in occupied voxels have non-positive distance.
2. **Sphere field:** sampled clearances are no greater than analytic
   point-to-sphere clearances for a sphere rasterized into voxel occupancy.
3. **Exact overlap check:** 128 seeded scenes compare the verifier with exact
   axis-aligned rectangle-versus-voxel-box overlap tests. The observed false
   accept count was **0/128**. Conservative false rejections are allowed.
4. **Dynamic crossing and invalid inputs:** an interpolated crossing forecast
   is rejected, out-of-map queries fail closed, and duplicate waypoint times
   are rejected.

The exact-geometry experiment is a small regression check, not an exhaustive
proof or a statistical estimate of field performance.

## NumPy/SciPy latency

The benchmark was run on Python 3.11.0, NumPy 2.3.3, SciPy 1.17.1, macOS
15.7 / arm64. Each case had one warm-up call and 20 measured calls. Grid size
means a grid of `N x N x max(8, N/4)` voxels at 0.5 m resolution; p99 is an
empirical percentile over only 20 samples and should be treated as indicative.

| Grid side | Voxels | Forecast agents | p50 (ms) | p99 (ms) |
|---:|---:|---:|---:|---:|
| 32 | 8,192 | 0 | 1.43 | 1.66 |
| 32 | 8,192 | 5 | 1.89 | 2.13 |
| 32 | 8,192 | 20 | 3.06 | 3.44 |
| 64 | 65,536 | 0 | 7.07 | 7.49 |
| 64 | 65,536 | 5 | 7.44 | 8.03 |
| 64 | 65,536 | 20 | 8.28 | 8.53 |
| 96 | 221,184 | 0 | 21.87 | 22.72 |
| 96 | 221,184 | 5 | 22.87 | 24.60 |
| 96 | 221,184 | 20 | 24.07 | 24.82 |

The measured workload uses a fixed footprint and short trajectory. Grid size
dominates this run because the distance transform is rebuilt on every
verification. Agent count adds work, especially at the smallest grid, but these
measurements should not be generalized to longer paths, finer voxels, or
larger/more complex footprints. The 50 ms target in configuration is not a
hard real-time guarantee.

## nuScenes train/validation metadata benchmark

The official AWS Open Data bucket provided `v1.0-trainval_meta.tgz`
(461,678,030 bytes). The five JSON tables needed by the adapter were extracted;
no sensor blobs were written to disk. The archive passed `gzip -t`, but its
computed MD5 (`537d3954ec34e5bcb89a35d4f6fb0d4a`) does not match the published
`md5.checksum` value (`3eee698806fcf52330faa2e682b9f3a1`). The earlier mini
archive also passed gzip validation and sampled byte-range checks but had a
different computed and published MD5 (`dc2267f6e62b221efb9575929028603b` vs.
`791dd9ced556cfa1b425682f177b5d9b`). Neither discrepancy is resolved, so
interpret both dataset runs as provisional.

The train/validation tables contain 850 scenes. This run deterministically
selected 100 evenly spaced scenes from the scene table order and evaluated one
window in each. Each window uses five annotated samples over a 2-second horizon
at 2 Hz; the candidate set contains the logged ego path and fixed lateral
offsets of -1 m and +1 m. Ground truth checks yaw-oriented 3D
rectangular-prism intersection at annotated times, using a fixed ego box of
4.8 x 1.9 x 1.5 m. The certifier uses an empty static occupancy map and future
annotated objects represented by per-agent enclosing circular footprints.
Requested clearance was 0.5 m. No interpolation between ground-truth
annotation times is included in the collision labels.

The evaluation contains 300 candidate trajectories: 21 exact collisions and
279 exact-clear candidates. False-accept rate is the fraction of exact
collisions classified safe; false-reject rate is the fraction of exact-clear
candidates classified unsafe.

| System | False accepts / GT collisions | False-accept rate | False rejects / GT clear | False-reject rate | Margin MAE (m) | p50 latency (ms) | p99 latency (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Geometric verifier | 0 / 21 | 0.000 | 126 / 279 | 0.452 | 2.536 | 110.066 | 1192.421 |
| Center-distance threshold | 0 / 21 | 0.000 | 135 / 279 | 0.484 | 1.826 | 0.033 | 0.116 |
| Constant-velocity TTC | 0 / 21 | 0.000 | 140 / 279 | 0.502 | 3.068 | 0.260 | 1.573 |
| RSS-inspired longitudinal rule | 9 / 21 | 0.429 | 92 / 279 | 0.330 | 32.813* | 0.147 | 0.630 |

False-accept rate is the fraction of exact-collision candidates classified
safe; false-reject rate is the fraction of exact-clear candidates classified
unsafe. The distance baseline uses enclosing-circle clearance and a 0.5 m
threshold. The TTC baseline assumes constant relative velocity and predicts
contact between enclosing circles within the horizon. The RSS-inspired rule
applies a longitudinal braking-distance formula to a forward corridor. It
classified 9 of the 21 collisions as safe in this selected set.

`*` The RSS value is braking-distance slack rather than Euclidean clearance,
so its MAE is not directly comparable with the other margin errors. This is a
transparent local RSS-style baseline, **not** an execution of an open-source
RSS library such as `ad-rss-lib`; the published Python wheels currently target
Linux x86-64 and are not compatible with this macOS/arm64 environment. The
selected windows, small number of positive collision labels, coarse
object-circle approximation, limited sample times, and unresolved archive
checksum constrain interpretation. The verifier's measured p50/p99 also
exceed the configured 50 ms target.

The reproducible adapter and runner are in
[`benchmarks/nuscenes_eval.py`](../benchmarks/nuscenes_eval.py) and
[`benchmarks/run_nuscenes_benchmark.py`](../benchmarks/run_nuscenes_benchmark.py).
Detailed per-candidate results are stored outside the repository at
`$HOME/datasets/nuscenes/nuscenes-trainval-results.json` and are not committed.
The nuScenes data is restricted to non-commercial use under its
[terms](https://www.nuscenes.org/terms-of-use). Applicable GM/HP IP terms must
also be reviewed before publishing benchmark artifacts. No software license
has been added.

## Limitations and next work

The current forecast schema has centers but no agent dimensions, so agents are
modeled as configurable-radius discs. Forecast confidence is not converted into
a geometric uncertainty set. The occupancy uncertainty array is also ignored
after thresholding. The verifier is conservative at coarse resolutions and may
reject physically clear trajectories. The current benchmark measures a Python
NumPy/SciPy path and includes SDF construction; caching or an optimized C++ /
pybind11 hot loop may change the latency profile.

Next work is to resolve the archive-checksum discrepancy, evaluate more of the
licensed train/validation scenes, add a reproducible comparison against an
actual open-source RSS implementation on a supported platform, and profile or
cache the SDF / implement an optimized native hot loop. The prototype should
not be described as a production certification mechanism until those
assumptions and the complete implementation have been independently validated.

## References

- S. Shalev-Shwartz, S. Shammah, and A. Shashua, [“On a Formal Model of Safe and Scalable Self-driving Cars”](https://arxiv.org/abs/1708.06374), 2017.
- H. Caesar et al., [“nuScenes: A multimodal dataset for autonomous driving”](https://arxiv.org/abs/1903.11027), 2019.
- [Flexible Collision Library](https://github.com/flexible-collision-library/fcl), a general-purpose collision and distance-query library (not used by this prototype).
