# Real-Time Geometric Safety Verification Platform

**Description:** A NumPy/SciPy prototype for conservative, geometry-based trajectory safety checks over voxel occupancy and linear agent forecasts.

**Topics:** autonomous-vehicles, trajectory-verification, computational-geometry, signed-distance-fields, voxel-grids, safety

Planner-independent research prototype for evaluating whether a candidate vehicle
trajectory maintains a requested geometric margin against a 3D occupancy grid and
simple dynamic-agent forecasts. A safe verdict is conservative with respect to the
modeled geometry and stated discretization bounds; it is not a production safety
case or a guarantee about perception and prediction errors.

## Status

| Area | Status | Notes |
|---|---|---|
| Voxel SDF and conservative clearance query | Implemented | Euclidean distance transform, trilinear interpolation, voxel-shape/interpolation error bounds, and fail-closed map boundaries |
| Swept planar footprint sampler | Implemented | Interpolated position/heading, filled footprint samples, vertical extrusion, and a sampling-error allowance |
| Safety certifier | Prototype | Checks static occupancy and linearly interpolated agent centers represented by configurable-radius discs |
| Analytic and brute-force geometry tests | Implemented | Box and sphere cases plus 128 deterministic axis-aligned rectangle/voxel comparisons; see the report for scope |
| NumPy/SciPy latency benchmark | Implemented | Reports p50/p99 by grid size and forecast-agent count; hardware-specific measurements are not real-time guarantees |
| nuScenes trainval metadata benchmark and simple baselines | Prototype | Supports the official trainval metadata tables; selected-window evaluation and an actual RSS-library comparison remain |
| Production certification, uncertainty model, optimized native kernel | Planned | Requires broader geometry, sensor, prediction, and platform validation |

## Quick start

Requires Python 3.11 or newer. Install the project dependencies and development
tools, then run the test suite:

```bash
python -m pip install -e ".[dev]"
python -m pytest
```

Run the NumPy/SciPy latency sweep (defaults: 32/64/96 grid side lengths, 0/5/20
agents, 20 measured iterations per case):

```bash
python benchmarks/benchmark_numpy.py
python benchmarks/benchmark_numpy.py --iterations 50 --output benchmark.csv
```

The measurements in [the technical report](docs/technical-report.md) are from
the NumPy implementation; a C++/pybind11 hot loop is future work.

### nuScenes benchmark

The official train/validation metadata archive is about 462 MB and contains
annotations and poses without sensor blobs. nuScenes is restricted to
non-commercial use under the [nuScenes terms](https://www.nuscenes.org/terms-of-use).
Download the archive and keep it outside the repository:

```bash
mkdir -p "$HOME/datasets/nuscenes"
curl -L --output "$HOME/datasets/nuscenes/v1.0-trainval_meta.tgz" \
  https://d36yt3mvayqw5m.cloudfront.net/public/v1.0/v1.0-trainval_meta.tgz
python benchmarks/prepare_nuscenes_metadata.py \
  "$HOME/datasets/nuscenes/v1.0-trainval_meta.tgz" \
  "$HOME/datasets/nuscenes/trainval-tables" \
  --version v1.0-trainval
python benchmarks/run_nuscenes_benchmark.py \
  "$HOME/datasets/nuscenes/trainval-tables" \
  --version v1.0-trainval --max-windows 100
```

The extractor stores only five nuScenes JSON tables. When limiting windows,
the runner selects evenly spaced scenes from the dataset before taking
additional windows from any scene. Results
default to `$HOME/datasets/nuscenes/nuscenes-trainval-results.json`. Use
`--version v1.0-mini` for the mini tables; its sensor archive is about 4.17 GB.
The benchmark uses oriented-box collision labels at annotated timestamps,
logged ego paths plus lateral perturbations, a distance-threshold rule,
constant-velocity TTC, and a transparent RSS-inspired rule. The latter is not
the `ad-rss-lib` implementation.

## Repository layout

```text
src/
├── common/                 # Shared dataclasses and configuration
├── verification_engine/    # SDF, swept-footprint sampling, and certifier
├── spatial_fusion/         # Probabilistic occupancy-grid construction
├── predictive_occupancy/   # Agent forecasting prototypes
├── trajectory_arbitration/ # Candidate fallback trajectory prototypes
├── simulation_harness/     # Scenario and benchmark scaffolding
├── cross_platform/         # Vehicle profiles and adapters
├── api/                    # FastAPI backend scaffolding
└── frontend/               # React + Three.js visualization prototype
```

See [src/README.md](src/README.md) for module boundaries and [PROJECT_PLAN.md](PROJECT_PLAN.md)
for the broader project proposal.

For project positioning, proposed customers, delivery gates, validation,
commercialization hypotheses, and immediate actions, see the
[executive project brief](docs/executive-project-brief.md).

## Known limitations and assumptions

- Occupancy probabilities are thresholded; `uncertainty` is not propagated into
  the distance field. The result is only as sound as the supplied occupancy map.
- The ego footprint is a 2D polygon extruded between configured z bounds.
  Dynamic agents are modeled as circles with a configured radius because the
  shared forecast type currently provides centers but not agent dimensions.
- Agent positions between forecast samples are linearly interpolated and treated
  as deterministic. Confidence values and unmodeled prediction error are not
  incorporated. Forecasts are not extrapolated beyond their horizon; such
  trajectory portions fail closed.
- The finite sampling and voxel bounds are deliberately conservative and can
  produce false rejections, especially at coarse resolutions. Tests establish
  zero false accepts only for the documented finite cases; they are not a formal
  verification of every supported input.
- Points outside the observed grid fail closed. Safe verdicts do not account for
  obstacles or agents absent from the provided inputs.
- This Python implementation has no hard real-time guarantee. Latency varies by
  machine, grid shape, footprint, and forecast count.
- The nuScenes mini and trainval metadata archives were obtained for
  non-commercial evaluation. Both passed gzip integrity checks, but their
  computed MD5 values did not match the publisher's listed checksums; benchmark
  results are provisional. An actual open-source RSS-library comparison has
  not been performed; the published `ad-rss` wheels currently target Linux
  x86-64, not this macOS/arm64 environment.
- nuScenes is restricted to non-commercial use under its terms. Applicable
  GM/HP IP terms must be reviewed before publishing benchmark artifacts or
  adding a software license.

## Related work

- Shalev-Shwartz, Shammah, and Shashua, [“On a Formal Model of Safe and Scalable Self-driving Cars”](https://arxiv.org/abs/1708.06374) — Responsibility-Sensitive Safety (RSS).
- Caesar et al., [“nuScenes: A multimodal dataset for autonomous driving”](https://arxiv.org/abs/1903.11027) — public AV dataset used for a prototype benchmark.
- The [Flexible Collision Library](https://github.com/flexible-collision-library/fcl) — a general-purpose collision/distance-query library; not used by this NumPy prototype.

No license file is included. Review the applicable intellectual-property terms
before adding a license or publishing additional project artifacts.
