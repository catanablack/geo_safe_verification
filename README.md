# Geometric Trajectory Safety Verification

https://github.com/user-attachments/assets/31b0ae50-3435-41ca-ae26-58a584b81635

**Project description:** A planner-independent research prototype for checking
whether candidate autonomous-vehicle trajectories maintain geometric clearance
from modeled voxel occupancy and forecasted agents.

**Topics:** autonomous-vehicles, trajectory-verification, computational-geometry,
signed-distance-fields, voxel-grids, safety

## Executive summary

Autonomous vehicles (AVs) are developed and tested by different organizations
across cities and states, using different planners, vehicle platforms, and
validation processes. This fragmentation makes it harder to compare evidence,
reuse safety engineering across deployments, and build confidence that an AV
system behaves predictably when it expands into new operating areas. The
project addresses one bounded part of that challenge: checking whether a
proposed vehicle trajectory maintains geometric clearance from the environment
and modeled moving agents.

The project idea is a planner-independent verification component with a
consistent input/output contract. It receives a candidate trajectory, vehicle
footprint, voxel occupancy representation, agent forecasts, and a requested
clearance; it then evaluates the modeled swept volume and reports a verdict,
clearance, and supporting diagnostics. The intended scope is an engineering
tool for offline replay and validation first, with shadow-mode evaluation as a
later stage. It is not a perception, prediction, planning, or control system,
and it is not intended to serve as the sole runtime safety mechanism.

A reusable interface and repeatable evaluation method could help AV teams
compare planner changes and validation results across vehicles and locations,
instead of rebuilding this particular geometric check for each program. If
validated and adopted by developers, simulation providers, and fleet
engineering teams, such a common technical building block could lower
integration friction and support more consistent safety evidence as AV
technology scales across cities and states. It would complement—not replace—
local laws, regulatory processes, system-specific safety cases, or
independent review; this project cannot itself standardize regulations or
guarantee nationwide deployment.

The current repository demonstrates a NumPy/SciPy implementation of a voxel
signed-distance field (SDF), swept-footprint sampling, conservative clearance
checks, geometry tests, latency measurements, and an early nuScenes metadata
benchmark. It is a research prototype—not a production safety product,
vehicle-level safety guarantee, or regulatory certification tool. Perception
and forecast uncertainty are incomplete, several broader platform components
remain scaffolding, and measured dataset-workload latency exceeds the
configured 50 ms target. The near-term objective is to establish technical
soundness and useful operating limits, then test the workflow with engineering
users and authorized, representative data.

Any progression toward shadow-mode or operational use depends on independent
technical review, representative authorized data, partner-specific validation,
performance improvements, and legal/IP clearance. Current dataset benchmark
results are provisional because downloaded archive MD5 values do not match the
publisher's listed checksums.

### Project documents

| Document | What it covers |
|---|---|
| [Executive project and commercialization brief](docs/executive-project-brief.md) | Project positioning, objectives, potential customers, product hypotheses, delivery gates, validation, commercialization, risks, and next actions |
| [Technical report](docs/technical-report.md) | Prototype methods, correctness tests, latency measurements, nuScenes benchmark results, and technical limitations |
| [Project plan](PROJECT_PLAN.md) | Broader problem statement, proposed system components, phased implementation, deployment, and strategic goals |
| [Source architecture](src/README.md) | Repository modules, responsibilities, data flow, and planner-agnostic interface |

## Technical approach

The verification prototype:

1. Converts thresholded voxel occupancy into a Euclidean distance field.
2. Samples an extruded vehicle footprint along interpolated trajectory poses.
3. Accounts for voxel, interpolation, and sweep sampling allowances in its
   clearance estimate.
4. Checks dynamic forecasts using linearly interpolated agent centers and
   configured circular bounds.

A safe verdict applies only to the modeled geometry and supplied inputs. It
does not account for missing obstacles, perception errors, uncertain
predictions, or other unmodeled behavior.

## Current status

| Area | Status | Notes |
|---|---|---|
| Voxel SDF and conservative clearance query | Implemented | Euclidean distance transform, trilinear interpolation, discretization bounds, and fail-closed map boundaries |
| Swept planar footprint sampler | Implemented | Interpolated position/heading, filled footprint samples, vertical extrusion, and a sampling-error allowance |
| Safety certifier | Prototype | Checks static occupancy and linearly interpolated agent centers represented by configurable-radius discs |
| Geometry tests | Implemented | Analytic box/sphere tests and 128 deterministic rectangle/voxel comparisons; see the technical report for scope |
| NumPy/SciPy latency benchmark | Implemented | Reports p50/p99 by grid size and forecast-agent count; measurements are not real-time guarantees |
| nuScenes benchmark and simple baselines | Prototype | Selected-window trainval evaluation; archive checksum discrepancy and RSS-library comparison remain |
| Production certification, uncertainty model, optimized native kernel | Planned | Requires broader validation, improved performance, and independent review |

## Implementation and validation plan

The recommended sequence is to harden and independently validate the geometry
core; expand offline scenario evaluation and reproducibility; integrate with a
design partner in non-actuating replay/shadow mode; and only then decide whether
productization or runtime use is justified. The [executive brief](docs/executive-project-brief.md)
sets out stage gates, metrics, customer discovery, and governance. The broader
[project plan](PROJECT_PLAN.md) describes the proposed end-to-end platform.

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

The measurements are documented in the [technical report](docs/technical-report.md).
The implementation rebuilds the SDF during verification; caching or an
optimized C++/pybind11 hot loop is future work.

### nuScenes metadata benchmark

The official train/validation metadata archive is about 462 MB and contains
annotations and poses without sensor blobs. nuScenes is restricted to
non-commercial use under the [nuScenes terms](https://www.nuscenes.org/terms-of-use).
Keep data outside the repository:

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

The extractor stores only five JSON tables. The runner selects evenly spaced
scenes when limiting the window count. Results default to
`$HOME/datasets/nuscenes/nuscenes-trainval-results.json`. This benchmark uses
oriented-box labels at annotated timestamps, logged ego paths plus lateral
perturbations, distance-threshold and constant-velocity TTC baselines, and a
transparent RSS-inspired rule. The latter is not an execution of `ad-rss-lib`.
See the technical report for results and the unresolved archive checksum
discrepancy.

## Repository layout

```text
src/
├── common/                 # Shared dataclasses and configuration
├── verification_engine/    # SDF, swept-footprint sampling, and certifier
├── spatial_fusion/         # Probabilistic occupancy-grid construction scaffolding
├── predictive_occupancy/   # Agent forecasting prototypes
├── trajectory_arbitration/ # Candidate fallback trajectory prototypes
├── simulation_harness/     # Scenario and benchmark scaffolding
├── cross_platform/         # Vehicle profiles and adapters
├── api/                    # FastAPI backend scaffolding
└── frontend/               # React + Three.js visualization prototype

benchmarks/                 # NumPy latency and nuScenes evaluation tools
tests/                      # Geometry and benchmark tests
docs/                       # Technical report and executive project brief
```

## Known limitations and assumptions

- Occupancy probabilities are thresholded; occupancy uncertainty is not
  propagated into the distance field. Results depend on the supplied map.
- The ego footprint is a 2D polygon extruded between configured z bounds.
  Dynamic agents use circular bounds because the shared forecast type provides
  centers but not agent dimensions.
- Agent positions between forecast samples are linearly interpolated and
  deterministic. Confidence and unmodeled prediction error are not included;
  trajectory portions beyond a forecast horizon fail closed.
- Conservative sampling and voxel bounds can produce false rejections,
  especially at coarse resolutions. Tests cover finite cases, not every
  supported input.
- Points outside the observed grid fail closed. Safe verdicts cannot account
  for obstacles or agents absent from the provided inputs.
- This Python implementation has no hard real-time guarantee. Latency varies
  by machine, grid shape, footprint, and forecast count.
- The nuScenes mini and trainval metadata archives were obtained for
  non-commercial evaluation. Both passed gzip integrity checks, but computed
  MD5 values differed from published checksums; benchmark results are
  provisional. An actual open-source RSS-library comparison has not been
  performed.
- Applicable GM/HP IP terms must be reviewed before publishing benchmark
  artifacts or adding a software license.

## Related work

- Shalev-Shwartz, Shammah, and Shashua, [“On a Formal Model of Safe and Scalable Self-driving Cars”](https://arxiv.org/abs/1708.06374) — Responsibility-Sensitive Safety (RSS).
- Caesar et al., [“nuScenes: A multimodal dataset for autonomous driving”](https://arxiv.org/abs/1903.11027) — public AV dataset used for a prototype benchmark.
- The [Flexible Collision Library](https://github.com/flexible-collision-library/fcl) — a general-purpose collision/distance-query library; not used by this NumPy prototype.

No license file is included. Review applicable intellectual-property terms
before adding a license or publishing additional project artifacts.
