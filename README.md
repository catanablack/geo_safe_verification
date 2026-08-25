# Real-Time Geometric Safety Verification Platform

A vendor-independent, geometry-based safety verification platform for autonomous vehicle (AV) motion planning — certifying in real time that a planned trajectory maintains a provable safety margin against current and forecasted occupancy of the environment.

https://github.com/user-attachments/assets/31b0ae50-3435-41ca-ae26-58a584b81635


See [PROJECT_PLAN.md](PROJECT_PLAN.md) for the full problem statement, challenges, goals, and phased implementation/deployment plan, and [src/README.md](src/README.md) for the module-level architecture and data-flow.

## Repository Layout

```
.
├── PROJECT_PLAN.md          # Problem statement, goals, implementation & deployment plan
├── pyproject.toml           # Python backend package config
└── src/
    ├── common/               # Shared types & config used by every module
    ├── spatial_fusion/       # LIDAR/camera/radar ingestion → probabilistic 3D occupancy
    ├── predictive_occupancy/ # GNN-based forecasting of dynamic agents
    ├── verification_engine/  # Core swept-volume + signed distance field safety verification
    ├── trajectory_arbitration/ # Multi-objective fallback trajectory generation
    ├── simulation_harness/   # Scenario generation + benchmark execution
    ├── cross_platform/       # Vehicle-class generalization (robotaxi/trucking/last-mile)
    ├── api/                  # FastAPI backend wiring the modules together
    └── frontend/             # React + Three.js 3D visualization dashboard / simulation PoC
```

## Backend (Python)

Requires Python >= 3.11.

```bash
# from the repo root
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"

# run the API
uvicorn src.api.main:app --reload

# run tests
pytest
```

Key dependencies: NumPy, SciPy, Open3D, scikit-image, PyTorch, PyTorch Geometric, FastAPI, Uvicorn.

## Frontend (TypeScript / React / Three.js)

A `@react-three/fiber` (Three.js) 3D dashboard and vehicle/pedestrian traffic simulation proof-of-concept.

```bash
cd src/frontend
npm install
npm run dev       # start the Vite dev server
npm run build     # type-check (tsc -b) and production build
```

### Simulation stress test

The frontend includes a standalone stress-test harness that runs the vehicle/pedestrian
simulation logic across multiple seeded scenarios and reports any collision incidents
detected via SAT-based oriented-rectangle overlap:

```bash
cd src/frontend
npx tsx scripts/simulationStressTest.ts
```

## Architecture

See [src/README.md](src/README.md) for the full module responsibility table, data-flow diagram, and the planner-agnostic interface contract that keeps `verification_engine` and `trajectory_arbitration` independent of any specific AV planning stack.

## Regulatory Context

The verification engine is designed to generate provable, mile-independent safety evidence supporting AV safety-case submissions under frameworks such as California Title 13 CCR Articles 3.7/3.8 and the federal 49 C.F.R. Part 555 exemption process. See [PROJECT_PLAN.md § 3.2](PROJECT_PLAN.md#32-regulatory-context) for details.

## Status

Early-stage structural scaffold (interfaces + stubs) with an actively developed frontend simulation PoC. Not production-ready.
