# Geometric Safety Verification Platform for Autonomous and Assisted Driving

**Author:** Dr. Juan Carlos Catana Salazar
**Document type:** Project Plan — Problem Statement, Challenges, Overview, Goals, Implementation Plan, Deployment Plan
**Status:** Draft

---

## 1. Problem Statement

Autonomous vehicle (AV) and advanced driver-assistance system (ADAS)
developers face a **safety assurance** challenge: assessing candidate motion
against modeled surroundings across different vehicles and driving contexts.
In assisted driving, the driver remains responsible for driving and
supervising the assistance function as applicable; driver behavior, overrides,
and transitions introduce additional validation requirements.

- Modern AV stacks increasingly rely on learned, black-box planning and prediction models whose behavior cannot be formally or exhaustively verified before deployment.
- Regulators, insurers, and the public require assurance that a planned vehicle trajectory will not result in collision, yet most current validation approaches depend on **statistical testing** across limited real-world or simulated miles rather than a **provable, geometry-based safety guarantee**.
- This verification gap:
  - Slows regulatory approval.
  - Increases liability exposure.
  - Is widely recognized as a central obstacle to scaling AV deployment across passenger, freight, and delivery applications in the United States.

The core problem this endeavor addresses is the need for a reusable,
vendor-independent way to evaluate a candidate trajectory's modeled geometric
clearance against current and forecasted occupancy. Formal bounds and real-time
operation are research goals, not established capabilities or guarantees.

---

## 2. Challenges

| # | Challenge | Description |
|---|-----------|--------------|
| 1 | **Real-time performance under uncertainty** | Sensor fusion (LIDAR, camera, radar) must build a probabilistic 3D occupancy representation fast enough for real-time decision-making, while explicitly modeling sensor noise and uncertainty. |
| 2 | **Forward-looking (predictive) safety, not just reactive** | Static or instantaneous occupancy checks are insufficient; the system must forecast near-future positions of dynamic agents (pedestrians, cyclists, vehicles) to provide a genuine safety margin. |
| 3 | **Formal, provable geometric verification at speed** | Swept-volume collision analysis and signed distance field (SDF) computations are computationally expensive; achieving formal guarantees within real-time constraints is non-trivial. |
| 4 | **Planner- and function-agnostic design** | The engine needs a clean contract for compatible AV planner, ADAS function, and replay candidates without coupling the core to a proprietary stack. |
| 5 | **Graceful degradation / response integration** | Define explicit invalid/unknown and rejected-candidate behavior. Any future warning, intervention, or fallback must be designed and validated with the host system; a geometric verdict alone does not establish a safe response. |
| 6 | **Validation at scale** | The system must be stress-tested against large libraries of synthetic and rare/safety-critical edge-case scenarios before any real-world or partner deployment. |
| 7 | **Cross-platform generalization** | Evaluate passenger ADAS, robotaxis, autonomous trucking, and last-mile delivery with their different vehicle dynamics, sensor configurations, and operating environments. |
| 8 | **Regulatory alignment** | Identify requirements applicable to each AV or ADAS function and jurisdiction; determine whether bounded geometric evidence is useful to the partner's review. |
| 9 | **Industry adoption / trust** | As a vendor-independent layer, the platform must earn integration trust from AV and ADAS developers with different proprietary stacks. |
| 10 | **ADAS driver interaction** | Assess driver overrides, engagement/disengagement, and transitions without equating a predicted driver path with intent or treating assistance as autonomous driving. |

---

## 3. Overview

The proposed endeavor is a multi-component geometric safety verification
platform for AV and ADAS development. Its initial focus is a non-actuating
offline trajectory-checking tool, followed by shadow-mode evaluation only
after review. Real-time operation, broader fusion and forecasting, and
arbitration are future research goals, not current or certified capabilities.
ADAS candidates may be supplied by an assistance function or reconstructed or
predicted for evaluation; they are not necessarily the driver's intended path.

### 3.1 System Components

The following components describe the proposed platform; only the geometric
core has a runnable prototype, and broader modules remain scaffolding or
prototypes.

1. **Spatial Fusion and Voxelization Layer**
   Would ingest multi-sensor input (LIDAR, camera, radar) and construct a probabilistic 3D occupancy representation with explicit sensor noise and uncertainty.

2. **Predictive Occupancy Forecasting Module**
   Would use graph neural networks (GNNs) to forecast near-future positions of dynamic objects (pedestrians, cyclists, other vehicles).

3. **Geometric Collision-Safety Verification Engine** *(core technical contribution)*
   Uses **swept-volume collision analysis** and **signed distance fields (SDFs)** to assess a candidate trajectory's modeled clearance against current and forecasted occupancy. Formal bounds and independent validation remain goals; the prototype is not a certification mechanism.

4. **Constraint-Aware Trajectory Arbitration Module**
   A possible later module for proposing alternatives when a candidate fails verification, subject to separate feasibility and system-safety validation.

5. **Scalable Simulation and Validation Harness**
   Would stress-test the pipeline against synthetic driving scenarios, including rare and safety-critical edge cases.

6. **Cross-Platform Generalization Layer**
   Proposes profiles for assisted-driving passenger vehicles, robotaxis, autonomous trucking, and last-mile delivery platforms, with separate validation for each application.

### 3.2 Regulatory Context

The endeavor is conceived as a direct technical response to the regulatory frameworks governing AV testing and deployment in the U.S.:

- **California Title 13 CCR Articles 3.7 and 3.8** (amended by the California DMV in 2026) require structured, documented safety cases and testing-mileage thresholds before progressing from supervised testing to driverless deployment.
- **State-level frameworks** in Texas, Nevada, Arizona, Florida, Michigan, Pennsylvania, and Georgia impose their own distinct AV testing/operating requirements.
- **Federal level**: NHTSA withdrew its proposed AV STEP evaluation program (June 2026) and replaced it with a case-by-case exemption process under **49 C.F.R. Part 555**, with revised interim guidance issued July 2026.

These AV-specific references do not automatically apply to ADAS. Applicable
requirements depend on the function, vehicle, jurisdiction, and deployment;
obtain qualified review. Geometric evidence may inform a system-specific
assessment but does not establish regulatory compliance or replace a safety case.

---

## 4. Goals

### 4.1 Technical Goals
- Build a real-time probabilistic 3D occupancy representation from multi-sensor (LIDAR/camera/radar) input with explicit uncertainty modeling.
- Develop a GNN-based predictive occupancy forecasting module for dynamic agents.
- Design and independently validate geometric verification (swept-volume + SDF), with formal bounds and real-time performance as research goals.
- Build a constraint-aware, multi-objective trajectory arbitration module for real-time fallback generation.
- Establish a scalable simulation/validation harness with procedurally/generatively created edge-case scenario libraries.
- Generalize and separately validate the framework across assisted-driving passenger vehicles, robotaxis, trucking, and last-mile delivery.
- Define candidate-source and driver-interaction assumptions for ADAS, including overrides and assistance engagement/disengagement.

### 4.2 Strategic / National-Importance Goals
- Provide a **vendor-independent, generalizable safety-verification layer** for AV and ADAS engineering rather than a single company.
- Explore whether reproducible geometric evidence can support partner review under applicable AV or ADAS requirements, without assuming regulatory acceptance or faster approvals.
- Strengthen U.S. competitiveness in AV technology amid growing state-supported international programs.
- Demonstrate transferability of the underlying computational geometry / voxel-based simulation techniques to advanced manufacturing (collision-aware toolpath planning, physical stress simulation), reinforcing relevance to U.S. advanced manufacturing and supply chain resilience.

---

## 5. Implementation Plan

The phases below are proposed work, not implemented capabilities or delivery
commitments. The evidence-gated sequence in the [executive brief](docs/executive-project-brief.md#6-implementation-roadmap-and-stage-gates) prioritizes the offline core.

### Phase 1 — Core Engine and Baseline Validation
- Build the spatial fusion and voxelization layer (multi-sensor ingestion, probabilistic occupancy representation, uncertainty modeling).
- Build and validate the geometric collision-safety verification engine (swept-volume collision analysis, signed distance fields) against established public AV perception and planning benchmark datasets.
- Establish baseline accuracy and real-time performance metrics (latency, throughput, false-positive/false-negative safety verification rates).
- Specify AV and ADAS candidate sources, driver/assistance assumptions, timestamps, and scenario labels; establish separate offline baselines for representative use cases.

### Phase 2 — Predictive Intelligence and Arbitration Integration
- Integrate the graph neural network-based predictive occupancy forecasting module into the pipeline.
- Integrate the multi-objective, constraint-aware trajectory arbitration system.
- Validate the combined pipeline at scale within the scalable simulation and validation harness, including procedurally and generatively created rare/edge-case scenarios.
- Add ADAS scenarios including following, lane changes, cut-ins, driver overrides, and engagement/disengagement. Evaluate modeled trajectories and transitions separately; do not infer warning or intervention safety from clearance alone.

### Phase 3 — Pilot Integration and Generalization
- Pursue an offline pilot with an AV or ADAS developer, simulation provider, or fleet engineering team; advance to non-actuating shadow mode only after review.
- Extend and validate profiles for assisted-driving vehicles and additional AV categories (trucking, last-mile delivery).
- Position the platform as shared, vendor-independent safety infrastructure for the domestic AV industry.
- Review generated evidence against the partner's applicable AV or ADAS requirements with qualified advisors; do not assume the same regulatory process applies to both.

---

## 6. Deployment Plan

### 6.1 Deployment Principles
- **Function-agnostic integration**: accept compatible AV planner, ADAS function, or replay candidates via a documented interface without replacing the host system.
- **Explicit failure behavior**: report rejected, invalid, or unsupported inputs without a success-shaped fallback. A safe response cannot be assumed from a candidate rejection.
- **Staged rollout**: begin with offline validation and, subject to review, non-actuating shadow mode. Any warning, intervention, or vehicle-control use requires a separate safety program, human-factors assessment, and explicit go/no-go.

### 6.2 Deployment Stages
1. **Offline / Benchmark Validation** — Validate the voxelization and verification engine against public AV datasets in a non-production environment.
2. **Simulation-Scale Validation** — Run the integrated pipeline (voxelization + prediction + verification + arbitration) through the simulation and validation harness at scale, covering standard and rare/edge-case scenarios.
3. **Shadow-Mode On-Vehicle Deployment** — Deploy the platform on a partner's vehicle(s) in a non-actuating, "shadow" mode, comparing verification outputs against real-world driving without influencing vehicle control, to validate real-time performance and accuracy under live sensor conditions.
4. **Operational-use decision** — Consider any active AV or ADAS pilot only after independent technical, system-safety, human-factors, legal, and partner review; define authority, driver responsibilities, and failure handling before a separate go/no-go.
5. **Cross-Platform Assessment** — Repeat validation for assisted-driving vehicles and additional AV categories; document which assumptions and results transfer.
6. **Evidence Packaging** — Provide scoped engineering evidence for partner review; applicable AV and ADAS requirements must be assessed separately.

### 6.3 Success Criteria
- Verification engine operates within real-time latency budgets required for on-vehicle deployment.
- Demonstrated reduction in unverified/unsafe trajectory pass-through across benchmark and simulation testing.
- Successful offline and, if approved, non-actuating shadow-mode integration with an AV or ADAS partner.
- Separately reported AV and ADAS scenario coverage, candidate-source assumptions, and driver-interaction limitations.
- Evidence is reproducible and usable in partner engineering review, without assuming regulatory acceptance.

---

## 7. Regulatory Landscape Reference (Supporting Context)

- **California**: Title 13 CCR Articles 3.7 and 3.8, amended by the California DMV in 2026, requiring documented safety cases and enhanced incident reporting. ([California DMV — Autonomous Vehicle Regulations](https://www.dmv.ca.gov/portal/vehicle-industry-services/autonomous-vehicles/california-autonomous-vehicle-regulations/))
- **Other states**: Texas, Nevada, Arizona, Florida, Michigan, Pennsylvania, and Georgia each impose distinct state-level AV testing and operating requirements.
- **Federal**: NHTSA withdrew the proposed AV STEP evaluation program in June 2026, replacing it with a case-by-case exemption process under 49 C.F.R. Part 555, with revised interim guidance issued July 2026.
