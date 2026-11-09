# Executive Project & Commercialization Brief

**Project:** Planner-independent geometric trajectory verification

**Prepared:** 2026-11-09

**Audience:** Project sponsors, technical collaborators, prospective design partners, and commercialization advisors

**Status:** Internal planning document; product, market, and performance claims below are hypotheses or proposed gates unless explicitly identified as measured

For the repository's concise overview and current implementation status, start
with the [README](../README.md). This brief expands the project rationale,
intended scope, adoption thesis, and route to implementation.

## Executive summary

Autonomous-vehicle (AV) developers face a difficult scaling problem: they must
validate increasingly complex automated driving systems across distinct
vehicles, operating domains, cities, and state requirements. Each deployment
may involve a different planner, data pipeline, test environment, and evidence
format. This can lead to duplicated integration and validation effort, make
results difficult to compare, and slow the transfer of engineering lessons
from one program or location to another. More simulation or road testing alone
does not necessarily make the underlying evidence comparable or reusable.

**Project idea:** create a planner-independent geometric trajectory
verification component with a stable, documented interface. Given a candidate
trajectory, a vehicle footprint, a voxelized environment, forecasts for
relevant moving agents, and a requested clearance, it checks the trajectory's
modeled swept volume and reports a verdict, clearance estimate, assumptions,
and diagnostic evidence. The first intended use is offline replay and
engineering validation, followed—only if the evidence supports it—by
non-actuating shadow-mode evaluation.

**Problem addressed:** teams need a repeatable way to ask and inspect a
specific question—does this candidate path keep the configured geometric
clearance from the supplied, modeled scene over the checked horizon? The
proposed component is intended to make that check independent of a particular
planner and easier to reproduce across validation runs. It is not a complete
answer to AV safety, which also depends on perception, prediction, planning,
vehicle behavior, operational controls, and jurisdiction-specific review.

**Why it could support broader AV adoption:** a common technical interface and
repeatable evidence format could reduce the work required to integrate and
compare this class of check across AV programs and operating locations. If
developers, simulation/validation providers, and fleet teams find it useful,
the component could help carry validated engineering practices and test
artifacts from one city or state deployment to another. In that way, it may
contribute to more consistent engineering and lower deployment friction as AV
technology scales nationally.

This is a **pathway hypothesis, not a claim of nationwide impact**. Software
alone cannot standardize AV laws, obtain regulatory acceptance, guarantee
vehicle safety, or cause deployment. The project proposes a reusable
engineering building block that could complement different local processes;
it does not replace them or impose a single safety case on every jurisdiction.

**Current maturity:** the repository contains a runnable NumPy/SciPy prototype
of a voxel signed-distance field, swept-footprint sampling, and a conservative
certification API, along with early tests, a latency sweep, a basic API
scaffold, and an exploratory nuScenes metadata benchmark. It is not a
production-ready AV safety product. Uncertainty treatment is incomplete,
dynamic agents are simplified, most broader platform modules remain
scaffolding, and measured latency on the dataset workload exceeds the
configured 50 ms target. The benchmark results are provisional because
downloaded archive MD5 values do not match the publisher's listed checksums.

**Recommended strategy:** validate the narrow engineering tool and user
problem first. Do not begin with a full perception-to-control platform or an
actuating safety controller. Advancement toward shadow mode or any operational
use requires independent technical review, representative authorized data,
partner-specific validation, a workload-specific performance case, and
legal/IP clearance.

### Decisions sought

1. Approve a focused problem-discovery and verification-core stage rather than
   building the entire proposed perception-to-planning platform at once.
2. Select one initial design-partner profile—recommended: an AV planning,
   simulation, or validation team—and agree on an offline, non-actuating pilot
   objective.
3. Assign owners for technical validation, customer discovery, and IP/data
   review before external product or safety claims.
4. Treat the existing benchmark as exploratory until the dataset integrity
   issue is resolved and the evaluation protocol and results receive further
   review.

## 1. The problem and the adoption challenge

### The problem to tackle

The initial problem is not “make AVs safe” in the abstract. It is the narrower
engineering and integration problem of consistently checking candidate
trajectories against a supplied geometric scene model. A team may need to
answer the same clearance question for multiple planner versions, vehicles,
simulation runs, or deployment locations, but may lack a reusable,
planner-independent component and a consistent way to record the assumptions
and result.

The broader adoption challenge is that AV deployments are developed by
different organizations and reviewed within different operational and
jurisdictional contexts. A successful deployment in one location does not
automatically establish suitability in another: road conditions, operating
domains, fleet operations, system configurations, local rules, and validation
expectations can differ. This project does not resolve those differences. It
targets one reusable technical activity—geometric trajectory checking—that
could be integrated into different development and validation pipelines.

### Where the current process may be fragmented

These are problem hypotheses to validate with customers and partners, not
universal claims about every AV program:

- Trajectory checks may be tightly coupled to an individual planner or
  simulation stack, increasing the effort to reuse them elsewhere.
- Teams may use different scene, vehicle-footprint, forecast, and report
  representations, making results harder to reproduce or compare.
- A geometric verdict may be difficult to audit if assumptions, input
  versions, margins, and failure cases are not recorded with it.
- Validation from one vehicle or operating location may require repeated
  integration work before it can inform another program.
- Safety engineering teams must combine technical results with their own
  system-level analyses and local obligations; an isolated algorithm result
  is not a portable safety case.

### The proposed contribution to AV adoption

The project can contribute to broader, more consistent adoption only through a
chain of outcomes:

1. **Define a portable technical contract.** Specify the trajectory, vehicle,
   occupancy, forecast, time-base, coordinate-frame, margin, and result
   semantics so a compatible component can be integrated without replacing a
   planner.
2. **Make checks reproducible.** Version inputs and configurations; retain
   verdicts, modeled clearances, diagnostics, and known assumptions.
3. **Validate across representative scenarios.** Use common replay procedures,
   geometry references, baselines, and transparent metrics to compare results
   across configurations and locations.
4. **Reduce repeated engineering effort.** If the interface and evidence
   artifacts prove useful, developers and tool vendors can reuse integrations
   and validation procedures rather than recreate this specific check for
   each program.
5. **Support local review rather than replace it.** A consistent technical
   artifact may help teams organize and compare evidence, while each operator
   and jurisdiction retains its own safety, legal, and regulatory processes.

The intended standardization is therefore **voluntary technical
interoperability and repeatable engineering practice**, not a new legal or
regulatory standard. Nationwide relevance is an outcome to earn through
multi-partner evaluation and adoption; it cannot be claimed from the current
prototype or a single dataset.

### Adoption theory of change and proof points

| Adoption friction to investigate | Potential project contribution | Evidence needed before claiming value |
|---|---|---|
| Teams repeatedly adapt trajectory checks to different planners or simulation tools | A stable, documented interface plus narrow adapters can make the check reusable without replacing those systems | Independent integrations with more than one partner stack; setup effort and unsupported cases recorded |
| Results are difficult to reproduce or compare across runs | Versioned inputs/configuration, deterministic replay, and common result fields can make runs more inspectable | Partners independently reproduce a result and agree that the artifacts are useful in their review process |
| A check validated in one vehicle or operating context may not transfer directly | Separate the reusable geometry contract from vehicle footprints, local scene inputs, operating-domain scenarios, and partner-specific acceptance rules | Repeat evaluations across different vehicles or operating domains; document what transfers and what must be revalidated |
| Engineering evidence may be fragmented across organizations and locations | A common technical vocabulary and export format could make a bounded class of evidence easier to exchange | Evidence consumers confirm the format is interpretable; no assumption that agencies accept it or that legal requirements are uniform |

This theory connects the product to adoption indirectly: if the component
reduces duplicated engineering work and helps teams create reproducible
evidence, it could lower one source of friction in developing and expanding AV
services. Whether this affects deployment timelines, costs, public confidence,
or market adoption is an empirical question for multi-partner pilots. Many
other technical, operational, economic, social, and regulatory factors remain
outside the project's control.

For cross-city and cross-state use, the design goal is a **portable core with
explicitly configurable context**, not one universal configuration. Vehicle
geometry, map/forecast inputs, operating design domain (ODD), scenario
coverage, and local review criteria may vary. Each deployment would need to
document its own assumptions and validation; reuse of the interface does not
transfer approval from one location to another.

### Mission and vision

- **Mission:** make trajectory-level geometric checks easier to integrate,
  reproduce, inspect, and validate across planning systems.
- **Vision:** contribute a reusable verification component and evidence
  workflow that can complement AV development and validation across programs
  and operating locations.

### Product principles

1. **Narrow claims:** report the modeled assumptions, bounds, and failure
   conditions with every result.
2. **Planner independence:** accept a documented trajectory and scene contract;
   avoid planner-specific logic in the verification core.
3. **Fail visibly:** malformed inputs, missing coverage, stale data, and
   out-of-domain requests produce explicit errors or an unsafe/unknown result,
   never a success-shaped fallback.
4. **Reproducibility:** retain versioned inputs, configuration, software build,
   and machine/workload details for each evaluation.
5. **Evidence before actuation:** begin with offline and shadow-mode use; do not
   connect to control authority until the safety and operational case supports
   it.

### What the project is—and is not

| It is | It is not today |
|---|---|
| A research prototype for geometric trajectory checks and a potential reusable engineering component | A certified safety system or proof of vehicle-level safety |
| A proposed portable interface for planner, simulation, and validation workflows | A replacement for perception, prediction, planning, control, or local review |
| A basis for repeatable offline and future shadow-mode experiments | A demonstrated hard real-time in-vehicle component |
| A possible contributor to more consistent technical evidence across programs | A national AV deployment mandate, regulatory standard, or approval shortcut |

## 2. Scope: what implementation includes

### Core verification function

At its core, the product accepts a time-indexed candidate trajectory and a
description of the modeled environment and vehicle. It evaluates whether the
modeled vehicle swept volume meets the requested geometric clearance from
static occupancy and supported dynamic-agent forecasts. A useful result should
include more than a Boolean: it should identify the minimum modeled clearance,
where/when the limiting interaction occurs, the assumptions and bounds applied,
and any missing or invalid coverage that prevents a meaningful verdict.

### Target implementation package

The recommended initial package is an offline verification SDK and replay
workflow, comprising:

- **Versioned integration contract:** documented units, coordinate frames,
  timestamp rules, trajectory and footprint representations, occupancy and
  forecast semantics, margin configuration, and output/error behavior.
- **Geometric verification core:** distance-field queries, swept-volume
  coverage, static occupancy checks, supported dynamic-agent geometry, and
  explicit discretization/error allowances.
- **Input validation and result semantics:** validation for malformed,
  incomplete, stale, out-of-bounds, or unsupported inputs; explicit unsafe or
  unknown/error outcomes instead of silently treating missing information as
  safe.
- **Replay and evaluation tooling:** deterministic scenario ingestion,
  repeated candidate evaluation, reference-geometry comparisons, baselines,
  and workload-level measurements.
- **Evidence output:** machine-readable records and reviewer-oriented reports
  that preserve software/configuration versions, assumptions, verdicts,
  margins, diagnostics, and performance measurements.
- **Partner adapters and examples:** initially one agreed data format and
  workflow; broader integrations only when validated demand justifies them.
- **Quality and release practices:** test suites, regression cases, supported
  environment definitions, dependency inventory, documentation, and
  reproducible releases.

The [README](../README.md) summarizes what is implemented today and links to
the [technical report](./technical-report.md) for algorithm and measurement
details. The target package above is a product-development scope, not a claim
that every component is already present in the repository.

### Current implementation versus remaining platform gaps

| Capability | Current state | Gap to a credible product |
|---|---|---|
| Voxel SDF and conservative clearance | NumPy/SciPy prototype with explicit discretization allowances | Independent review, broader geometry/property tests, numerical and supported-domain specification |
| Swept-volume checking | Sampled extruded footprint and interpolated poses | Broader footprint/vehicle models, independent geometry oracle, validated bounds for supported motion and input conditions |
| Dynamic agents | Forecast centers interpolated and represented by configurable circles | Agent dimensions, uncertainty sets, confidence semantics, time synchronization, stale/missing forecast behavior |
| Occupancy input | Thresholded voxel-grid contract | Calibrated probability/uncertainty treatment, map provenance, sensor and fusion integration; fusion is not currently a production subsystem |
| Trajectory input | Shared project types and prototype verification interface | Stable, versioned external schema and supported planner/simulator adapters |
| Evidence and replay | Benchmark scripts and JSON output for selected experiments | Scenario registry, deterministic replay, provenance, report versioning, reproducible CI workflow |
| Performance | Prototype latency sweeps | Profiling, caching/algorithm improvements, tail-latency characterization, workload-specific performance envelope; current nuScenes workload exceeds the configured 50 ms target |
| Evaluation | Analytic tests, finite brute-force cases, and selected nuScenes metadata run | Resolve archive checksum mismatch, expand authorized scenarios, improve time-continuous ground truth, compare independently implemented baselines, and obtain external review |
| System-level components | API/frontend and several planning/perception modules are scaffolding or prototypes | Not in the initial scope; do not imply end-to-end AV capability |

### Explicitly out of scope for the initial product

- Building a full perception, forecasting, planning, arbitration, or
  vehicle-control stack.
- Certifying a vehicle, satisfying a regulator, or proving a complete system
  safe.
- Actuating braking or steering or serving as the sole runtime safety layer.
- Harmonizing laws, regulations, permits, or operating requirements among
  cities, states, or nations.
- Supporting every vehicle type, planner, sensor stack, operating domain, or
  jurisdiction at launch.
- Selling or redistributing restricted dataset content.

## 3. Objectives

**Technical objectives**

- Define and validate a precise supported input contract, soundness
  assumptions, and operating envelope.
- Verify geometric bounds with independent reference implementations,
  adversarial/property-based tests, and external review.
- Reduce false rejections while preserving conservative treatment of modeled
  collisions and explicit handling of unknown input.
- Measure throughput, p50/p95/p99 latency, memory, and failure behavior on
  partner-representative workloads and hardware.
- Add and validate uncertainty sets for ego and agents; define handling for
  stale, incomplete, or inconsistent inputs.

**Product objectives**

- Confirm a specific user problem, workflow, and economic buyer through
  structured interviews.
- Deliver a usable offline SDK/CLI, replay workflow, and integration example
  before expanding into a hosted service or full platform.
- Secure at least one design partner willing to define acceptance criteria and
  provide authorized, representative data or replay scenarios.

**Commercial objectives**

- Identify a viable initial buyer, procurement path, and deployment model
  without assuming market size, demand, or willingness to pay.
- Establish ownership, licensing, data rights, and publication permissions.
- Test willingness to pay before setting a price or making a market-size claim.

## 4. Potential customers and stakeholders

These are customer hypotheses, not confirmed customers.

| Segment | Likely user / champion | Possible buyer | Initial need to test |
|---|---|---|---|
| AV developers and autonomous fleet technology teams | Planning, prediction, simulation, and safety engineers | Engineering director, VP autonomy, or safety leader | Independent trajectory checks, regression analysis, and shadow-mode diagnostics |
| AV simulation and validation vendors | Scenario, test, and integration engineers | Product or business-unit leader | A verification component and repeatable metrics to complement scenario testing |
| Autonomous trucking and delivery developers | Motion-planning and vehicle-integration teams | Program or engineering leadership | Vehicle-specific geometry, long-horizon interactions, and validation evidence |
| Robotics and industrial mobile-robot developers | Navigation and safety engineers | Robotics engineering or product leadership | Adapted geometric clearance checks in constrained operating domains |
| Research labs and universities | Principal investigators and graduate researchers | Lab or sponsored-program leadership | Reproducible algorithms, evaluation harnesses, and research collaboration |
| Insurers, auditors, and safety consultants | Technical assessors | Practice or program leadership | Potentially useful evidence inputs; not a substitute for independent safety assessment |

**Recommended beachhead:** AV developer simulation, planning, or validation
teams. They can evaluate an offline tool without immediate vehicle-control
authority, and are positioned to provide the integration and scenario feedback
needed to test the product thesis.

**Defer as initial customers:** regulators and consumers. Regulators are not
assumed to endorse or purchase a commercial verifier, and consumer-facing
safety claims would be premature.

### Discovery questions

Conduct structured interviews before committing to a commercial model:

- How are candidate trajectories currently checked and regression-tested?
- What failure modes and false alarms create the most engineering cost?
- Which data formats, frameworks, runtime environments, and evidence formats
  must be supported?
- Is the desired use offline analysis, CI regression, simulation-time checking,
  shadow mode, or active runtime gating?
- What latency, throughput, availability, and integration effort are acceptable
  for each use?
- Who owns the budget, security review, safety approval, and procurement?
- What evidence would justify a paid pilot, and what would block adoption?
- Can the partner provide authorized scenarios and a reference labeling process?

## 5. Commercial packaging hypotheses

The initial deliverable remains the offline verification SDK and replay
workflow defined in [Section 2](#2-scope-what-implementation-includes). It
should be positioned as **engineering validation software** until the evidence
supports a more safety-critical use. Packaging and pricing below are options
to test with prospective users, not established demand or commitments.

### Possible later offerings

1. **Commercial SDK:** per-program or annual engineering license with
   integration support and supported releases.
2. **Enterprise validation suite:** scenario replay, regression dashboards,
   team workflows, and evidence management, deployed on-premises or in a
   customer's controlled cloud.
3. **Integration and validation services:** paid support to integrate with a
   customer's data formats and establish workload-specific acceptance tests.
4. **Research or evaluation partnerships:** scoped, non-production
   collaborations to improve independent validation and broaden supported
   scenarios.

## 6. Implementation roadmap and stage gates

The timing below is indicative and gated by evidence, staffing, and partner
access. It is not a delivery commitment.

| Stage | Indicative duration | Scope and deliverables | Exit gate |
|---|---:|---|---|
| 0. Establish the project | 0–4 weeks | Customer interviews; product requirements; threat/hazard and intended-use boundary; IP, data, and licensing review; benchmark integrity investigation | Named user/problem, authorized evaluation data, and written intended-use statement |
| 1. Harden the core | 1–3 months | Input validation; geometry reference tests; uncertainty and map-boundary policy; reproducible CLI; profiling; failure/unknown semantics | Independent code/algorithm review; test evidence; bounded and documented operating envelope |
| 2. Expand offline validation | 2–5 months | Larger authorized scenario set; time-aware ground truth; relevant baselines; statistical reporting; CI/replay workflow; evidence export | Pre-registered metrics and evaluation protocol pass review; limitations disclosed |
| 3. Design-partner integration | 4–9 months | Partner adapter; workload-specific profiling; offline and shadow-mode replay; issue triage; security and deployment review | Partner signs off on scoped shadow-mode objectives; no control actuation |
| 4. Productization decision | 6–12+ months | Supported packaging, release process, documentation, support model, commercial/legal terms; decide whether runtime use merits a separate safety program | Evidence of adoption and a funded use case; independent safety and legal reviews; explicit go/no-go |
| 5. Cross-program portability assessment | After initial partner evidence; timing TBD | Replicate the agreed protocol with additional independent organizations, vehicle/configuration profiles, and—where authorized—distinct operating contexts; document reusable versus local validation work | Independent partners can integrate and interpret results; portability limits are explicit; do not infer regulatory equivalence or nationwide readiness |

### Technical workstreams

1. **Geometry and soundness:** independently review SDF lower bounds, footprint
   coverage, pose/time interpolation, agent geometry, numerical tolerances, and
   edge conditions.
2. **Scene/model inputs:** define coordinate frames, time synchronization,
   occupancy confidence semantics, agent dimensions, uncertainty, stale-data
   behavior, and provenance.
3. **Performance:** profile before optimizing; reuse or cache valid distance
   fields; reduce unnecessary allocations; evaluate a native implementation
   only after behavior is stable and benchmarked.
4. **Evaluation:** build exact or high-fidelity reference labels, adversarial
   cases, diverse scenarios, and baseline implementations with comparable
   output semantics.
5. **Developer experience:** stabilize API/schema versioning, errors, install
   paths, CI examples, reports, and integration support.
6. **Security and operations:** document trust boundaries, dependency and
   software-bill-of-materials practices, logging, reproducible builds, and
   on-premises/data-retention options.

## 7. Validation and evidence plan

### Validation ladder

1. **Unit and analytic checks:** known shapes, distances, coordinate
   transformations, degenerate geometries, numeric extremes, and invalid input.
2. **Independent geometry oracle:** compare against a separately implemented
   exact or higher-fidelity geometry engine; test randomized and adversarial
   trajectories and obstacles.
3. **Property and metamorphic tests:** invariance to coordinate translation and
   rotation where applicable; monotonicity under increased obstacle size,
   vehicle size, requested margin, or uncertainty.
4. **Scenario replay:** logged and synthetic scenarios with explicit temporal
   ground truth; include collisions, near misses, occlusions, map edges, and
   data dropouts.
5. **Baseline comparison:** include simple distance and TTC checks and a
   properly integrated open-source RSS implementation if platform and license
   review permit. Compare only metrics with consistent meaning.
6. **Performance and reliability:** report p50/p95/p99 latency, throughput,
   memory, timeout and error behavior, cold/warm runs, and hardware/software
   configurations. Include complete end-to-end costs, not just a hot loop.
7. **Independent review:** invite an external geometry/safety reviewer to
   inspect assumptions, test design, results, and claims before any
   safety-critical positioning.

### Proposed metrics

| Metric | Definition / reporting guidance |
|---|---|
| False-accept rate | Modeled collision cases returned safe / all labeled collision cases; report numerator, denominator, scenario grouping, and confidence interval |
| False-reject rate | Labeled clear cases returned unsafe / all labeled clear cases |
| Indeterminate/error rate | Cases with insufficient or invalid input, timeout, or explicit unknown verdict / all attempted cases |
| Clearance error | Absolute difference between reported conservative clearance and reference geometry clearance; report distribution and worst case |
| Bound violations | Count and magnitude of cases where the claimed lower bound exceeds reference clearance; target zero in reviewed test scope |
| Latency | p50/p95/p99 and maximum observed under defined workload, hardware, grid size, horizon, agent count, and warm/cold state |
| Robustness | Behavior under missing, stale, malformed, out-of-bounds, and inconsistent inputs |
| Integration effort | Engineering time, supported-format gaps, and operational failures during partner integration |
| Product value | User-reported time saved, defects found, regression coverage, and willingness to pay; measure with partner evidence |

**Statistical caution:** zero observed false accepts is not proof of zero risk.
As a rough binomial rule, zero failures among \(n\) independent positive cases
gives an approximate one-sided 95% upper bound of \(3/n\). Cases sampled from
the same scenes are correlated and do not count as fully independent evidence.
Any acceptance threshold must be set with an independent reviewer and the
intended application; formal geometric soundness and system-level safety
assurance require evidence beyond a dataset metric.

### Current evidence and gaps

- Analytic geometry tests and 128 deterministic rectangle/voxel comparisons
  reported zero false accepts in those finite test cases.
- The selected nuScenes trainval metadata run covered 300 candidates across
  100 of 850 scenes and found 0/21 geometric-verifier false accepts, with a
  45.2% false-reject rate. This is exploratory evidence, not a population
  estimate.
- The archive passed gzip validation but its computed MD5 did not match the
  published checksum. The results are therefore provisional.
- On that dataset workload, measured geometric-verifier latency was 110.066 ms
  p50 and 1,192.421 ms p99. The current implementation does not meet the
  configured 50 ms target for this workload.
- The in-repo RSS-inspired baseline is not an execution of an RSS library; it
  had 9 false accepts among 21 labeled collisions in that selected run.
- The dataset license is non-commercial. Do not use it to imply commercial
  data rights; obtain legal review before publishing or commercializing
  derivatives.

See the [technical report](./technical-report.md) for methods, measurements,
and caveats. Results must be regenerated from a verified archive before being
used externally.

## 8. Commercialization and engagement plan

### Design-partner engagement

1. Recruit 5–10 engineering stakeholders for discovery interviews; do not
   present the prototype as a certified product.
2. Select one design partner with an offline validation use case and authorized
   data access.
3. Agree in writing on input format, operating assumptions, test scenarios,
   output interpretation, data handling, confidentiality, IP, and publication
   rights.
4. Run offline replay first; progress to shadow mode only after the partner
   approves the validation protocol.
5. Review the results jointly, record false accepts/rejects and integration
   friction, and make a productization decision from observed value.

### Commercial hypotheses to test

- Teams may pay for earlier discovery of trajectory-verification regressions
  and reduced manual test effort.
- A customer may prefer an on-premises SDK or controlled-cloud deployment
  because scenario and vehicle data are sensitive.
- Engineering integration and support may be as important as algorithm
  licensing for an initial sale.
- Evidence exports may assist internal engineering review, but must not be
  marketed as regulator acceptance or a safety-case substitute.

### Route to market

- Begin with founder-led technical discovery and a small number of narrowly
  scoped pilots.
- Demonstrate value through customer-authorized replay scenarios and
  reproducible reports.
- Use technical workshops, publications, and research collaborations only
  after IP and dataset rights are cleared and claims are independently checked.
- Consider partnerships with simulation, verification, or AV tooling vendors
  after the SDK contract and integration model stabilize.
- Do not publish customer names, scenario data, benchmark results, or
  comparative claims without written permission and legal review.

### Pricing and business model

No defensible price or market-size estimate exists yet. Test willingness to
pay for:

- annual developer/SDK licenses;
- program- or vehicle-platform licenses;
- enterprise on-premises deployments and support;
- paid integration and validation services.

Compare these against customer procurement expectations, support burden,
deployment costs, liability exposure, and the value evidenced in pilots. Avoid
pricing tied to safety outcomes until measurement and contractual definitions
are mature.

## 9. Governance, legal, and data management

- **IP and ownership:** establish code authorship, employment/contractor
  assignment, prior-work boundaries, patent/filer obligations, and publication
  review with qualified counsel. Resolve the noted GM/HP terms before
  commercial distribution, publication, or adding a software license.
- **Dataset rights:** nuScenes is restricted to non-commercial use under its
  terms. Keep archives and derived outputs outside the repository unless
  counsel and dataset terms explicitly allow publication. Resolve archive
  checksum discrepancies before treating results as final.
- **Open-source dependencies:** inventory licenses, notice obligations, and
  compatibility before distributing binaries or source. Review any RSS
  library independently before integration.
- **Safety claims:** use terms such as “prototype,” “modeled clearance,” and
  “engineering validation.” Do not claim certification, regulatory approval,
  guaranteed collision avoidance, or replacement of a safety case.
- **Customer data:** define access control, retention, deletion, anonymization,
  incident handling, and publication permissions before accepting partner
  datasets.
- **Quality and traceability:** version source, builds, dependencies,
  configuration, benchmark protocol, and result artifacts. Preserve dated
  project history; do not rewrite or squash commits used as project evidence.

This section is a project checklist, not legal advice.

## 10. Risks and mitigations

| Risk | Why it matters | Mitigation / decision trigger |
|---|---|---|
| False accepts outside tested assumptions | Could create misplaced safety confidence | Constrain intended use, fail closed, use independent geometry oracles, and maintain explicit operating limits |
| Excessive false rejects | Limits utility and can encourage bypassing the tool | Improve geometry/model fidelity and optimize only while preserving verified lower bounds |
| Runtime latency and tail spikes | Current dataset p99 is above one second | Profile end-to-end; define a workload budget with partner; do not enable runtime gating until it is met reliably |
| Dataset integrity/license uncertainty | Undermines reproducibility and commercial use | Resolve checksums, document provenance, use authorized data, and separate non-commercial research from commercial validation |
| Customer integration burden | May prevent adoption even if core geometry is useful | Start with one format and workflow, use discovery, and measure integration effort |
| Liability and overclaiming | Safety-critical use has high consequence | Keep early product non-actuating; require counsel and independent safety review before changing intended use |
| IP ownership or licensing conflict | Could block publication or commercialization | Clear ownership and license before external distribution |
| Full-platform scope creep | Increases cost before core value is proven | Gate perception, GNN forecasting, arbitration, and cross-platform work on partner evidence |
| Benchmark selection bias | Small selected sample may mislead | Expand scenario diversity, document selection, report denominators and intervals, and use external review |

## 11. Operating metrics and immediate actions

### Monthly project dashboard

- Customer discovery: interviews completed, validated pain points, design
  partner commitments, and willingness-to-pay evidence.
- Engineering: test coverage by risk area, independent-oracle discrepancies,
  false accepts/rejects, unknown/error rate, and reproducibility.
- Performance: p50/p95/p99 latency, memory, workload definition, and target
  attainment.
- Product: integration hours, supported inputs, usability feedback, and
  feature requests tied to a confirmed workflow.
- Governance: resolved IP/data questions, dependency/license inventory, and
  approved external claims.

### Next 30 days

1. Write a one-page intended-use and prohibited-use statement for the prototype.
2. Interview 5–10 target users using the discovery questions in this brief.
3. Ask the dataset publisher or official support channel to clarify the
   metadata and mini archive checksum mismatches; retain archive hashes and
   provenance records.
4. Profile the verifier on representative grid/path workloads and identify
   which costs dominate p99 latency.
5. Commission a focused independent review of the SDF and swept-volume
   conservatism claims.
6. Create an IP, data-rights, and dependency-license review checklist with
   qualified counsel/advisors.
7. Select one partner-oriented offline workflow and define measurable
   acceptance criteria before adding broader platform features.

## 12. Source documents

- [Project plan](../PROJECT_PLAN.md)
- [Repository status and limitations](../README.md)
- [Source architecture](../src/README.md)
- [Prototype technical report](./technical-report.md)
