# Executive Project & Commercialization Brief

**Project:** Planner-independent geometric trajectory verification

**Prepared:** 2026-11-07

**Audience:** Project sponsors, technical collaborators, prospective design partners, and commercialization advisors

**Status:** Internal planning document; product, market, and performance claims below are hypotheses or proposed gates unless explicitly identified as measured

## Executive summary

This project explores an independent software layer that checks whether a
candidate vehicle trajectory maintains a requested geometric clearance from
modeled static occupancy and forecasted moving agents. Its potential value is
to help autonomous-vehicle (AV) engineering teams find unsafe candidate paths,
compare planner changes, and produce reproducible verification evidence without
replacing their planner.

The repository currently contains a runnable NumPy/SciPy prototype of a voxel
signed-distance field, swept-footprint sampling, and a conservative
certification API. It also contains early tests, a latency sweep, a basic API
scaffold, and a nuScenes metadata benchmark. It does **not** contain a
production-ready AV safety product: uncertainty is incomplete, dynamic agents
are simplified, most system modules remain scaffolding, performance is not
real-time on the dataset workload, and the benchmark has unresolved archive
checksum discrepancies.

**Recommended strategy:** validate a narrow engineering product first: an
offline and shadow-mode trajectory verification SDK with a clear input/output
contract, deterministic replay, diagnostics, and evidence export. Start with
developer tools and validation workflows, not an actuating in-vehicle safety
controller or a claim of regulatory certification. Advance toward operational
use only after independent technical validation, partner-specific integration
tests, legal/IP clearance, and safety-engineering review.

### Decisions sought

1. Approve a focused discovery and validation stage rather than building the
   entire proposed perception-to-planning platform at once.
2. Select one initial design-partner profile (recommended: AV planning or
   simulation/validation team) and agree on a non-actuating pilot objective.
3. Assign owners for technical validation, customer discovery, and IP/data
   review before any external product or safety claims.
4. Treat the current benchmark as exploratory only until the dataset integrity
   issue is resolved and the evaluation is expanded and independently reviewed.

## 1. Project establishment

### Purpose

Develop and evaluate a planner-independent geometric checking component that
answers a limited, auditable question:

> Given a specific trajectory, vehicle footprint, occupancy representation,
> agent forecasts, and explicit model assumptions, does the modeled geometry
> maintain at least the requested clearance over the checked horizon?

The answer applies only to the supplied inputs and supported model. It is not a
claim that the scene was perceived correctly, that forecasts are accurate, or
that a vehicle is safe to operate.

### Proposed mission and vision

- **Mission:** make trajectory-level geometric checks easier to integrate,
  reproduce, inspect, and validate across planning systems.
- **Vision:** provide a trusted verification component and evidence workflow
  that can complement AV development and validation pipelines.

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
| A research prototype for geometric trajectory clearance checks | A certified safety system or proof of vehicle-level safety |
| A potential SDK/tooling layer for planner and validation teams | A replacement for perception, prediction, planning, or vehicle control |
| A basis for repeatable offline and future shadow-mode experiments | A demonstrated hard real-time in-vehicle component |
| A project with explicit assumptions and testable engineering hypotheses | A verified regulatory-compliance solution or safety-case approval shortcut |

## 2. Problem, product thesis, and objectives

### Problem hypothesis

AV developers may benefit from an independently testable geometric check that
can inspect candidate trajectories against scene representations and expose
where clearance is lost. Existing product value and willingness to pay have
not yet been established through customer discovery.

### Product thesis

The first useful product is likely not a full autonomy stack. It is a developer
and validation tool that can:

- ingest a candidate trajectory, vehicle dimensions, occupancy grid, and
  dynamic-agent forecasts;
- calculate a conservative modeled clearance and safe/unsafe/indeterminate
  verdict;
- identify the time and spatial region driving the verdict;
- run consistently on replayed scenarios and compare versions or baselines;
- export machine-readable and human-reviewable evidence with assumptions and
  provenance.

The existing verifier addresses part of this thesis. Replay management,
validated uncertainty handling, robust evidence packaging, integration
hardening, and a supported operational performance envelope remain to be built
or demonstrated.

### Objectives

**Technical objectives**

- Establish a precise supported input contract and soundness assumptions.
- Verify geometric bounds with independent reference implementations and
  adversarial property-based tests.
- Reduce false rejections while preserving conservative treatment of modeled
  collision cases.
- Measure throughput, p50/p95/p99 latency, memory, and failure behavior under
  representative workload and hardware conditions.
- Add uncertainty sets for ego and agents and explicitly handle stale,
  incomplete, or inconsistent inputs.

**Product objectives**

- Confirm a specific user problem, workflow, and economic buyer through
  structured interviews.
- Deliver a usable offline SDK/CLI and integration example before expanding
  into a hosted service or full platform.
- Secure at least one design partner willing to define acceptance criteria and
  provide authorized, representative data or replay scenarios.

**Commercial objectives**

- Identify a viable initial buyer, procurement path, and deployment model.
- Establish ownership, licensing, data rights, and publication permissions.
- Test willingness to pay before setting a price or making a market-size claim.

## 3. Potential customers and stakeholders

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

## 4. Product scope and packaging hypotheses

### Initial product: verification SDK and replay tool

Recommended first deliverable:

- Python library and command-line runner for offline/replay evaluation;
- documented schemas for trajectories, footprints, occupancy, and forecasts;
- clear verdicts including safe, unsafe, and invalid/unknown where appropriate;
- clearance and violation diagnostics with model/configuration provenance;
- baseline comparisons and exportable JSON/CSV reports;
- deterministic scenario replay and a partner-facing integration example;
- versioned behavior, tests, and explicit supported platform/workload limits.

This should be positioned as **engineering validation software** until the
evidence supports a more safety-critical use.

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

These are business-model options to test, not commitments or pricing.

### Explicitly out of scope for the first product

- Building a full perception, prediction, planning, or vehicle-control stack.
- Claiming to certify a vehicle, satisfy a regulator, or establish a
  vehicle-level safety guarantee.
- Actuating braking or steering or serving as the sole runtime safety layer.
- Supporting every vehicle type, sensor stack, operating domain, or planner.
- Selling or redistributing restricted dataset content.

## 5. Implementation roadmap and stage gates

The timing below is indicative and gated by evidence, staffing, and partner
access. It is not a delivery commitment.

| Stage | Indicative duration | Scope and deliverables | Exit gate |
|---|---:|---|---|
| 0. Establish the project | 0–4 weeks | Customer interviews; product requirements; threat/hazard and intended-use boundary; IP, data, and licensing review; benchmark integrity investigation | Named user/problem, authorized evaluation data, and written intended-use statement |
| 1. Harden the core | 1–3 months | Input validation; geometry reference tests; uncertainty and map-boundary policy; reproducible CLI; profiling; failure/unknown semantics | Independent code/algorithm review; test evidence; bounded and documented operating envelope |
| 2. Expand offline validation | 2–5 months | Larger authorized scenario set; time-aware ground truth; relevant baselines; statistical reporting; CI/replay workflow; evidence export | Pre-registered metrics and evaluation protocol pass review; limitations disclosed |
| 3. Design-partner integration | 4–9 months | Partner adapter; workload-specific profiling; offline and shadow-mode replay; issue triage; security and deployment review | Partner signs off on scoped shadow-mode objectives; no control actuation |
| 4. Productization decision | 6–12+ months | Supported packaging, release process, documentation, support model, commercial/legal terms; decide whether runtime use merits a separate safety program | Evidence of adoption and a funded use case; independent safety and legal reviews; explicit go/no-go |

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

## 6. Validation and evidence plan

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

## 7. Commercialization and engagement plan

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

## 8. Governance, legal, and data management

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

## 9. Risks and mitigations

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

## 10. Operating metrics and immediate actions

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

## 11. Source documents

- [Project plan](../PROJECT_PLAN.md)
- [Repository status and limitations](../README.md)
- [Source architecture](../src/README.md)
- [Prototype technical report](./technical-report.md)
