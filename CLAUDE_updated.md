# CLAUDE.md --- Property Capture to Dimensioned Floor Plan + Damage Assessment

## Purpose

Engineering contract for the Applied AI case study: turn phone captures
into a stitched, dimensioned floor plan with visible-damage assessment,
explicitly qualified concealed-damage risk flags, calibrated measurement
uncertainty, and reproducible evaluation.

**Authority:** The original case-study PDF and evaluator clarifications
control exact deliverables, score weights, and thresholds. Verify
candidate thresholds below against that source before treating them as
authoritative. Track unresolved interpretations in
`docs/requirements/assumptions.md`.

## How Claude must work

1.  Inspect the repository, original case-study PDF, existing code,
    tests, and actual ZIP contents before coding.
2.  Create `docs/requirements/traceability.csv`:
    `requirement_id, source, exact_requirement, implementation, evidence, status, blocker, owner`.
3.  Label claims `SOURCE`, `OBSERVED`, `ASSUMPTION`, or `PROPOSED`.
    Never claim a metric passed without reproducible evidence.
4.  Preserve a working end-to-end baseline; make incremental, testable
    changes.
5.  Never fabricate dimensions, poses, calibration, damage, or
    confidence. Use `unknown`, `not_measurable`, or abstention when
    evidence is insufficient.
6.  Never use supplied ground-truth poses as predicted poses. Use them
    only for evaluation/oracle analysis, clearly isolated. If prohibited
    by the case study, production-path access is an automatic failure.
7.  Keep raw data immutable. Save run-specific artifacts, input hashes,
    config, software versions, commit, command, and random seeds.
8.  Do not hard-code sample-specific answers. Prefer explainable,
    validated methods.
9.  Before commits, run formatting, static checks, unit/integration
    tests, smoke test, and relevant gates; record exact commands and
    outcomes.

## Objective and common output

Produce: - stitched room/property floor plan and room connectivity
graph; - wall dimensions and detected doors/windows, with units and
uncertainty; - room ceiling height only where measurable; - visible
damage annotations with class, location, severity/confidence, and
evidence; - concealed-damage **risk flags** only from explicit
documented rules/evidence, never as confirmed hidden defects; -
machine-readable output, rendered plan/report, overlays, and
reproducibility bundle.

Use one normalized schema across input tiers. Suggested `property.json`
fields:
`schema_version, run_id, capture_id, units, input_tier, software_commit, config_hash, rooms[], connections[], measurements[], warnings[], unobservable[], evaluation[]`.
Each room includes polygon, floor elevation, ceiling height/status,
walls, openings, damage, and quality. Each measurement includes value,
unit, interval, method, calibration status, evidence IDs, and quality
flags. Never encode unavailable values as zero.

## Mandatory scope tracks

### Track A --- Supplied-data RGB-D baseline

Start with `single_room`, `single_scan_floor_only`, and
`single_scan_with_ceiling`. The project review describes
depth/confidence PNG sequences, `rgb.mp4`, and `camera_matrix.csv`,
`imu.csv`, `odometry.csv`; inspect the actual archives to confirm names,
counts, formats, units, and conventions.

### Track B --- Full case-study compliance

Do not treat the RGB-D baseline as full completion. Trace and implement
or explicitly block: 1. **Photo-only:** 2--8 still images without depth
or supplied poses; use SfM/MVS or equivalent. Resolve metric scale via
reference/user measurement or report scale as unknown. 2. **Video:**
handheld walkthrough tracking, drift management, loop closure where
feasible, and geometry extraction; disclose monocular scale limitations.
3. **LiDAR/depth:** calibrated depth, units, invalid-pixel masks, sensor
transforms, confidence. 4. **Common schema:** same field meanings across
modalities. **Published evaluator schema must be obtained and validated
before claiming compliance; never claim official schema satisfaction
without it.** 5. **Multi-room:** at least three rooms plus connector where
required; correct adjacency, shared-wall consistency, no
duplicate/overlapping interiors. **Benchmark composition (per PDF): one
multi-room capture (≥3 rooms + connector), one furnished room with staged
damage spanning two damage classes, same rooms captured at all three
input tiers (photo as per-room folders), at least one room captured twice
at the same tier for repeatability.** 6. **Capture route/device matrix:**
satisfy the required approved app route or documented stock-phone
protocol; record device, OS, sensors, settings, coverage, and
limitations. **Weights and large binaries fetched by script or volume.**
Offline ZIP processing alone is not capture-app completion. 7. **Walk-in
test:** cold-start operator workflow from capture through output,
including any required laser reference measurement. **Real properties
contain mirrors, glass, wet-look surfaces and low light; cover these in
submission.** 8. **Consumer app comparison:** run the named app(s) on
required rooms against the same references and protocol; save raw paired
results. 9. **Concealed damage:** every flag has rule fired, evidence,
limitations, and review recommendation. 10. **Calibrated uncertainty:**
every measurement has a calibrated interval or is explicitly marked
uncalibrated/unavailable. A heuristic score is not a calibrated
interval.

If hardware, references, labels, app access, or capture route is
missing, create a blocker and continue with a clearly labeled baseline;
never mark blocked work complete.

## Acceptance gates / scorecard

Create `configs/evaluation/gates.yaml` as the sole threshold source.
Candidate values from the project review, **verify against original
PDF**: - opening localization within 2 cm on at least 85%; - ceiling
height within 1.5 cm per room; - repeat spread below 1 cm; - wall
repeatability ≤1 cm or 0.5% (clarify exact rule); - using supplied poses
as-is = automatic fail; - beat/tie consumer app on ≥70% of dimensions
(review associates this with 10% score; verify); - walk-in test (review
identifies 30% score; verify exact rubric); - **photo-tier whole-property
stitch:** footprint within ±8%, photo wall lengths ±8% with calibrated
intervals, video wall lengths ±3%, correct adjacency/no room overlaps,
single-room-only photo path fails; - **drift accountability ablation:**
stitched footprint with drift correction ON vs OFF must be reported;
"poses used as-is" is automatic fail.

**Fix-loop scoring (PDF Part 4):** full marks = correct root cause,
shipped fix, gate moves fail→pass; majority marks = correct root cause,
shipped fix, meaningful movement short of gate with explanation why it
fell short; marks for post-mortem honesty only = prediction badly wrong;
zero = analysis with no shipped fix; zero = fix with no regenerable
before/after.

Do not guess denominator, rounding, "or" semantics, or tie definitions.
Every run saves `metrics.json`, `per_measurement.csv`,
`gate_results.json`, references/protocol, plots/overlays, hashes, and
per-gate failure reasons. Report each mandatory gate separately;
aggregate score must not hide a failed gate.

## Suggested repository structure

Adapt existing repository rather than restructuring without reason:

``` text
configs/{default.yaml,modalities/,evaluation/gates.yaml}
docs/{requirements/traceability.csv,requirements/assumptions.md,architecture.md,capture_protocol.md,data_dictionary.md,evaluation_protocol.md}
src/property_capture/{cli.py,schemas/,ingestion/,calibration/,modalities/{photo_sfm/,video_slam/,rgbd_lidar/},reconstruction/,registration/,geometry/,rooms/,damage/,uncertainty/,evaluation/,reporting/}
tests/{unit/,integration/,regression/,fixtures/}
data/  # ignored; raw captures never committed
outputs/ # ignored; run-specific
scripts/
```

## Phase 0 --- Requirements and data audit (do first)

### Requirements

Read the original PDF end-to-end. Extract every deliverable, modality,
score item, threshold, constraint, and evaluation protocol into
traceability. Record conflicts and blockers: unavailable device, laser
references, multi-room data, consumer app, damage labels, compute
limits.

### Dataset

For each ZIP: - verify archive integrity; list file paths and sizes; -
inspect image/video dimensions, codec, frame rate, counts, timestamps,
missing/corrupt frames; - inspect depth/confidence dtype, range, invalid
sentinels, units, RGB alignment; - inspect camera intrinsics/extrinsics,
distortion, IMU axes/units, timestamp basis, odometry conventions,
quaternion order, transforms; - quantify synchronization offset;
document assumptions; - generate contact sheets, depth/confidence
previews, and machine-readable manifest. Do not infer conventions from
column names. Save `data_audit/<capture_id>/manifest.json`, data
dictionary, and visual artifacts.

### Baseline

Build the smallest RGB-D end-to-end run, save intermediate artifacts,
and establish baseline metrics before tuning.

## Architecture requirements

### Ingestion and calibration

Use typed structures for frames, timestamps, intrinsics, transforms,
poses, and units. Normalize timestamps to one documented time base.
Validate calibration, transforms, sensor synchronization, and
invalid-depth masks. Record provenance.

### Modality-specific reconstruction

**Photo-only:** feature detection/matching, camera motion, bundle
adjustment, sparse/dense reconstruction as feasible; detect blur, low
texture, insufficient parallax, and scale ambiguity. If metric scale is
unobservable, do not output metric dimensions.

**Video:** visual odometry/SLAM, tracking-loss detection, loop
closure/pose graph when feasible, drift diagnostics. Preserve raw and
corrected trajectories separately.

**RGB-D/LiDAR:** calibrated back-projection, validated camera-to-world
transforms, conservative filtering, confidence/source-frame
preservation, documented voxel/TSDF or equivalent fusion. Compare
against a simple baseline; do not blindly trust odometry.

All paths emit common geometry plus provenance and quality flags.

### Registration and drift

Document coordinate frames and transform direction; test them. Use
robust registration (e.g. RANSAC, initialized ICP) and loop closure/pose
graph where supported. Detect loop inconsistency, drift, duplicate
surfaces, and scale mismatch. Structural priors (vertical walls, floor,
parallelism) must be bounded and explicit, not fabricated evidence.

### Floor plan and rooms

Estimate floor/gravity frame; extract wall planes/segments; consolidate
collinear fragments; reject clutter; generate valid
non-self-intersecting room polygons. Detect openings from evidence and
label inferred vs observed. Estimate ceiling height only with adequate
floor/ceiling evidence. Build room graph and shared-wall consistency.
Render dimensioned SVG/PNG/PDF and machine-readable geometry with units,
scale, labels, uncertainty, and warnings.

### Damage

Separate visible detections from severity. Include evidence locations
and class-specific confidence. Concealed risk flags require explicit,
validated rules and must say "risk indicator---not confirmation." Do not
infer hidden structural defects from absence of visible damage. If
labels are unavailable, report the validation gap.

### Calibrated uncertainty

Every measurement needs method, uncertainty sources, interval, and
calibration status. Calibrate on held-out reference measurements; report
empirical coverage and interval width by type/modality. Use justified
split/conformal or other calibration. Low-quality/OOD inputs should
widen intervals or abstain. If calibration data are insufficient, set
`uncertainty_status: uncalibrated`.

## Evaluation protocol

1.  Freeze laser/reference protocol, endpoints, repeat count, units, and
    observer procedure.
2.  Separate tuning from held-out captures.
3.  Compute absolute/relative error, repeatability, interval
    coverage/width, completeness, and topology errors.
4.  Report per room, wall/opening, modality, and overall; do not average
    away missing outputs.
5.  Compare consumer app using same rooms and references; record
    app/version/device/settings and raw output.
6.  For every fix, save failing gate, evidence, root-cause hypothesis,
    code/config change, rerun command, paired metrics, and gate
    transition.
7.  Include failures and limitations.

## CLI and reproducibility

Provide one documented entry point; adapt names to packaging:

``` bash
python -m property_capture audit --input data/capture.zip --output outputs/audit
python -m property_capture run --input data/capture.zip --tier auto --config configs/default.yaml --output outputs/run_<id>
python -m property_capture evaluate --run outputs/run_<id> --references data/references.csv --gates configs/evaluation/gates.yaml
python -m property_capture report --run outputs/run_<id>
```

Validate input, create fresh output directory, record
command/config/commit/dependencies/seeds/hashes, emit
logs/intermediates/final artifacts, and return nonzero for pipeline
failure or strict gate failure. Measure cold-start runtime on the actual
evaluation machine; do not promise a time without evidence.
**Cached model outputs acceptable when the cache replays deterministically
and the live path also runs, because the walk-in test runs live.** One
documented entry point; adapt names to packaging.

## Testing

-   Unit: projection, transforms, quaternion parsing, depth masks,
    coordinates, polygons, units, interval calibration.
-   Integration: one capture per available tier and common
    schema/artifacts.
-   Regression: all supplied ZIPs; compare metrics within tolerances,
    not brittle pixel equality.
-   Adversarial: missing timestamps, malformed calibration, invalid
    depth, low texture, blur, partial room, tracking loss, loop closure
    failure, duplicate geometry, **mirrors, glass, wet-look surfaces, low light**.
-   Compliance: production path cannot access evaluation-only poses;
    every measurement has uncertainty status; concealed flags have
    rule/evidence; no fabricated zeros.
-   Gate boundary tests and known failing fixtures.

## Roadmap

1.  **Verified baseline:** PDF traceability, assumptions, ZIP audits,
    one RGB-D end-to-end run and baseline metrics.
2.  **Geometry quality:** calibration/synchronization, registration,
    drift, wall/room extraction, repeatability evidence.
3.  **All input tiers:** photo and video paths, scale
    observability/abstention, common schema.
4.  **Multi-room and damage:** adjacency/shared walls; visible damage
    and supported risk rules.
5.  **Compliance benchmark:** capture/device matrix, walk-in protocol,
    consumer comparison, calibrated uncertainty, all gates, reproduction
    bundle, source-required report limit.
6.  **Hardening:** fresh-machine setup, one-command workflow, tests,
    complete traceability and final evidence.

## Required deliverables checklist

-   [ ] Working repository, setup instructions, tests
-   [ ] Traceability matrix, assumptions, blockers
-   [ ] Audit manifests/data dictionary for all supplied captures
-   [ ] Capture/device matrix and protocol, or documented blocker
-   [ ] Common-schema outputs for each supported tier
-   [ ] Dimensioned floor plans and machine-readable geometry
-   [ ] Measurement uncertainty/calibration report
-   [ ] Damage results and concealed-risk rules/evidence
-   [ ] Benchmark and consumer-app comparison
-   [ ] Gate-by-gate evidence
-   [ ] Before/after fix-loop bundle
-   [ ] Reproduction bundle: config, versions, hashes, commands, logs
-   [ ] Walk-in/cold-start evidence
-   [ ] Technical report within source-required page limit

## Final rules

Implement a validated vertical slice first, but preserve the full
requirement map. For each blocked requirement, state missing input,
scoring impact, and next action. Do not replace mandatory work with
"future work" without approval. Optimize for measured accuracy,
repeatability, calibrated uncertainty, coverage, and
reproducibility---not visual plausibility alone. Final status must list
implemented/tested items, gate results, blockers, exact commands, and
artifact paths.
