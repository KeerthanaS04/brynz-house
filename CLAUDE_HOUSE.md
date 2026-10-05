# Claude Code Engineering Specification

## End-to-End RGB-D Single-Room Reconstruction, Measurement, and Damage Assessment

**Purpose:** Implement a reproducible, end-to-end pipeline for the
supplied three ZIP datasets. This file is the working engineering
contract for Claude Code. Read it before changing code.

**Status:** Architecture and implementation plan based on the supplied
folder inventory. Actual archive contents, official evaluator schema,
coordinate conventions, units, capture-tier obligations, and damage
labels remain to be verified. Never invent these.

------------------------------------------------------------------------

# 1. Mission and success criteria

Build a local, reproducible system that accepts one of the supplied
sensor-recording datasets and produces:

1.  A validated, machine-readable capture manifest.
2.  Synchronized RGB, depth, confidence, calibration, IMU, and odometry
    streams where available.
3.  A registered 3D point cloud in a documented coordinate frame.
4.  Estimated room geometry: floor, walls, ceiling when observable, room
    footprint, and measurable openings when supported.
5.  A 2D dimensioned floor-plan representation and rendered image.
6.  Damage detections and physical extent estimates only if visible
    damage and suitable labels/ground truth exist.
7.  Per-measurement quality/confidence and uncertainty information, with
    assumptions disclosed.
8.  A machine-readable result through a schema adapter. Do not invent
    the evaluator's official schema.
9.  A reproducible evaluation report, including errors, failed cases,
    and a documented improvement iteration.

The system must run from a clean environment using documented commands,
without relying on the developer's private machine, undocumented
services, or hidden files.

## 1.1 Supplied dataset inventory

Three ZIP archives are described by the user:

``` text
single_scan_floor_only/
  confidence/       # image sequence
  depth/            # image sequence
  camera_matrix     # spreadsheet
  imu               # spreadsheet
  odometry          # spreadsheet
  rgb               # video

single_scan_with_ceiling/
  confidence/
  depth/
  camera_matrix
  imu
  odometry
  rgb

single_room/
  confidence/
  depth/
  camera_matrix
  imu
  odometry
  rgb
```

Treat these names and contents as reported inventory, not verified
facts. First inspect archive members, file formats, dimensions, row
counts, timestamps, units, and data semantics.

Do not assume: - the three scans are the same physical room or same
route; - depth is in meters rather than millimeters or another scale; -
confidence values have a particular direction or range; - camera
matrices are intrinsics rather than a sequence of calibration
matrices; - odometry is camera-to-world rather than world-to-camera; -
RGB video and depth frames have matching timestamps or frame rates; -
IMU orientation uses a particular axis convention; - a ceiling is absent
merely because the dataset is named `floor_only`; - any damage labels or
ground truth are included; - a supplied schema is known until located
and read.

## 1.2 Explicit non-goals unless the source brief requires them

-   Building a mobile capture application.
-   Collecting a new large dataset.
-   Building a production cloud service.
-   Claiming generalization from three samples.
-   Inferring concealed damage as fact.
-   Fabricating missing dimensions, labels, metrics, or evaluator
    requirements.

A capture protocol, additional capture, or separate
photo-only/video-only evaluation may still be required if the original
case-study brief explicitly asks for it. Verify before declaring those
requirements out of scope.

------------------------------------------------------------------------

# 2. Engineering principles

1.  **Inspect before implementing.** Inventory all three archives and
    inspect representative samples.
2.  **Preserve raw data.** Never overwrite source files. Store derived
    artifacts separately.
3.  **Make units and frames explicit.** Use typed names such as
    `depth_m`, `translation_m`, `T_world_camera`, and document axis
    handedness.
4.  **Keep observed separate from inferred.** Label measured,
    reconstructed, inferred, and unavailable outputs distinctly.
5.  **Do not silently fabricate.** Missing calibration, labels, ground
    truth, or schema fields must be reported.
6.  **Deterministic pipeline.** Fix random seeds where relevant; record
    software/model versions and configuration.
7.  **Small vertical slices.** First reconstruct one scan, then add
    geometry, evaluation, and additional scans.
8.  **Fail loudly and diagnostically.** Invalid dimensions, timestamps,
    units, transforms, or empty data should produce actionable errors.
9.  **Metrics are computed from actual outputs.** Never insert expected
    or illustrative values as measured results.
10. **Human-review uncertain geometry.** Avoid representing
    low-confidence inferred walls/openings as verified measurements.

------------------------------------------------------------------------

# 3. Revised module plan

The earlier 12-module plan is consolidated into eight implementation
modules. Capture-app development and new data collection are removed
from the initial build because sensor recordings are supplied. Ingestion
remains essential. Multi-room stitching is deferred until the data and
source requirements demonstrate that multiple rooms/routes must be
joined.

## Module 1 --- Archive inspection, data contract, and validation

### Objective

Convert each ZIP into a validated capture package with a manifest and
clear data semantics.

### Inputs

The three ZIP archives and any accompanying documentation.

### Work

-   List archive members, file sizes, extensions, and directory
    structure.
-   Detect archive corruption, duplicate names, path traversal hazards,
    and unsupported formats.
-   Inspect RGB video metadata: codec, duration, frame rate, frame
    count, width, height.
-   Inspect depth and confidence files: count, dimensions, dtype,
    encoding, min/max, invalid values.
-   Read spreadsheet headers, row counts, timestamp columns, missing
    values, units, and sample rows.
-   Determine whether camera calibration is static or time-varying.
-   Determine whether odometry contains translation, quaternion,
    rotation matrix, covariance, or status flags.
-   Determine IMU fields and sampling frequency.
-   Match streams by timestamps or documented frame indices; do not
    align by array index unless verified.
-   Write a manifest with observed facts and unresolved questions.

### Outputs

-   `data/manifest/<capture_id>.json`
-   `reports/data_audit/<capture_id>.md`
-   validation errors/warnings
-   a data dictionary

### Suggested functions

-   `inspect_archive(path) -> ArchiveInventory`
-   `inspect_video(path) -> VideoMetadata`
-   `inspect_image_sequence(path) -> ImageSequenceMetadata`
-   `inspect_table(path) -> TableMetadata`
-   `build_capture_manifest(...) -> CaptureManifest`
-   `validate_capture(manifest) -> ValidationReport`

### Acceptance

All three archives are inventoried. Every stream has documented count,
shape, dtype/format, timestamp source, and units if known. Unknown units
are explicitly `unknown`, never guessed. A broken or incomplete capture
is reported without crashing the entire audit.

------------------------------------------------------------------------

## Module 2 --- Ingestion, synchronization, calibration, and preprocessing

### Objective

Create consistent typed sensor records suitable for reconstruction.

### Inputs

Validated archive data and manifest.

### Work

-   Extract archives safely into a configurable raw-data directory.
-   Decode RGB video to frames while preserving timestamps and source
    frame indices.
-   Load depth/confidence images without lossy conversion.
-   Load camera matrix, IMU, and odometry tables.
-   Normalize timestamps to one internal time unit while preserving
    original values.
-   Estimate or verify stream offsets using documented metadata or
    measurable synchronization evidence.
-   Validate intrinsic matrix shape and positive focal lengths.
-   Validate pose representation and rotation orthonormality.
-   Convert depth to meters only after identifying its encoding/scale.
-   Apply invalid-depth masks and confidence filtering using
    configurable, documented policy.
-   Preserve unfiltered data for audit.
-   Store derived records in a documented intermediate format (e.g.,
    NumPy arrays plus JSON metadata, or parquet where suitable).

### Coordinate conventions

Adopt a single internal convention only after inspecting source
conventions. Document: - camera axes; - world axes; - handedness; -
units; - transform direction; - quaternion order; - timestamp epoch and
units.

Use explicit transform names, e.g. `T_world_camera`. Provide conversion
functions and tests rather than scattering axis flips through code.

### Outputs

-   synchronized frame index/table;
-   validated calibration object;
-   normalized pose stream;
-   preprocessed depth/confidence;
-   synchronization diagnostics.

### Suggested functions

-   `load_capture(capture_id) -> Capture`
-   `decode_rgb_video(video_path) -> FrameStream`
-   `load_depth_sequence(path) -> DepthStream`
-   `load_sensor_tables(...) -> SensorStreams`
-   `synchronize_streams(streams, config) -> SynchronizedCapture`
-   `validate_intrinsics(K, image_shape)`
-   `normalize_depth(raw_depth, encoding_config)`
-   `filter_depth(depth_m, confidence, config)`

### Acceptance

Frame matching is reproducible and logged. A test verifies known
pixel/depth/intrinsics produce the expected camera-frame point. Invalid
calibration or ambiguous units block metric reconstruction with a clear
error.

------------------------------------------------------------------------

## Module 3 --- RGB-D 3D reconstruction and point-cloud fusion

### Objective

Reconstruct a registered 3D representation from depth images and camera
poses.

### Core back-projection

For pixel `(u,v)`, depth `Z`, and intrinsic matrix:

``` text
K = [[fx, 0, cx],
     [0, fy, cy],
     [0,  0,  1]]
```

``` text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
P_camera = [X, Y, Z, 1]^T
P_world = T_world_camera @ P_camera
```

This formula assumes the depth is camera-axis Z depth and the chosen
coordinate convention. If depth is radial range or uses another
convention, implement the correct conversion and document it.

### Work

-   Back-project valid depth pixels.
-   Attach RGB colors where frame alignment is valid.
-   Transform each frame into the common world frame using verified
    odometry.
-   Filter invalid, low-confidence, and physically implausible points
    using configurable thresholds.
-   Downsample using voxel filtering.
-   Remove outliers conservatively.
-   Merge frames into a global cloud.
-   Optionally refine registration using ICP only when initial alignment
    and geometry support it; preserve odometry-only baseline.
-   Save intermediate per-frame clouds and final cloud.
-   Produce top, side, and perspective diagnostic views.

### Candidate libraries

-   NumPy, OpenCV, Open3D, pandas.
-   Use PyTorch only if a learned model is justified.
-   Record versions and licenses.

### Outputs

-   `pointcloud_raw.ply`
-   `pointcloud_filtered.ply`
-   optional `pointcloud_rgb.ply`
-   registration diagnostics and trajectory visualization.

### Suggested functions

-   `depth_to_points(depth_m, K, mask=None)`
-   `transform_points(points, T_world_camera)`
-   `build_frame_cloud(frame_record, config)`
-   `fuse_frame_clouds(frame_clouds, poses, config)`
-   `refine_registration(source, target, config)`
-   `export_point_cloud(cloud, path)`

### Acceptance

Synthetic unit tests validate back-projection and transforms.
Point-cloud bounds are plausible and expressed in documented units.
Registration drift/overlap diagnostics are reported. If pose data is
missing or invalid, the pipeline does not pretend that unregistered
frames form a metric global scan.

------------------------------------------------------------------------

## Module 4 --- Room geometry, planes, dimensions, and floor plan

### Objective

Derive a structured room model and a 2D plan from the reconstructed
cloud.

### Work

1.  Estimate dominant horizontal planes (floor and ceiling where
    observed).
2.  Estimate vertical planes for walls.
3.  Use robust fitting (e.g., RANSAC) and normal/orientation
    constraints.
4.  Cluster planes and reject small/noisy components.
5.  Estimate wall boundaries and intersections.
6.  Derive room footprint by projecting suitable geometry onto a
    horizontal plane.
7.  Estimate dimensions from fitted geometry, not arbitrary image pixel
    ratios.
8.  Detect openings only where data supports a geometric gap or explicit
    annotation.
9.  Distinguish observed ceiling from inferred ceiling.
10. Preserve confidence, support count, residual, and source evidence
    for each plane/measurement.
11. Generate a 2D vector representation and render SVG/PNG.

### Candidate algorithms

-   Open3D plane segmentation/RANSAC.
-   PCA for local normals and principal directions.
-   Polygon boundary extraction and line intersection.
-   Optional Manhattan-world regularization only as a configurable
    assumption, never silently applied.

### Outputs

-   room model JSON;
-   floor/wall/ceiling plane records;
-   room footprint polygon;
-   wall segments and dimensions;
-   plan SVG/PNG;
-   geometry diagnostics.

### Suggested functions

-   `estimate_normals(cloud)`
-   `fit_dominant_planes(cloud, config)`
-   `classify_planes(planes, gravity_axis)`
-   `extract_room_footprint(planes, config)`
-   `estimate_wall_dimensions(room_model)`
-   `detect_openings(room_model, cloud, rgb=None)`
-   `render_floor_plan(room_model, output_path)`

### Acceptance

All dimensions carry units and evidence status. Floor-only scans must
not receive fabricated ceiling measurements. Failed or ambiguous plane
extraction is surfaced. Geometry can be visualized and inspected.

------------------------------------------------------------------------

## Module 5 --- Damage detection and metric extent (conditional)

### Objective

Detect visible damage and estimate physical extent only when supported
by available data.

### Prerequisite gate

First inspect whether the supplied RGB data contains staged damage,
annotations, masks, or damage ground truth. If absent, implement the
interface and report `not_evaluated` rather than inventing detections or
training labels.

### Work when data supports it

-   Sample representative RGB frames.
-   Define damage taxonomy from the case brief or dataset labels; do not
    invent required classes.
-   Establish a baseline detector/segmenter and record model provenance,
    weights, license, and runtime requirements.
-   Segment damage regions.
-   Associate detections with camera frame and room surface.
-   Project mask pixels through aligned depth and camera pose into 3D.
-   Estimate area/length only where depth coverage and scale permit.
-   Flag occlusion, glare, blur, low resolution, and out-of-view
    surfaces.
-   Keep confidence and evidence links.
-   Do not infer concealed damage as a confirmed fact.

### Outputs

-   damage detections/masks;
-   surface association;
-   metric extent with uncertainty or `unavailable`;
-   qualitative limitations report.

### Suggested functions

-   `detect_damage(frame, model, config)`
-   `project_damage_mask(mask, depth, K, T_world_camera)`
-   `estimate_damage_extent(points, surface_model)`
-   `validate_damage_outputs(...)`

### Acceptance

No damage score is reported without labels/ground truth. Physical area
is not calculated from RGB pixel area alone. Every output links to
source frames and indicates uncertainty.

------------------------------------------------------------------------

## Module 6 --- Measurement uncertainty and evaluation

### Objective

Quantify reliability and compare predictions to available ground truth.

### Work

-   Identify ground-truth dimensions and how they were measured.
-   Define error metrics before looking at results.
-   Compute absolute error, relative error, bias, and repeatability
    where repeated captures truly exist.
-   Compare room dimensions and openings where reference values exist.
-   Track point-to-plane residuals and plane support.
-   Estimate uncertainty from sensor noise, confidence, calibration,
    pose uncertainty, and geometric fit residuals where data permits.
-   If a statistically calibrated interval cannot be supported, report a
    quality score or heuristic uncertainty clearly labeled as
    uncalibrated---not a calibrated confidence interval.
-   Report missing/undetected features as misses, not silently exclude
    them.
-   Include per-scan and aggregate results, with sample counts.

### Metrics

For reference `g` and estimate `p`: - Absolute error: `|p - g|` -
Relative error: `|p - g| / max(|g|, epsilon)` - Mean absolute error
across evaluated measurements. - Bias: mean signed error. -
Repeatability: spread across genuinely repeated scans of the same
physical scene and modality. - Opening precision/recall only if labeled
openings exist. - Damage IoU/precision/recall only if masks/labels
exist.

Do not claim target thresholds from the original brief are met until
measured on the relevant benchmark.

### Outputs

-   machine-readable metrics JSON/CSV;
-   plots;
-   evaluation report with failures and caveats.

### Suggested functions

-   `evaluate_dimensions(predictions, ground_truth)`
-   `evaluate_openings(predictions, labels)`
-   `evaluate_damage(predictions, labels)`
-   `estimate_measurement_quality(measurement, evidence)`
-   `generate_evaluation_report(results)`

### Acceptance

Metrics are reproducible from saved predictions and ground truth. Every
metric states its population, units, count, and missing-data policy.

------------------------------------------------------------------------

## Module 7 --- Output schema, rendering, and CLI integration

### Objective

Package all module outputs into one end-to-end command and
evaluator-compatible artifacts.

### Schema rule

The original case-study brief may refer to an official JSON schema.
Locate it and implement a dedicated adapter. Until provided, use a
clearly labeled provisional internal schema and do not claim evaluator
compatibility.

### Internal result should include

-   capture ID and dataset provenance;
-   processing config and software versions;
-   coordinate frame and units;
-   room geometry;
-   dimensions with value, unit, evidence status, uncertainty/quality,
    and source;
-   openings with observed/inferred status;
-   damage records or explicit `not_evaluated`;
-   artifact paths;
-   warnings and failures.

### CLI

Suggested interface:

``` bash
python -m src.cli inspect --input data/raw/single_room.zip
python -m src.cli process --capture single_scan_with_ceiling --config configs/default.yaml
python -m src.cli evaluate --predictions outputs/... --ground-truth data/ground_truth/...
```

A single end-to-end command should run audit/validation, preprocessing,
reconstruction, geometry, rendering, and result export. Support a
`--skip-damage` or equivalent if damage prerequisites are absent.

### Outputs

``` text
outputs/<capture_id>/<run_id>/
  manifest.json
  synchronized_frames.csv
  trajectory.csv
  pointcloud_filtered.ply
  room_model.json
  floor_plan.svg
  floor_plan.png
  result.json
  metrics.json              # when ground truth exists
  report.md
  logs/
```

### Acceptance

A clean environment can run the documented command. Errors are
actionable. Outputs are versioned by run/config and never overwrite raw
inputs. Official schema conformance is tested once the schema is
available.

------------------------------------------------------------------------

## Module 8 --- Benchmark, comparison, and evidence-backed fix loop

### Objective

Demonstrate measured performance and improve the largest failure.

### Work

-   Run the same pipeline/config on all three supplied datasets.
-   Compare floor-only and ceiling-visible cases without assuming they
    are same-scene repeats.
-   Inspect reconstruction, floor plan, and measurement differences.
-   Compare against ground truth where available.
-   Identify the most consequential observed failure.
-   State a root-cause hypothesis supported by logs/visual evidence.
-   Implement one fix.
-   Re-run identical evaluation conditions.
-   Produce before/after metrics and artifact diff.
-   Record regressions and unresolved limitations.
-   If the source brief requires consumer-app comparison or
    photo-only/video-only tiers, add a separate benchmark workstream
    rather than pretending these RGB-D recordings satisfy it.

### Outputs

-   benchmark matrix;
-   per-scan result bundle;
-   before/after fix report;
-   reproducibility instructions;
-   known limitations.

### Acceptance

At least one complete evidence-backed improvement loop is delivered. No
improvement claim without a reproducible before/after run.

------------------------------------------------------------------------

# 4. System architecture

``` text
                         ┌───────────────────────────┐
                         │ Three source ZIP archives │
                         └─────────────┬─────────────┘
                                       │
                                       v
                         ┌───────────────────────────┐
                         │ Archive audit + manifest  │
                         │ file/schema/time checks   │
                         └─────────────┬─────────────┘
                                       │
                                       v
                         ┌───────────────────────────┐
                         │ Sensor ingestion          │
                         │ RGB / depth / confidence  │
                         │ intrinsics / IMU / odom   │
                         └─────────────┬─────────────┘
                                       │
                                       v
                         ┌───────────────────────────┐
                         │ Synchronization +         │
                         │ calibration validation   │
                         └─────────────┬─────────────┘
                                       │
                         ┌─────────────┴──────────────┐
                         v                            v
              ┌────────────────────┐       ┌─────────────────────┐
              │ RGB-D backproject  │       │ Pose/trajectory     │
              │ depth -> 3D points│       │ normalization       │
              └──────────┬─────────┘       └──────────┬──────────┘
                         └─────────────┬──────────────┘
                                       v
                         ┌───────────────────────────┐
                         │ Transform + point-cloud   │
                         │ fusion / registration     │
                         └─────────────┬─────────────┘
                                       v
                         ┌───────────────────────────┐
                         │ Geometry extraction       │
                         │ floor / walls / ceiling   │
                         │ footprint / openings     │
                         └─────────────┬─────────────┘
                                       │
                     ┌─────────────────┴─────────────────┐
                     v                                   v
          ┌──────────────────────┐            ┌─────────────────────┐
          │ 2D floor plan        │            │ RGB damage branch   │
          │ dimensions + render  │            │ optional/conditional│
          └──────────┬───────────┘            └──────────┬──────────┘
                     └─────────────────┬─────────────────┘
                                       v
                         ┌───────────────────────────┐
                         │ Uncertainty + evaluation  │
                         │ GT comparison / diagnostics│
                         └─────────────┬─────────────┘
                                       v
                         ┌───────────────────────────┐
                         │ Schema adapter + CLI      │
                         │ JSON / SVG / PNG / report │
                         └───────────────────────────┘
```

## 4.1 Logical software layers

-   **Adapters:** ZIP, video, image sequence, spreadsheet readers.
-   **Domain models:** Capture, FrameRecord, Calibration, Pose,
    PointCloud, Plane, RoomModel, Measurement, DamageRecord.
-   **Core algorithms:** synchronization, back-projection, transforms,
    filtering, registration, plane fitting, footprint extraction.
-   **Evaluation:** ground-truth loaders, metrics, plots, failure
    analysis.
-   **Application:** CLI orchestration, configuration, logging, output
    packaging.
-   **Presentation:** floor-plan renderer and reports.

Keep algorithm code independent of CLI and file paths. Use dependency
injection/configuration for paths and thresholds.

------------------------------------------------------------------------

# 5. Suggested repository structure

``` text
project/
├── CLAUDE.md
├── README.md
├── pyproject.toml
├── configs/
│   ├── default.yaml
│   ├── floor_only.yaml
│   ├── with_ceiling.yaml
│   └── single_room.yaml
├── data/
│   ├── raw/                 # ignored by git unless explicitly permitted
│   ├── manifests/
│   ├── ground_truth/
│   └── README.md
├── src/
│   ├── __init__.py
│   ├── cli.py
│   ├── config.py
│   ├── domain/
│   │   ├── models.py
│   │   └── coordinate_frames.py
│   ├── data/
│   │   ├── archive.py
│   │   ├── video.py
│   │   ├── image_sequence.py
│   │   ├── tables.py
│   │   ├── manifest.py
│   │   └── validation.py
│   ├── preprocessing/
│   │   ├── synchronization.py
│   │   ├── calibration.py
│   │   └── depth.py
│   ├── reconstruction/
│   │   ├── backprojection.py
│   │   ├── transforms.py
│   │   ├── pointcloud.py
│   │   └── registration.py
│   ├── geometry/
│   │   ├── planes.py
│   │   ├── room_model.py
│   │   ├── footprint.py
│   │   ├── openings.py
│   │   └── dimensions.py
│   ├── damage/
│   │   ├── interface.py
│   │   ├── detection.py
│   │   └── projection.py
│   ├── evaluation/
│   │   ├── ground_truth.py
│   │   ├── metrics.py
│   │   ├── uncertainty.py
│   │   └── report.py
│   ├── output/
│   │   ├── internal_schema.py
│   │   ├── official_schema_adapter.py
│   │   └── render.py
│   └── pipeline/
│       └── runner.py
├── tests/
│   ├── unit/
│   ├── integration/
│   └── fixtures/
├── scripts/
│   ├── audit_datasets.py
│   └── verify_environment.py
├── outputs/                 # generated, ignored by git
└── reports/
```

Do not create empty modules merely to match this tree; implement
incrementally.

------------------------------------------------------------------------

# 6. Data contracts

Use dataclasses or Pydantic models with explicit types. Exact field
definitions may be adapted after inspection.

## CaptureManifest

-   `capture_id`
-   `source_archive`
-   `streams`: paths, counts, formats, dimensions, timestamp source
-   `camera_intrinsics`: values, resolution, source, status
-   `depth_encoding`: unit/scale, invalid sentinel, verified flag
-   `pose_convention`: frame names, transform direction, axis convention
-   `imu_metadata`
-   `warnings`
-   `unresolved_assumptions`

## FrameRecord

-   `frame_id`
-   `rgb_timestamp`
-   `depth_timestamp`
-   `confidence_timestamp`
-   source frame indices
-   file references
-   synchronization delta
-   validity status

## Measurement

-   `name`
-   `value`
-   `unit`
-   `method`
-   `evidence_status`: observed / reconstructed / inferred / unavailable
-   `quality`: numeric only if defined
-   `uncertainty`: numeric interval only if defensible; otherwise null
-   `source_artifacts`
-   `warnings`

## RoomModel

-   coordinate frame and units
-   floor plane
-   ceiling plane or unavailable status
-   wall segments
-   footprint polygon
-   openings
-   dimensions
-   quality and warnings

## PipelineResult

-   capture and run identifiers
-   config hash/version
-   software versions
-   artifact references
-   room model
-   damage state (`evaluated`, `not_evaluated`, `failed`)
-   metrics when ground truth exists
-   warnings/errors

------------------------------------------------------------------------

# 7. Implementation sequence and integration gates

## Phase 0 --- Source requirements and data audit

1.  Read the original case-study PDF and extract exact required outputs,
    thresholds, required modalities, and official schema references.
2.  Locate the three ZIP files and audit them.
3.  Inspect sample rows and representative images/frames.
4.  Produce a data dictionary and unresolved-question list.
5.  Do not proceed to metric reconstruction until depth units and
    transform conventions are understood.

**Gate:** reproducible audit for all three archives.

## Phase 1 --- Single-frame geometry

1.  Load one RGB frame, depth image, confidence image, and camera
    matrix.
2.  Validate pixel dimensions and calibration.
3.  Implement back-projection.
4.  Unit-test against hand-computed synthetic examples.
5.  Visualize the frame point cloud.

**Gate:** expected geometry from synthetic input and plausible
real-frame cloud.

## Phase 2 --- Multi-frame registration

1.  Parse odometry and determine transform convention.
2.  Validate timestamps and trajectory continuity.
3.  Transform frame clouds into the common frame.
4.  Fuse a short sequence, then the full scan.
5.  Compare odometry-only and optional ICP-refined outputs.

**Gate:** trajectory and cloud visualizations plus registration
diagnostics.

## Phase 3 --- Geometry and floor plan

1.  Estimate gravity direction using valid IMU/pose evidence or document
    fallback.
2.  Fit floor and vertical planes.
3.  Estimate footprint and wall dimensions.
4.  Handle ceiling-present and ceiling-not-observed cases separately.
5.  Render and inspect plan.

**Gate:** room model and dimensions with source evidence and errors
where ground truth exists.

## Phase 4 --- Quality, uncertainty, and evaluation

1.  Establish ground-truth availability.
2.  Implement metrics with explicit missing-data policy.
3.  Add quality/uncertainty representation.
4.  Run all three datasets.
5.  Save plots and failure examples.

**Gate:** reproducible benchmark report.

## Phase 5 --- Damage branch

Only proceed if the sample data or brief provides damage
examples/labels, or if an explicitly approved external dataset/model is
used. Record provenance and limitations.

**Gate:** detections linked to source frames; metric extent only when
valid depth and surface mapping exist.

## Phase 6 --- Output and packaging

1.  Implement provisional internal JSON.
2.  Add official schema adapter only after obtaining schema.
3.  Implement CLI and clean setup.
4.  Run from a fresh environment following README.
5.  Package outputs, config, versions, and report.

**Gate:** one-command end-to-end run with no hidden manual step.

## Phase 7 --- Evidence-backed fix

1.  Select one measured failure.
2.  Record baseline artifacts and metric.
3.  Implement targeted change.
4.  Re-run same config/data.
5.  Produce before/after comparison and note regressions.

**Gate:** reproducible improvement bundle, even if the change does not
improve every metric.

------------------------------------------------------------------------

# 8. Testing strategy

## Unit tests

-   Intrinsic matrix validation.
-   Depth-unit conversion.
-   Pixel-to-3D back-projection.
-   Rigid transform composition/inversion.
-   Quaternion parsing and rotation orthogonality.
-   Timestamp matching and tolerance behavior.
-   Confidence mask policy.
-   Plane fitting on synthetic planes.
-   Dimension and unit conversions.
-   JSON serialization and schema validation.

## Integration tests

-   One sample frame through back-projection.
-   Short sequence through pose transform and fusion.
-   One capture through room-model extraction and floor-plan render.
-   Full CLI run on a small fixture.
-   Invalid/missing stream behavior.
-   Repeat run determinism within documented tolerance.

## Failure cases to test

-   Missing depth frame.
-   RGB/depth resolution mismatch.
-   Unknown depth scale.
-   Non-monotonic or duplicate timestamps.
-   Empty confidence image.
-   NaN/zero/invalid depth.
-   Missing odometry pose.
-   Singular/invalid intrinsics.
-   Pose discontinuity.
-   Insufficient wall coverage.
-   Floor not visible.
-   Ceiling not visible.
-   Open doors/windows or reflective surfaces causing depth holes.
-   Video decode failure.
-   Ground truth missing.
-   Damage labels missing.
-   Output schema unavailable.

------------------------------------------------------------------------

# 9. Metrics and reporting

Report only metrics actually computed.

For each capture: - input frame counts and valid synchronized pair
counts; - valid-depth fraction and confidence distribution; - trajectory
length and pose availability; - point count before/after filtering; -
plane inlier counts and fit residuals; - room dimension estimates and
reference errors where references exist; - floor-plan visual artifact; -
warnings and unsupported outputs; - processing time and peak memory if
measured.

For aggregate metrics, state: - number of captures and measurements; -
unit; - mean/median and spread; - whether captures are independent,
repeated, or different scenes; - treatment of missing predictions and
failures.

Do not claim statistical calibration from heuristic confidence. Do not
treat the three named datasets as a statistically representative
benchmark.

------------------------------------------------------------------------

# 10. Candidate technology stack

Start with a minimal stack: - Python 3.10 or 3.11 (choose one after
dependency compatibility check). - NumPy, pandas, OpenCV, Open3D. -
SciPy for numerical geometry where useful. - Pydantic for typed
configuration and result models. - PyYAML for configuration. - pytest
for tests. - matplotlib for diagnostics. - Pillow/imageio or OpenCV for
image/video I/O, choosing one consistent approach. - A CLI library such
as Typer or argparse.

Avoid adding heavyweight learned models until the classical RGB-D
baseline is measured. If using pretrained models, document model
name/version, license, download source, checksum, hardware requirements,
and whether inference needs network access.

------------------------------------------------------------------------

# 11. Configuration

Keep thresholds and paths outside algorithm code. Example provisional
configuration (values are placeholders to be tuned, not validated
defaults):

``` yaml
capture_id: single_scan_with_ceiling
paths:
  raw_root: data/raw
  output_root: outputs
synchronization:
  max_delta_ms: null   # determine from timestamps and frame rates
depth:
  unit: unknown        # must be resolved from source
  min_m: null
  max_m: null
  confidence_policy: unresolved
reconstruction:
  voxel_size_m: null   # tune from scene scale and desired detail
  registration: odometry_baseline
geometry:
  plane_distance_threshold_m: null
  min_plane_support: null
damage:
  enabled: false       # enable only after confirming data/labels/model
evaluation:
  ground_truth_path: null
```

Claude must not silently replace nulls with arbitrary values. Propose
values with rationale, make them configurable, and report that they are
provisional until validated.

------------------------------------------------------------------------

# 12. Definition of Done

The project is complete only when:

-   [ ] Original case-study requirements and official schema have been
    checked.
-   [ ] All three ZIPs have audit manifests.
-   [ ] Data units, synchronization, and coordinate conventions are
    documented or explicitly unresolved.
-   [ ] One RGB-D frame produces a tested 3D point cloud.
-   [ ] A sequence produces a registered cloud with diagnostics.
-   [ ] Room geometry and a 2D floor plan are generated where coverage
    supports them.
-   [ ] Ceiling absence/visibility is represented honestly.
-   [ ] Damage is evaluated only if data supports it; otherwise status
    is explicit.
-   [ ] Measurements include units, evidence status, and defensible
    quality/uncertainty.
-   [ ] Ground-truth metrics are reproducible where references exist.
-   [ ] All three datasets are run through the pipeline, with failures
    preserved.
-   [ ] A before/after fix loop is documented.
-   [ ] CLI, README, environment setup, and output structure work from a
    clean setup.
-   [ ] No fabricated metrics, labels, dimensions, schema fields, or
    success claims.

------------------------------------------------------------------------

# 13. Instructions to Claude Code

You are the implementation engineer. Follow these rules:

1.  Read this file and the original case-study PDF before coding.
2.  Begin with a repository and data inventory. Do not immediately
    generate the entire application.
3.  Ask for or locate the actual ZIPs, schema, and ground-truth files if
    they are not present in the workspace. Do not assume they are
    available.
4.  First deliver a concise audit: files found, metadata discovered,
    unresolved assumptions, and a proposed Phase 0 plan.
5.  Implement one vertical slice at a time, with tests and runnable
    commands.
6.  Keep raw data immutable; never commit large/private data unless
    explicitly requested.
7.  Never guess depth units, pose direction, camera axes, or timestamp
    alignment.
8.  Keep observed, reconstructed, inferred, and unavailable fields
    distinct.
9.  Do not invent results. Run the code and report actual commands,
    outputs, and failures.
10. Before adding a model or dependency, explain why the baseline needs
    it and disclose size, license, download, and runtime implications.
11. Preserve failed examples and logs. Do not hide failures by filtering
    them out of metrics.
12. Do not claim the official output schema is satisfied until it has
    been obtained and validated.
13. At the end of each phase, report: files changed, design decisions,
    tests run and results, artifacts generated, known limitations, and
    next gate.
14. Keep the README and this specification synchronized with material
    changes.
15. Prefer a simple, inspectable classical RGB-D baseline before adding
    learned reconstruction or damage models.

## First task for Claude Code

Do not start with model training or a large implementation. Perform
Phase 0: - inspect the repository and available files; - locate and
inventory the three ZIPs; - inspect video, image, and spreadsheet
metadata; - produce `reports/data_audit/` reports and a data
dictionary; - identify missing official schema/ground truth and
unresolved conventions; - propose the first tested single-frame RGB-D
reconstruction slice.

Then stop and summarize findings before implementing the next phase.
