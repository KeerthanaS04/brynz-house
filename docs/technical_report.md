# Technical report: property capture to dimensioned floor plan

**Status: draft.** The benchmark captures and laser references have not been recorded yet, so every
number below is internal consistency or synthetic ground truth, **never accuracy against a laser**.
Sections marked *Pending benchmark* will be filled from `property_capture evaluate`.
Evidence for each statement: `reports/<topic>/README.md`. Every run regenerates from raw inputs (`scripts/reproduce.py`).

## 1. Architecture

One command per capture: `python -m property_capture run --input <capture>`.
- The tier is detected from the input layout:
  - ZIP or folder with `odometry.csv` → LiDAR
  - a video file → video
  - folder of images → photo, refused (not implemented)
- Every tier ends in the same stages and writes the same output contract: `property.json`, `floorplan.png`, `metrics.json`, `gate_results.json`, and `run_info.json` (config hash, commit, input hashes, timings).

```
LiDAR capture ─ validate inputs ─ pose conventions ─ gravity ─ RGB/depth alignment ─ drift (validated) ─┐
                                                                                                         ├─ fusion ─ floor/ceiling planes
Video file ─ frame selection ─ SfM (COLMAP) ─ depth network ─ per-frame depth scale ─ largest piece ────┘   ─ walls ─ rooms ─ openings
                                                                                                             ─ property.json / plan / gates
```

**Shared stages** (`plan_outputs.py`), which take any frames with depth, poses and an up direction:
- 2 cm voxel fusion
- floor and ceiling plane detection
- tall-cell wall detection with RANSAC lines, refined onto their points
- a wall-snapped outline
- room segmentation at doorways
- shared-wall pairing and alignment
- ceiling height under an evidence rule
- openings from camera rays
- a half-split consistency check

**Design rules applied throughout:**
- **Nothing claimed without evidence.** A quantity whose evidence fails its rule is listed in `unobservable` with the reason. That applies to opening widths without both jambs, ceilings seen in fewer than 8 cells, and video scale.
- **No fabricated values.** Broken inputs stop with the cause (12 cases, §7), and tests check that no measurement is zero or NaN.
- **Every measurement carries `uncertainty_status: uncalibrated`** until references exist.
- **The video tier never reads device data.** A test enforces this, and `video-oracle`, which compares against device poses and LiDAR depth, is evaluation-only.

## 2. Tier design and device matrix

| Tier | Hardware / app | Pipeline | What it delivers today |
|---|---|---|---|
| LiDAR | iPhone 12 Pro or newer with LiDAR; Stray Scanner export (B-20, to verify on a phone) | depth + device poses → shared stages | rooms, walls, openings, ceilings, shared walls, drift ablation |
| Video | any iPhone 15+, Camera app | COLMAP SfM → Depth Anything V2 Metric Indoor Small → depth scale per SfM run → shared stages | plan of the **largest unbroken stretch** of the walk only (7–22% of the LiDAR area) |
| Photo | any iPhone 15+, 2–8 stills per room | not implemented | refused with a message; no photo data supplied |

Capture route: Route 2, a one-page stock protocol (`docs/capture_protocol.md`). The full matrix with
consistency figures is in `docs/device_matrix.md`.

**Why the video tier is partial** (`reports/video_tier`, `reports/video_depth`):
- **SfM tracks well within stretches.** Each stretch is within 0.5–0.8% of distance travelled (median, against device poses).
- **The walk breaks at every turn on the spot.** Rotation at the breaks is 175°/m against 50°/m elsewhere. There is no parallax, so single_room splits into 6–7 pieces and with_ceiling into 17.
- **The depth network gives each frame's shape but not its scale.** After one factor per frame it matches LiDAR to within 5%, but that factor varies 0.74–1.85× from frame to frame.
- **Tied to an SfM run, the scale is sometimes right and sometimes not.** It was within 3–4% on floor_only in two runs, 21% off on with_ceiling, and +3.6% then −17% on single_room. COLMAP's multi-threaded mapping is not deterministic.
- **Joining the pieces failed.** Four tuning rounds of depth-based links joined more frames only at the cost of scale error (14–45%). The tier is therefore restricted to the largest piece and flagged `video_partial_coverage` and `scale_from_depth_network`.
- **The ±3% video wall gate needs a user-measured reference length.**

## 3. Input conventions and calibration analysis

No reference measurements exist (B-21), so calibration means checking every convention and unit against the data itself, and the checks are repeated on every run:

| Item | Check | Result (3 supplied captures) |
|---|---|---|
| Pose convention | 8 hypotheses (quaternion order × direction × camera axes) scored by multi-view depth residual | xyzw, camera-to-world, OpenCV axes; 5.2–6.7 mm residual vs 132–180 mm runner-up (B-11, B-12) |
| Depth unit | scale sweep 0.5–2.0 | 1.0 (mm) best (B-03) |
| Gravity | accelerometer rotated into world, best of 24 device-to-camera rotations | spread 3.3–3.6° vs 6.6–7.8° runner-up; world +y up |
| RGB vs depth | edge alignment at low-motion frames | constant −0.39 to −0.50 depth px offset, corrected; no scale or rotation error; video frame i ↔ depth frame i+1 (B-07) |
| Intrinsics | per-frame from odometry; validated | positive focal, principal point inside image (new check) |
| Depth validity | usable-pixel fraction | 95–98% |
| Floor plane | RANSAC fit | 6.9–8.1 mm RMS, tilt 0.04–0.41° |

**Video tier.** Intrinsics come from COLMAP self-calibration (focal 1320–1344 px at 1600 px, consistent across pieces); scale comes from the depth network, measured against LiDAR by the oracle (§2).

**Calibrated intervals.** Pending benchmark. `property_capture evaluate` scores runs against a reference CSV (`docs/benchmark_protocol.md`: each item measured twice by laser) and reports each gate separately.

Intervals will be derived from those residuals: per quantity, empirical coverage on held-out rooms. Until then, `interval` is `null` and no heuristic is presented as an interval.

## 4. Drift handling

The device poses turned out to be very self-consistent. Frames at least 10 s apart that see the same surfaces agree to **0.95 cm** (floor_only) and **2.6 cm** (with_ceiling) median depth residual (B-23). Six rounds of drift correction were built (`reports/drift`):

1. **Rounds 1–4: keyframe-to-map ICP.** It wandered 3.7° over 37 s. One round took with_ceiling from 2.7 cm to 63 cm residual.
2. **Rounds 5–6: 4-DoF pose graph** with odometry edges and loop edges. A loop edge needs frames at least 10 s apart, at least 30% view overlap, and a forward-backward check within 0.5° / 3 cm. Edges are robustly weighted and inconsistent loops removed.

**Validation decides, not the optimiser.** The corrected trajectory is applied only if it beats the device poses on both the median residual and the share of pixels within 5 cm. This is measured on held-out revisit pairs that never served as loop constraints.

| Capture | Informative pairs | Device poses | Corrected | Decision |
|---|---|---|---|---|
| single_room | 0 of 32 | — | — | rejected: nothing to validate |
| floor_only | 16 of 40 | 0.95 cm, 83% | 3.6 cm, 76% | rejected: worse |
| with_ceiling | 38 of 38 | 2.6 cm, 76% | 6.8 cm, 41% | rejected: worse |

**Ablation (PDF drift gate).** Every run builds the plan with both trajectories. Total area with device / corrected poses: 22.7 / 23.2, 53.7 / 54.2 and 64.0 / 64.2 m².
- Raw and corrected trajectories are saved side by side.
- On these captures the device poses are kept, so `gate_drift_accountability` is honestly reported as `would_fail` ("poses used as-is").
- It is to be re-run on the benchmark multi-room capture, where the gate is defined.

**Tracking jumps.** Any pose step above 10 cm between consecutive frames raises `TRACKING_JUMP`. It fires only on floor_only's 0.314 m relocalization (B-19). On a synthetic capture with an injected 0.3 m jump, the jump is reported, but the walls stay 0.3 m short; the correction does not repair it (strict `xfail` test).

## 5. Error budget (LiDAR tier, per wall length)

Estimated contributions from the measurements above. These are **uncalibrated estimates** to be replaced by benchmark residuals.

| Source | Size | Evidence |
|---|---|---|
| Depth noise after fusion | 5–7 mm per frame pair, averaged over many frames per 2 cm voxel → ≤ 2 mm on a wall line | `reports/baseline` |
| Wall line position (each end) | was up to 20 mm (cell-centre quantization); now ≤ 6 mm on synthetic ground truth | `reports/adversarial` |
| Pose consistency (revisits) | 9.5–26 mm median between visits ≥ 10 s apart; much less within one pass of a wall | `reports/drift` |
| Floor plane / tilt | 7–8 mm RMS, < 0.5° (cos error < 0.004%) | `reports/baseline` |
| Depth scale | sweep best 1.0 on a 5% grid; finer scale error **unknown**, needs a laser | B-03 |
| Outline in unobserved regions | flagged `inferred`; a 1–2 cm wall shift moved one such outline by 1.8 m² | `reports/adversarial` |

**Expected result for observed walls:**
- Length error is about ±1–2 cm plus any depth-scale error × length (1% = 3 cm on a 3 m wall).
- The depth-scale term is the dominant unknown and the first thing the benchmark measures.

**Ceiling height:**
- Precision (standard error) is 0.5–1.1 mm in the 3 measurable rooms of with_ceiling (3.061–3.073 m).
- Bias is unknown. The rooms agree with each other to within 1.2 cm.

**Opening width:**
- Reported only between observed jambs.
- The same doorway measured from its two rooms agreed within 0.7 cm. That was measured before the wall-offset fix.
- 13 of 19 physical openings have a measured width.

## 6. Fix loop

**Benchmark fix loop.** Pending benchmark. The PDF's fix loop targets the worst gate of our own benchmark, which does not exist yet. Before/after tooling is in place (`scripts/reproduce.py`, `scripts/diff_runs.py`).

**Internal fix loop on synthetic ground truth** (`reports/adversarial`):
- **Symptom.** A synthetic 4 × 3 × 2.5 m box, written in the supplied capture format, came out with exact lengths and area, but every wall was **2.0 cm** off its true position.
- **Root cause.** Wall lines were fitted to the centres of 4 cm tall-wall cells, not to the points, which allows up to 2 cm per wall and 4 cm per length. That is as large as the 2 cm opening gate.
- **Fix.** Each line is moved along its normal onto the median of its own points (`walls.refine_offsets`).
- **Before / after.**
  - Synthetic walls: 2.0 cm → ≤ 0.6 cm.
  - Real captures: 130 lines moved by a median 0.6 cm (max 3.4 cm), with no direction change.
  - Real totals: −2.3% to +0.2%, with the largest change in an inferred region.
  - Both runs regenerate (`outputs/repro_lidar` vs `outputs/repro_lidar_walls`).
- **What it cannot show yet.** Whether real walls are now closer to the truth needs laser references.

**Negative results kept on record:**
- **Drift correction** (§4) was validated and rejected.
- **Video joining** (§2): four rounds, each documented with its settings. The looser options remain in the config, switched off.

## 7. Known failure modes

| Failure | Behaviour | Status |
|---|---|---|
| Broken input: NaN or missing timestamps, poses or IMU; zero quaternion; invalid intrinsics; IMU in m/s²; all-zero depth; depth in other units; missing depth frame; featureless video | stops with the cause. Before: 3 ran silently, 5 gave misleading messages, 2 already stopped, 2 untested | fixed, 12 tests |
| Tracking jump | reported (`TRACKING_JUMP`), geometry not repaired | known, strict xfail |
| Partial coverage | observed walls correct, unseen side inferred, area underestimated; room may be labelled "not entered" | known |
| Inferred outlines | sensitive to small changes; flagged `inferred_low_support` | known |
| Short corner jogs (~10 cm) | can pass the 50% observed rule while ~11 cm off | known |
| Doors and windows | detected only where the camera saw through them; closed doors are not detected; phantom rate unknown without labels | known |
| Ceiling | not reported when seen in fewer than 8 cells or implausible (floor_only: furniture tops at 1.64–1.67 m) | by design |
| Room count | unstable between scan halves (half-split check) | known |
| Mirrors, glass, wet-look floors, low light | not tested: synthetic sensor is ideal; real benchmark needed | open |
| Video tier | largest piece only; scale 3–21% off; SfM not deterministic | known |
| Photo tier | not implemented | blocked on data |

**Pending benchmark.** Gates at all tiers, the repeatability table, the head-to-head against a consumer app, and walk-in timing are required (`docs/benchmark_protocol.md`). Current timings on a 4 GB laptop GPU:
- LiDAR tier: 1–8.5 min per capture.
- Video tier: 4–14.5 min per capture.
