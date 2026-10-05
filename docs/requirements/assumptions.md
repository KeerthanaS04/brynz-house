# Assumptions, interpretations and open questions

Tracks every interpretation the project depends on that the case-study PDF
(`Applied_AI_Case_Study.pdf`, Aug 2026) or the supplied data does not settle.
Labels follow `CLAUDE_updated.md`: **SOURCE** (stated in the PDF), **OBSERVED**
(measured, with evidence path), **ASSUMPTION** (adopted without proof),
**PROPOSED** (design choice we intend to make), **OPEN** (needs an answer
before the dependent work can be called complete).

Data evidence comes from `scripts/audit_datasets.py`; per-capture details are in
`reports/data_audit/<capture_id>/manifest.json` and `docs/data_dictionary.md`.

Status values: `open`, `adopted` (we proceed on it, revisit if contradicted),
`resolved` (closed with evidence).

## A. Requirement interpretation

| ID | Label | Statement | Evidence / reasoning | Impact if wrong | Status / next action |
|---|---|---|---|---|---|
| A-01 | SOURCE + ASSUMPTION | The three supplied ZIPs are development data only, not benchmark captures. | PDF Part 1: "We provide no captures"; the benchmark set must be captured by us (PDF p.2). | Results on these ZIPs cannot be reported as benchmark accuracy. | adopted. Benchmark captures remain BLOCKED (REQ-11, DEL-08). |
| A-02 | OPEN | The "Round 1 gates" are not defined in this PDF. It says "Round 1 gates apply, with five additions"; the five rows in the table (opening widths, ceiling height, repeatability, drift accountability, photo-tier stitch) are the additions. | PDF p.2. `configs/evaluation/gates.yaml` labels the five rows as "Round 1 gates", which conflicts with the PDF wording. | Unknown gates could be scored at the walk-in and benchmark. | open. Request the Round 1 gate list from the evaluator; correct the gates.yaml comment. |
| A-03 | SOURCE | The opening gate is on opening **widths** (≤ 2 cm on ≥ 85% of openings; missed and phantom openings each count as a miss). | PDF p.2 table. `gates.yaml` and `CLAUDE_updated.md` describe it as "opening localization". | Evaluating position instead of width would score the wrong quantity. | open. Correct the gates.yaml description. Denominator (ground-truth openings + phantoms?) still needs confirmation. |
| A-04 | OPEN | Repeatability "within 1 cm or 0.5% per wall": we read "or" as pass if either tolerance holds, per wall, with all walls required to pass. | PDF p.2. `gates.yaml` already sets `or_condition: true`, but CLAUDE_updated.md says not to guess "or" semantics. | Gate pass/fail could flip for long walls (0.5% of 4 m = 2 cm). | open. Confirm with evaluator; until then report both tolerances separately per wall. |
| A-05 | OPEN | Footprint ±8% (photo stitch): definition of footprint (sum of room floor areas vs outer envelope area) and reference (laser-derived plan). | PDF p.2 does not define it. | Different definitions give different errors. | open. |
| A-06 | OPEN | Head-to-head "beat or tie": tie definition (equal absolute error? within a tolerance?) and the set of "shared dimensions". | PDF Part 3. | Gate outcome depends on the tie rule. | open. Save raw paired errors so any rule can be applied later. |
| A-07 | SOURCE | Output contract includes **scope line items keyed to surfaces**. | PDF Part 2: "scope line items keyed to surfaces". Not in `docs/requirements/traceability.csv`. | Missing contract item lowers compliance-matrix score (10%). | open. Add a traceability row. |
| A-08 | OPEN | "JSON to the published schema": the schema has not been supplied. | PDF Part 2. | Cannot claim schema compliance. | open (already REQ-09). Use our own `property.json` until the schema is obtained. |
| A-09 | ASSUMPTION | Device poses supplied in a LiDAR capture are legitimate pipeline **inputs** (PDF tier 3: "Depth, poses and intrinsics"), but using them **as-is** without drift handling fails the drift-accountability gate. CLAUDE_updated.md rule 6 about "ground-truth poses" applies only to poses supplied as ground truth for evaluation. | PDF Part 1 tier 3 and drift row on p.2. The supplied `odometry.csv` is device odometry, not labelled ground truth. | If the evaluator considers odometry "ground truth", production use would be an automatic fail. | adopted. Keep raw and corrected trajectories separate; run the drift on/off ablation. Confirm with evaluator. |
| A-10 | SOURCE | Walk-in capture device is the evaluator's iPhone 15 or newer, tier chosen on the day; Pro-class only for LiDAR. | PDF p.4 and Part 1. | — | Device matrix must cover non-Pro iPhones for photo/video tiers. |

## B. Supplied data (all three captures)

Captures: `c00a170fe1` (single_room.zip), `1a8384c3f6` (single_scan_floor_only.zip),
`c7d28f72c6` (single_scan_with_ceiling.zip).

| ID | Label | Statement | Evidence | Impact if wrong | Status / next action |
|---|---|---|---|---|---|
| B-01 | OBSERVED | Archives intact: CRC pass on every member; no missing, duplicate or corrupt depth/confidence frames. | manifests, `archive` and `depth`/`confidence` sections. | — | resolved. |
| B-02 | OBSERVED | Depth is 256x192 uint16 PNG (PIL mode I;16); raw range 55..5852, 74..11406, 48..12125 for the three captures; **no zero pixels and no 65535 pixels**. | manifests, `depth.stats`. | — | resolved (format). |
| B-03 | ASSUMPTION | Depth units are millimetres. | Values give 0.05–12 m, plausible for indoor iPhone LiDAR; consistent with iOS logging apps. Not verified against a physical measurement. | Every metric dimension scales with this. | adopted. Verify with a laser-measured reference once a capture with ground truth exists; meanwhile cross-check depth against odometry translation (PROPOSED). |
| B-04 | PROPOSED | Invalid/unreliable depth is signalled by the confidence map, not a depth sentinel. Baseline masks confidence 0; sensitivity to also masking 1 is reported. | No zero depth pixels exist (B-02), so the depth channel carries no invalid marker; confidence has values {0,1,2} with 90–94% at 2. | Unmasked bad depth would add outliers to geometry. | adopted. Confidence semantics (B-05) still open. |
| B-05 | OPEN | Confidence values {0,1,2} follow ARKit `ARConfidenceLevel` (low/medium/high). | Value set matches; not documented in the archive. | Mask threshold choice. | open. |
| B-06 | OBSERVED | Video is 1920x1440 HEVC and decodes to **one frame fewer** than depth/confidence/odometry in every capture (1714/1715, 5250/5251, 9744/9745). | manifests, `synchronization.stream_counts`. | — | resolved (count). |
| B-07 | OBSERVED | In all three captures **video frame i = odometry/depth frame i + 1**: the video lacks the first odometry frame. The dropped-frame gap pattern in video PTS matches odometry timestamps exactly at offset 1 (0 mismatches) and not at offset 0 (1021 / 3242 / 6199 mismatches for c00a170fe1 / 1a8384c3f6 / c7d28f72c6). First decoded PTS is 0.0, so OpenCV reports the current frame's PTS. | `video_to_odometry_alignment` in each manifest; `video_frame_map.csv` per capture. | Naive index pairing puts every RGB frame on the wrong pose. | resolved. Pair RGB with depth/pose through `video_frame_map.csv`. |
| B-07a | OBSERVED | Video PTS drift against the odometry clock: cumulative residual 5.7 ms over 37 s, 16.6 ms over 115 s, 29.4 ms over 215 s (≈ 140 ppm). The gap-pattern match compares per-frame steps, so the mapping in B-07 is unaffected. | Same manifest section, `max_abs_time_residual_s`. | Using video PTS as time base would misalign video with IMU by tens of ms late in a scan. | resolved. **Use odometry timestamps as the single time base**; video PTS only for frame mapping. |
| B-08 | OBSERVED | Odometry, depth and video run on a 60 Hz clock with dropped frames: median dt 16.67 ms, average 45–46 Hz, gaps up to 50 ms (515 / 1635 / 3149 gaps). Video PTS show the same gaps. | manifests, `odometry.timestamps`, `rgb_video.frame_pts_as_reported_by_opencv`. | Constant-rate assumptions would misplace frames in time. | resolved. Always use per-frame timestamps. |
| B-09 | OBSERVED | IMU at ~99.3 Hz with no gaps. Accelerometer magnitude median ≈ 1.00, so **units are g with gravity included** (not m/s²). Gyro p90 magnitude matches odometry angular rate in rad/s (ratio 0.998–1.011), so **gyro is rad/s**. | manifests, `imu`, `synchronization`. | Wrong units break any IMU fusion. | resolved. |
| B-10 | OBSERVED + OPEN | Gyro and odometry angular rate correlate at 0.999–1.0 with ≤ 5 ms lag, which means odometry orientation is fused from this IMU. This does **not** independently validate camera-to-IMU synchronization. | manifests, `synchronization.imu_vs_odometry_lag`. | Sync errors between camera and depth would go undetected. | open. |
| B-11 | ASSUMPTION | Quaternion component order is (qx, qy, qz, qw) as column names state. | All norms are 1.000000; rotation-angle checks do not depend on order. | Wrong order produces wrong orientations. | adopted. Verify in the baseline: gravity direction from the accelerometer, rotated by the pose, should be constant in world. |
| B-12 | OPEN | Pose direction (camera-to-world vs world-to-camera) and world/camera axis conventions (ARKit: world y-up, camera −z forward). | Not stated in the archive. Trajectory spans 3.6–4.8 m, consistent with a room walk either way. | Wrong direction produces garbage fusion. | open. Resolve in the baseline by testing which direction makes depth from different frames overlap; add a unit test. |
| B-13 | OBSERVED | `camera_matrix.csv` is one 3x3 RGB intrinsic matrix with **no header row** (fx = fy = 1599.70, cx = 955.51, cy = 717.81 for c00a170fe1). Odometry carries **per-frame** intrinsics that vary (52–65 unique fx values; up to 17–20 px from `camera_matrix.csv`, about 1.2%). | manifests, `camera_matrix` and `odometry.per_frame_intrinsics`. | — | resolved. |
| B-14 | PROPOSED | Use per-frame intrinsics from `odometry.csv` rather than the static `camera_matrix.csv`. | B-13: focal length changes during capture (likely autofocus). | ~1% scale error in projections. | adopted. |
| B-15 | ASSUMPTION | Depth intrinsics = RGB intrinsics scaled by 256/1920 (same 4:3 aspect), i.e. depth is a resampled, aligned view of RGB. For c00a170fe1: fx ≈ 213.3, cx ≈ 127.0, cy ≈ 95.3. | ARKit `sceneDepth` is delivered aligned to the RGB frame; no depth intrinsics or depth-to-RGB extrinsics in the archive. | Back-projection error. | adopted. Check by overlaying depth edges on RGB at the aligned frame (B-07). |
| B-16 | OBSERVED | `distortion_center_x/y` columns in odometry are empty in every row; no distortion coefficients are supplied. | manifests. | — | resolved. |
| B-17 | ASSUMPTION | Treat images as undistorted (pinhole model). | ARKit images are typically used with pinhole intrinsics; no coefficients available. | Small edge errors. | adopted. |
| B-18 | OPEN | Timestamp epoch: values start near 65,575 s (about 18.2 h), consistent with device uptime, not Unix time. | manifests. | Only matters for cross-device alignment. | open, low priority. All streams share one clock. |
| B-19 | OBSERVED + OPEN | 1a8384c3f6 has a 0.314 m translation jump into frames 5199–5200 (normal max step ≈ 0.025–0.028 m), a likely tracking jump/relocalization. | Odometry `translation.jumps_to_frame_index` in the 1a8384c3f6 manifest. | Fused geometry near the end of the scan could be duplicated or shifted. | open. Inspect in the baseline; treat as a drift/registration test case. |
| B-20 | OPEN | Capture device, app, iOS version and settings are not recorded. | Archive contents. | Device matrix and protocol cannot cite these captures. | open. |
| B-21 | OPEN | No ground truth (laser/tape) exists for the supplied captures. | Archive contents. | No accuracy metric can be computed; baseline metrics are internal consistency only. | open (blocks accuracy evaluation; see A-01). |
