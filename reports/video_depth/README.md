# Video tier, step 2: depth network, metric scale and a floor plan (branch `feat/video-depth`)

Step 1 (`reports/video_tier`) recovered camera motion from the video alone. The walk came out
in pieces with unrelated, unknown scales. This step adds a pretrained depth network to give each
piece metric scale and dense geometry, tries to join the pieces, and runs the LiDAR tier's room
stages on the result.

**Result: the video tier produces a dimensioned plan, but only of the largest piece of the walk.**
That piece's scale is right to within 4% on floor_only (in both runs), off by 21% on with_ceiling, and
on single_room 3.6% in one run but −17% in a repeat run (step-1 SfM is not deterministic). The plan
covers 7–22% of the property's area (as measured by the LiDAR tier). Joining the pieces failed.
Making it work would need either a recapture that follows the protocol (not planned) or a much
larger reconstruction model than this laptop's 4 GB GPU can run.

## Method

`python -m property_capture video-plan --video-run <step-1 run>`: [video_pipeline.py](../../src/property_capture/video_pipeline.py), [modalities/video_depth.py](../../src/property_capture/modalities/video_depth.py)

**Inputs:** the step-1 run folder, the video it names, and the network weights. Nothing else from the capture is read (rule 6, enforced by `tests/unit/test_video_depth.py`).

1. **Depth per frame:** Depth Anything V2 Metric Indoor Small (assumptions.md B-24), on the GPU (about 0.15 s per frame). Pixels at depth edges are dropped, because the network blurs edges into points in mid-air.
2. **Runs:** each SfM piece is split into runs of consecutive frames. A run is kept only if its motion agrees with the links below.
3. **Links between consecutive frames:** feature matches plus the first frame's network depth give the second frame's pose (PnP). This needs no parallax. Each frame is also linked from the 2 frames before, as a cross-check.
4. **Depth correction per frame.** The network depth is multiplied by one factor per frame. The factors for each connected part are solved together in log scale: links and runs give tight constraints, and the network's own scale is a loose prior. Metres means the network's scale on the median frame, uncalibrated.
5. **Up direction:** taken as the direction perpendicular to all camera x axes (the phone is held upright), then refined on the floor plane.
6. **Shared stages:** the largest connected part goes through the same fusion, floor plan and openings code as the LiDAR tier. These stages were moved to [plan_outputs.py](../../src/property_capture/plan_outputs.py). The regression check on single_room shows 0 differences (`scripts/diff_runs.py`).
   - The output is `property.json` with `input_tier: video`.
   - Every measurement is flagged `scale_from_depth_network`, and `video_partial_coverage` when part of the walk is missing.

**Evaluation (`video-oracle`, evaluation only)** compares the video tier with the device's own data:
- the trajectory against device poses, fitted with scale and without (the latter tests the metric scale);
- network depth and corrected depth against LiDAR depth;
- the video plan against the LiDAR-tier plan.

## What the network depth is good for (single_room, vs LiDAR)

| | Median ratio to LiDAR | p10–p90 | Median error |
|---|---|---|---|
| Network depth as is | 1.07 | 0.74–1.85 | 26% |
| Rescaled by one best factor per frame | — | — | 5% |
| Corrected inside an SfM run (final run) | 1.01 | 0.98–1.07 | 3.6% |

The network gets each frame's **shape** right but not its overall **scale**. That factor changes by 7% between consecutive frames (median; 21% at the 90th percentile), and below 1 m the network reads 1.9× too far. So the network cannot give metric scale frame by frame. It can over a run of frames tied together by SfM.

## Tuning rounds (single_room)

| Round | Links | Pieces | Frames in main piece | Main piece scale error | Rigid fit error | Corrected depth / LiDAR (p10–p90) |
|---|---|---|---|---|---|---|
| 1 | 800 px, 40 inliers, network scale per run | 8 | 29 | 3.8% | 2.5 cm | (not corrected) |
| 2 | 1200 px, 20 inliers, 3 px, 50% agree; scale carried link to link | 9 | 29 | 3.2% | 2.2 cm | 1.00 (0.98–1.06) |
| 3 | + 6 px, 25% agree, reverse links, frames from the blurred gaps | 16 | 131 | 45% | 54 cm | 0.98 (0.74–1.01) |
| 4 | + scales solved together, cross-check links | 17 | 95 | 14% | 13 cm | 0.80 (0.55–0.86) |
| final | round 2 links + round 4 scale solve | 9 | 29 | 3.6% | 2.4 cm | 1.01 (0.98–1.07) |

*Scale error* is 1/s − 1, where s is the scale that fits the video trajectory to the device poses. *Rigid fit error* is the trajectory error with no scale allowed (RMS).

- **Rounds 1–2:** links failed at the same blurred fast turns where SfM broke. Failed links had a median of 19 matches.
- **Round 3:** looser links joined more frames, but wrong links got in. One reported a depth-scale change of 3.8×10¹⁴, and scale errors of about 2% per link multiplied over 300 links.
- **Round 4:** solving the scales together and cross-checking links stopped the collapse, but not the drift.

**Every change that joined more frames made the scale worse.** The final settings are the conservative ones of round 2. The looser options remain in the config, switched off.

## Final results, all captures (`vfinal_*`; LiDAR reference `ow_*`)

| | single_room | floor_only | with_ceiling |
|---|---|---|---|
| Frames | 170 | 253 | 495 |
| SfM runs accepted | 5 of 6 | 6 of 9 | 15 of 21 |
| Links succeeded | 155 of 230 | 133 of 309 | 225 of 582 |
| Connected parts | 9 | 15 | 36 |
| Main part: frames (share of walk) | 29 (17%) | 60 (24%) | 29 (6%) |
| Main part: placed by | one SfM run | one SfM run | one SfM run |
| Main part: path length | 2.0 m | 9.6 m | 3.9 m |
| **Main part: scale error** | **3.6%** | **3.3%** | **21%** |
| Main part: rigid fit error | 2.4 cm (1.2%) | 6.0 cm (0.6%) | 22 cm (5.6%) |
| Corrected depth / LiDAR (p10–p90) | 1.01 (0.98–1.07) | 1.11 (0.92–1.23) | 1.20 (1.13–1.36) |
| Scale error, other parts | −18% to +84% | −11% to +50% | −33% to +88% |
| Up direction refined on floor | no (one viewing direction) | no | no |
| **Plan area: video vs LiDAR** | **4.96 vs 22.70 m² (22%)** | **9.24 vs 53.70 m² (17%)** | **4.79 vs 64.00 m² (7%)** |
| Rooms: video vs LiDAR | 1 vs 3 | 1 vs 5 | 1 vs 4 |

## Repeat run (`outputs/repro_full`, `scripts/reproduce.py`)

The whole video tier was rerun from the raw ZIPs at the same settings.

| | single_room | floor_only | with_ceiling |
|---|---|---|---|
| SfM pieces (step 1) | **7** (was 6) | 9 (same) | 17 (same) |
| Main part frames | **36** (was 29) | 60 | 29 |
| Main part scale error | **−17%** (was +3.6%) | 3.4% (was 3.3%) | 21% (was 21%) |
| Main part rigid fit error | **11.6 cm** (was 2.4 cm) | 6.1 cm | 22 cm |
| Corrected depth / LiDAR | **0.78** (was 1.01) | 1.11 | 1.20 |
| Plan area / LiDAR plan | 22% | 17% | 7% |

- **Step 1 is not deterministic.** On the same 170 single_room frames, COLMAP's incremental mapping gave 7 pieces instead of 6 despite a fixed random seed (it runs multi-threaded). The plan then came from a different piece, and that piece's network scale is 17% off.
- **So the 3.6% single_room scale in the first run was partly luck.** Only floor_only gave a scale within 4% on both runs.
- Taken together, the network gives about 3% on some pieces and 17–21% on others, and nothing in the video alone tells which.

## Findings

- **No link ever joined another part to the main part.** In every capture the plan comes from one SfM run, i.e. one stretch of walking between two fast turns.
- **The network's metric scale holds for some stretches and not others.** The main run's scale is within about 3–4% with no measurement on floor_only (twice) and on single_room once. It is 17% off on single_room's repeat run and 21% off on with_ceiling, and other runs range from −33% to +88%. Nothing available from the video alone tells which runs are right. The ±3% video wall-length gate therefore needs a reference measurement.
- **Wall lengths cannot be compared with the LiDAR plan.** The video plan covers one partial room, so its edges do not correspond to the LiDAR plan's walls. The up direction was never refined on the floor plane, because the main parts look in too few directions, so the plan's frame may be tilted by up to about 20°.
- **The cause is in the capture, not the processing.** Fast turns on the spot blur the frames and leave no parallax. Capture-protocol step 12 ("turn while you walk, never on the spot") targets exactly this. A protocol-compliant recording would test it, but none is planned, so this remains untested.

## What would close the gap (not done)

1. **A recapture that follows step 12.** Expected to remove most breaks; untested.
2. **A learned reconstruction model** (VGGT, MASt3R-SLAM, DROID-SLAM) that estimates poses and depth jointly and copes with rotation-only motion. These need roughly 8–24 GB of GPU memory (this laptop has 4 GB), and some need compiled CUDA extensions.
3. **A user-measured reference length** for metric scale. Needed for the ±3% gate in any case.

## Reproduce

```powershell
python scripts/fetch_weights.py
python -m property_capture video-plan --video-run outputs/video5_single_room --output outputs/vfinal_single_room
python -m property_capture video-oracle --video-run outputs/vfinal_single_room --capture data/raw/single_room/c00a170fe1 --lidar-run outputs/ow_single_room
# floor_only and with_ceiling: same with video5_floor_only / ow_floor_only and video5_with_ceiling / ow_with_ceiling
```

Round settings are recorded in `configs/default.yaml` (`video_depth`), and each run's full config is in its `run_info.json`.
