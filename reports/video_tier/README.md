# Video tier, step 1: camera motion from video alone (branch `feat/video-tier`)

The PDF requires three input tiers. The video tier is "a handheld walkthrough
clip from any iPhone 15 or newer": RGB video only, with no depth, no device
poses, no intrinsics and no IMU. This branch builds the first step, recovering
camera motion and sparse structure from the video, and measures how well it
works on the supplied captures.

**Result: no floor plan yet.** Within short stretches the video tier tracks
the camera accurately, but the walk breaks into pieces with unrelated scales,
so there is no single trajectory or scale to build a plan on.

## Method (`src/property_capture/modalities/video_sfm.py`)

- **Frame selection:** the sharpest frame (variance of the Laplacian) in each 1/6 s window; frames blurrier than half the median are dropped.
- **Reconstruction:** COLMAP via `pycolmap` 4.2.1 (CPU):
  - SIFT features (1600 px, up to 8192 per image)
  - sequential matching (each frame with the next 20, plus quadratic overlap)
  - incremental mapping with one shared camera whose focal length is estimated
  - mapper thresholds loosened for small walkthrough baselines (`configs/default.yaml` `video`)
- **Inputs read:** only the video file. A unit test fails if the module imports or mentions depth, poses, IMU or the camera matrix (`CLAUDE_updated.md` rule 6).
- **Outputs:** the largest piece's trajectory, all pieces (`trajectory.json`), sparse points (`sparse_points.ply`), stats (`video_sfm.json`). `scale_status: unknown`.

**Evaluation (`evaluation/video_oracle.py`, `video-oracle` command, evaluation only).** Each piece is aligned to the device camera centres by a least-squares similarity (Umeyama), using the video/device frame pairing from the gap pattern. Reported per piece: the residual (ATE) and the scale factor. Device poses are a consistency reference (assumptions.md B-23: 1–3 cm), not ground truth.

## Tuning on single_room (`video` … `video4`)

| Round | Change | Largest piece | In any piece | Pieces | ATE (largest) |
|---|---|---|---|---|---|
| 1 | 3 fps, 1024 px, 4096 features, overlap 10, COLMAP default mapper | 18 / 93 | — | 4 | 1.1 cm (0.5%) |
| 2 | 6 fps, 1600 px, 8192 features, overlap 20, mapper thresholds loosened | 36 / 170 | — | 7 | 3.7 cm (1.6%) |
| 3 | + exhaustive matching (all pairs) | 43 / 170 | 160 / 170 | 7 | 1.1 cm (0.4%) |
| 4 | sequential + structure-less registration fallback | 43 / 170 | 163 / 170 | 7 | 1.3 cm (0.4%) |

**Why the walk breaks into pieces.** The pieces are consecutive stretches of the walk, and the breaks fall where the phone **turns on the spot**:

| Between consecutive frames | Rotation | Speed | Rotation per metre walked |
|---|---|---|---|
| Both frames placed | 19 °/s | 0.43 m/s | 50 °/m |
| At a break | 38 °/s | 0.21 m/s | 175 °/m |

Turning in place gives no parallax, so no 3D points can be built for the newly seen area, and the next stretch has nothing to attach to. Neither all-pairs matching nor COLMAP's structure-less fallback bridged these points.

## Results, all captures (`video5_*`; single_room at 6 fps, the larger two at 3 fps)

| | single_room | floor_only | with_ceiling |
|---|---|---|---|
| frames used | 170 | 253 | 495 |
| pieces | 6 | 9 | 17 |
| frames in any piece | 88% | 76% | 69% |
| largest piece | 36 frames | 61 frames | 48 frames |
| pieces within 1% of path length | 5 of 6 | 7 of 8 | 13 of 17 |
| median piece ATE (% of path) | 0.83% | 0.53% | 0.57% |
| worst piece | 3.6 cm (1.6%) | 79.6 cm (18.6%) | 81.3 cm (13.4%) |
| scale ratio between pieces (max/min) | 2.3 | 5.1 | 4.6 |

## Findings

- **Most pieces track the camera within about 1% of distance travelled** (median 0.5–0.8%).
- **Some pieces are wrong** (up to 19% of path). The video tier cannot tell them from good pieces; only the device poses reveal them here.
- **Pieces cannot be combined.** Each has its own scale (up to 5× apart) and its own frame, so there is no single trajectory, no metric scale and no floor plan from video yet.
- **Longer walks fragment more:** 6 pieces for one room, 17 for a multi-room scan.

## What a video floor plan needs (not in this branch)

1. **One scale across pieces, and denser geometry.** A pretrained monocular depth model (PDF: pretrained models allowed with disclosure) gives per-frame depth: relative scale between pieces from overlapping depth, and surfaces where sparse points are too thin. Alternatively a GPU enables COLMAP dense stereo, but that still leaves the pieces unjoined.
2. **Metric scale:** one user-measured distance, or a metric depth model. Otherwise report scale as unknown with widened intervals (PDF).
3. **Capture protocol:** "turn while you walk, never on the spot" (added to `docs/capture_protocol.md` step 12) directly targets the cause of the breaks.
4. **Rejecting bad pieces without device poses:** e.g. reprojection and track-length checks per piece. Not attempted.

## Reproduce

```powershell
python -m property_capture video --input data/raw/single_room/c00a170fe1 --output outputs/video5_single_room
python -m property_capture video-oracle --video-run outputs/video5_single_room --capture data/raw/single_room/c00a170fe1
python -m property_capture video --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/video5_floor_only --set video.target_fps=3
python -m property_capture video-oracle --video-run outputs/video5_floor_only --capture data/raw/single_scan_floor_only/1a8384c3f6
python -m property_capture video --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/video5_with_ceiling --set video.target_fps=3
python -m property_capture video-oracle --video-run outputs/video5_with_ceiling --capture data/raw/single_scan_with_ceiling/c7d28f72c6
```
