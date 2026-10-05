# Phase 0 RGB-D baseline

First end-to-end run of `python -m property_capture run` on the three supplied
captures, before any tuning. Run directories (`outputs/run_baseline_*_01/`) are
git-ignored and regenerable with the commands below; this file records their
numbers. Code: git commit containing this file; config: `configs/default.yaml`.

**No accuracy is claimed.** The supplied captures have no reference
measurements (assumptions.md A-01, B-21), so every measurement is
`uncalibrated` and every accuracy gate is `not_evaluable`. The drift gate is
reported `would_fail` because poses are used as-is.

## Commands

```powershell
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/run_baseline_single_room_01
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/run_baseline_floor_only_01
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/run_baseline_with_ceiling_01
```

Runtime on the development laptop: 21 s, 50 s, 95 s.

## Results

| Metric | single_room (c00a170fe1) | floor_only (1a8384c3f6) | with_ceiling (c7d28f72c6) |
|---|---|---|---|
| Pose convention selected | xyzw, camera-to-world, OpenCV axes | same | same |
| Multi-view depth residual, selected / runner-up | 5.2 / 132 mm | 6.7 / 180 mm | 6.3 / 159 mm |
| Best depth scale (0.001 m/count = 1.0) | 1.0 | 1.0 | 1.0 |
| Gravity direction spread, best / runner-up | 3.6 / 7.8° | 3.6 / 7.4° | 3.3 / 6.6° |
| Frames fused (stride 3) | 572 | 1751 | 3249 |
| Floor plane RMS / tilt from up | 7.8 mm / 0.41° | 8.1 mm / 0.13° | 6.9 mm / 0.04° |
| Median camera height above floor | 1.41 m | 1.41 m | 1.46 m |
| Ceiling candidate | none | 1.66 m (not a ceiling; 4% coverage) | 3.08 m (22% coverage) |
| Ceiling status | not_measurable | not_measurable | not_measurable (coverage < 30%) |
| Room outline edges | 106 | 190 | 185 |
| Outline area | 21.9 m² | 66.9 m² | 77.8 m² |
| Half-split area (first / second half of frames) | 10.5 / 14.6 m² | 41.3 / 50.6 m² | 69.8 / 70.2 m² |

## What works

- Conventions are resolved from data with a wide margin (B-11, B-12, B-12a).
- Depth scale agrees with odometry scale (B-03).
- Floor planes are flat to < 1 cm RMS and level to < 0.5°.
- The ceiling coverage rule rejected a false 1.66 m "ceiling" (a horizontal surface above the camera) in floor_only.

## Known failures (input to the fix loop)

1. **Outline is not a wall plan.** The contour follows everything in the 0.3–2.0 m height band, including furniture and depth seen through doors and windows, giving 106–190 short edges instead of a handful of walls. *Partly addressed by the wall-snapped outline (`reports/wall_snap/README.md`): 42–86 edges, outer walls as single edges; 40–47% of the outline is still unexplained by walls.*
2. **Outline area depends on coverage.** single_room half-split areas differ by 4.2 m² because each half sees different parts of the room. floor_only and with_ceiling likely span more than one room (67–78 m²); room segmentation is not implemented. *Room segmentation added (`reports/room_segmentation/README.md`): 5 and 4 rooms; room counts unverified and unstable across scan halves.*
3. **Ceiling coverage threshold (30%) is unjustified.** with_ceiling has a clean 3.08 m plane (RMS 7.3 mm, tilt 0.03°) seen over 22% of the outline, which is itself inflated by failure 1. The threshold needs a basis before it decides measurability.
4. **Drift: poses used as-is.** The floor_only 0.31 m tracking jump (B-19) is fused unchanged.
5. **Not in baseline:** openings, damage, RGB use, calibrated intervals, photo and video tiers.
