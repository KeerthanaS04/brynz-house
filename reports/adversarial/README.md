# Adversarial and ground-truth tests on synthetic captures (branch `feat/adversarial-tests`)

The supplied captures have no reference measurements, so no stage had been checked against known
geometry. This branch adds synthetic captures with known geometry, written in the supplied captures' exact
format, and uses them for two things:
- **Correctness:** the LiDAR tier must recover a known room.
- **Robustness (CLAUDE_updated.md, adversarial tests):** broken input must stop with the reason, and
  difficult input must either give the right answer or flag what it cannot measure.

Generator: [tests/synthetic_capture.py](../../tests/synthetic_capture.py)
- **Room:** a 4 × 3 m box with a 2.5 m ceiling.
- **Depth:** ray-cast from a camera walking an ellipse at 1.4 m height while turning and tilting, then written as 256 × 192 depth and confidence PNGs.
- **Odometry:** camera-to-world, xyzw quaternions, OpenCV axes, RGB intrinsics.
- **IMU:** gravity in g, in the camera frame.
- **Camera matrix, and a small `rgb.mp4`** that only supplies the RGB resolution.

Tests: [tests/integration/test_synthetic_lidar.py](../../tests/integration/test_synthetic_lidar.py) and [test_video_adversarial.py](../../tests/integration/test_video_adversarial.py). About 40 s in total.

## Found and fixed: wall lines sat up to 2 cm off the wall (fix loop)

**Symptom.** The clean box came out with exact lengths (4.000 / 3.000 m) and area (12.00 m²), but **every wall
was 2.0 cm off its true position**: the whole plan was shifted by exactly 2 cm.

**Root cause.** Wall lines are fitted by RANSAC to the **centres of 4 cm tall-wall cells**, not to the points.
- A wall surface can lie anywhere inside its cell, so each line is off by up to half a cell (2 cm), and a length by up to 4 cm.
- In the box, all four walls lie on cell boundaries and moved the same way, so lengths stayed exact.
- In real rooms the error differs per wall, so it reaches the lengths. That is the same size as the PDF's 2 cm opening-width tolerance.

**Fix.** `rooms/walls.py: refine_offsets` (config `floorplan.walls.refine_offsets`) moves each wall line along
its normal by the median offset of the wall-band points within one cell of it, inside its extent.

**Before / after:**

| | Before | After |
|---|---|---|
| Synthetic box, wall position error | 2.0 cm (every wall) | ≤ 0.6 cm |
| Synthetic box, lengths and area | exact | exact |
| Real captures, 130 wall lines: perpendicular move | — | median 0.6 cm, p90 1.4 cm, max 3.4 cm; no direction changes |
| single_room total area | 22.70 m² | 22.75 m² |
| floor_only total area | 53.70 m² | 53.79 m² |
| with_ceiling total area | 64.00 m² | 62.53 m² |
| floor_only shared-wall thickness | 6.7 / 6.1 / 13.4 cm | 4.7 / 4.1 / 12.8 cm |
| with_ceiling ceiling heights | unchanged | unchanged |

- The with_ceiling change of −1.5 m² is almost all in room-03's poorly observed end, bounded by inferred edges. The outline there now cuts around furniture.
- So outlines in unobserved regions are **sensitive to small wall shifts**: a 1–2 cm move changed an inferred region by 1.8 m². Those edges are already flagged `inferred_low_support`.
- Whether the real walls are now closer to the truth needs laser references (benchmark captures). The synthetic box shows the bias is gone.

Regenerate: before = `outputs/repro_lidar` (previous commit), after = `outputs/repro_lidar_walls`, both from
`python scripts/reproduce.py --tiers lidar`.

## Broken captures: stop with the reason

| Input | Before this branch | Now |
|---|---|---|
| Missing (NaN) timestamp | ran, silently | `odometry.csv: 1 row(s) with missing or non-numeric values ... ['timestamp']` |
| Timestamps not increasing | stopped | stopped (unchanged) |
| NaN position | "no floor plane found below the camera trajectory" | `odometry.csv: ... columns ['x']` |
| Zero quaternion | ran, silently | `quaternion(s) far from unit length (norm 0 at frame 59)` |
| Zero focal length | "no pose convention produced overlapping, consistent depth" | `invalid intrinsics, e.g. frame 0: fx 0 ...` |
| Principal point outside image | (not tested) | `invalid intrinsics` |
| IMU row with missing values | ran, silently | `imu.csv: ... columns ['a_x', 'a_y', 'a_z']` |
| IMU in m/s² instead of g | (not tested) | `median acceleration 9.81, expected about 1 (units of g, B-09)` |
| All depth zero | "no pose convention ..." | `only 0.0% of sampled depth pixels are usable ... in other units (B-03)` |
| Depth 10× too large (wrong units) | "no pose convention ..." | same message, with median depth 15.85 m |
| Missing depth PNG | stopped | stopped (unchanged) |
| Featureless video (video tier) | cryptic failure | `structure from motion reconstructed nothing; the video has too little texture, sharpness or camera movement` |

The real captures pass all the new checks: 95–98% usable depth (stop below 5%, `DEPTH_SPARSE` warning below 40%).

## Difficult captures

| Capture | Result | Test |
|---|---|---|
| Clean box | 12.00 m², walls 4/3/4/3 m within 2 cm, positions within 5 mm, ceiling 2.50 ± 0.01 m, all walls observed | `test_clean_box_gives_true_dimensions` |
| World frame tilted 30° and 20° (gravity not along +y) | box recovered: up comes from the IMU | `test_gravity_not_along_world_y` |
| 50% of depth pixels missing at random | area within 6% (11.45 m² before the wall fix); observed walls on the true walls; the rest inferred | `test_depth_dropout_keeps_observed_walls_true` |
| Camera never turns to one wall | observed walls (≥ 20 cm) on the true walls; unseen side inferred; area underestimated | `test_partial_coverage_keeps_observed_walls_true` |
| Camera looking slightly down throughout | **crashed** (degenerate room region in the half-split check); now completes, box recovered | `test_walk_looking_down_completes` |
| 0.3 m pose jump mid-walk | walls 3.68 m instead of 4.0, **no warning**; now a `TRACKING_JUMP` warning names the frame | `test_tracking_jump_is_reported` |

On the real captures, `TRACKING_JUMP` fires only on floor_only, at frames 5199–5200: the jump already recorded in assumptions.md B-19.

## Known limitations (tests document them)

- **A tracking jump is reported, not repaired.** The walls stay 0.3 m short. Drift correction, when enabled, rejects its own correction here. `test_tracking_jump_is_repaired` is marked as an expected failure (`xfail`, strict) and will flag when this gets fixed.
- **Short corner jogs can count as observed.** Edges of about 10 cm with 3 support points pass the 50% observed-fraction rule while sitting about 11 cm off a wall (partial coverage). The wall-position tests only check edges of at least 20 cm.
- **Partial coverage underestimates area.** The plan covers only what was seen, and the room is labelled "not entered" when the camera path lies outside the seen floor.
- **Inferred outlines are unstable** (see the fix-loop section).
- **Not simulated:** mirrors, glass, wet-look surfaces and low light. They need either real captures or a renderer with reflections; the synthetic box is an ideal sensor.

## Also changed

- **`property.json` coordinate frame:** it now states the convention. A 2D point is (P · x_axis_world, P · y_axis_world), and `origin_world` is a point on the floor, not the 2D origin. The old description implied the coordinates were measured from `origin_world`.
- **Degenerate room regions** (fewer than 3 corners) are dropped and counted (`segmentation.degenerate_regions_dropped`). If none is left, the plan fails with the reason.
