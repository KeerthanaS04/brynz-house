# Device matrix

Which input tier runs on which hardware, and what accuracy each tier honestly
delivers today. PDF Part 1: "You submit a device matrix stating which tier runs
on which hardware and what accuracy each tier honestly delivers."

**Accuracy against ground truth is unknown for every tier.** The supplied
captures have no reference measurements (assumptions.md B-21). The figures
below are internal consistency checks, not accuracy.

| Tier | Hardware | Capture tool | Inputs used | Pipeline status | What we can say about accuracy today |
|---|---|---|---|---|---|
| **LiDAR (3D scan)** | iPhone 12 Pro or newer with LiDAR (walk-in: iPhone 15 Pro / Pro Max, 16 Pro / Pro Max) | Stray Scanner (assumptions.md B-20) | depth 256×192 + confidence, RGB video 1920×1440, device poses, per-frame intrinsics, IMU | **Implemented:** `python -m property_capture run` | Internal only (see below) |
| **Video** | any iPhone 15 or newer | built-in Camera app | RGB video only; no depth, no poses | **Partial:** `python -m property_capture video` recovers camera motion and sparse points, in pieces with unknown and unrelated scales; **no floor plan yet** | Camera motion within pieces: median 0.5–0.8% of distance travelled vs device poses; some pieces wrong (up to 19%); pieces cannot be combined (`reports/video_tier`) |
| **Photo** | any iPhone 15 or newer | built-in Camera app | 2–8 stills per room, one folder per room; no depth, no poses | **Not implemented** (BLOCKED, REQ-01) | None |

## LiDAR tier: internal consistency on the supplied captures

| Check | Result | Source |
|---|---|---|
| Depth agreement between frames ~0.3 s apart | 5–7 mm median | `reports/baseline` |
| Device pose consistency at revisits ≥ 10 s apart | 0.95–2.6 cm median | `reports/drift` |
| Depth vs RGB alignment | constant 0.4–0.5 depth px offset, corrected; no scale error | `reports/rgb_alignment` |
| Floor plane flatness | < 1 cm RMS, < 0.5° tilt | `reports/baseline` |
| Ceiling height precision (where measurable) | standard error 0.5–1.1 mm (3 of 4 rooms in with_ceiling) | `reports/ceiling` |
| Opening width, same doorway from two rooms (both jambs seen) | 0.7 cm | `reports/opening_widths` |
| Shared wall thickness | 6–14 cm where faces are parallel | `reports/shared_walls`, `reports/wall_alignment` |

What these do **not** show: absolute scale error, wall-length error, ceiling height error, or opening width error against a laser. These need the benchmark captures (`docs/benchmark_protocol.md`).

## Known hardware and scene limits (LiDAR tier)

- **Range:** depth is used up to 5 m. Large rooms need walking closer to far walls.
- **Glass and mirrors:** may return no depth or false surfaces behind them. Windows are detected only where the camera saw through them (`reports/openings`).
- **Low light:** depth still works, but RGB-based steps (alignment check) degrade.
- **Wet or glossy floors:** may cause depth dropouts. Not tested on the supplied captures.
