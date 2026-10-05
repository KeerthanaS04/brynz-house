# Doors, windows and openings (branch `feat/openings`)

The PDF's opening gate: opening widths within 2 cm on at least 85% of openings,
where a missed opening and a phantom opening each count as a miss. Until now
only openings between rooms were found, by room segmentation. This branch
detects openings in every wall, classifies them, and measures their width,
sill and head.

**No accuracy is claimed.** No reference measurements or opening labels exist.
Widths are `uncalibrated`. The only checks available are internal: widths of
the same doorway seen from both rooms, and agreement with room segmentation.

## Method (`src/property_capture/rooms/openings.py`)

**Why rays.** A missing patch of wall cannot tell an opening from wall that was never seen. The fused point cloud cannot either: on a shared wall, the neighbouring room's points sit behind the whole wall. Rays can.

1. **Elevation grids.** For frames with the camera inside a room (every 6th frame, high-confidence depth), each depth pixel is a ray. Every wall edge of that room gets a grid (2 cm along the wall × 2 cm in height) counting:
   - **wall evidence:** the ray ends within 5 cm of the wall plane
   - **through evidence:** the ray crosses the plane and ends > 10 cm beyond it

   The grid extends 30 cm past both edge ends, because wall snapping often ends an edge at a doorway and the far jamb lies on the next stretch.
2. **Open cells:** seen through ≥ 2 times and ≥ 2× as often as seen as wall. Cells are grouped into candidates of at least 0.4 m wide by 0.3 m high, centred on their own edge.
3. **Width.** Taken from columns covering ≥ 50% of the opening's height. Each edge is refined within its cell by the through fraction of the column beyond it, because edge cells straddle the jamb. Without this, widths were biased about 4 cm short; with it, synthetic doors and windows measure within 2 cm.
4. **Acceptance.** A candidate is kept if wall is seen on both sides (jambs, ≥ 30% of its height within 10 cm), **or** if room segmentation found an opening between rooms within 0.5 m (independent evidence). Other candidates are reported as unbounded gaps and not counted.
5. **Type:**
   - **passage:** floor to within 10 cm of the ceiling
   - **door:** sill ≤ 10 cm and head ≥ 1.8 m
   - **window:** sill ≥ 30 cm
   - **opening:** anything else, e.g. the head was never seen
6. **Both sides.** The same doorway found from both rooms is linked (`same_as`) and counted once.

Outputs:
- `property.json`: each room's `openings[]` (type, wall, width measurement, sill, head, `accepted_by`, `same_as`, `matches_room_connection`)
- `opening_width` measurements
- `metrics.json` `openings` (found, unbounded gaps, both-sides widths)
- bars on `floorplan.png`: `D` door, `W` window, `P` passage, `O` other

## Results (`open2_*` runs)

| | single_room | floor_only | with_ceiling |
|---|---|---|---|
| openings found / physical | 3 / 2 | 9 / 9 | 11 / 8 |
| doors | — | — | 0.80, 0.82, 0.82, 0.69, 1.06 m |
| windows | — | 0.66, 0.64 m | 1.05, 0.84, 2.35 m |
| other openings | 0.55, 0.67, 0.67 m | 1.09, 1.11, 0.86, 0.94, 0.65, 0.51, 0.77 m | 0.57, 0.74, 1.29 m |
| accepted by jambs / by room connection | 3 / 0 | 5 / 4 | 7 / 4 |
| unbounded gaps excluded | 3 | 11 | 13 |
| same doorway from both sides: width difference | 0 cm | — | **1 cm, 6 cm, 37 cm** |

Full generated table: `comparison.md` (the both-sides row is computed by runs from this commit on).

## Findings

- **with_ceiling reads as a real plan.**
  - Every doorway between rooms is a door detected from both rooms, with typical widths 0.69–1.06 m.
  - Windows sit on outside walls (1.05, 0.84, 2.35 m).
- **Both-sides agreement is the only consistency check available.** Two doorways agree within 6 cm (0 cm in single_room, 1 cm in with_ceiling).
  - green/blue in with_ceiling differs by 6 cm (0.80 vs 0.74 m).
  - green/red differs by **37 cm** (0.69 vs 1.06 m): one side merges the door with an adjacent gap where room-03's outline has a notch. Under the PDF gate (≤ 2 cm), these disagreements would fail regardless of which side is right.
- **floor_only finds openings but no doors.** That scan aimed mostly at the floor, so door heads were rarely seen; without a head an opening cannot be classed as a door. Its doorways are reported as `opening`, which is honest rather than wrong.
- **Room connections are mostly confirmed** by detected openings (2 / 5 / 6 confirmations per capture). The 8 openings accepted only via a room connection had wall seen on one side only.

## Limitations

1. **Only openings the camera saw through are detected.** Closed doors, covered windows and glass that returns depth (rather than passing it) are missed. Under the PDF gate each counts as a miss.
2. **Width precision is unverified.** Synthetic openings measure within 2 cm; real ones disagree by up to 37 cm between sides. Next: refine jambs with RGB edges (`depth_pixel_to_rgb()` from `reports/rgb_alignment`), and reconcile both-side widths.
3. **Phantoms are possible.** Light seen through furniture can pass the open test; the jamb rule and size limits reduce but do not eliminate this. Without labels the phantom rate is unknown.
4. **Classification depends on seeing sill and head.** A door whose top was not scanned is `opening`.
5. **Thresholds** (`configs/default.yaml` `openings`) are design choices.

## Reproduce

```powershell
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/open2_single_room --set rgb.enabled=false --set drift.enabled=false
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/open2_floor_only --set rgb.enabled=false --set drift.enabled=false
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/open2_with_ceiling --set rgb.enabled=false --set drift.enabled=false
python scripts/compare_runs.py outputs/open2_single_room outputs/open2_floor_only outputs/open2_with_ceiling --out reports/openings/comparison.md
```
