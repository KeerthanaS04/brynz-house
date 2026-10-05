# Shared walls between rooms (branch `feat/shared-walls`)

Each room's outline snaps to its own face of a wall it shares with a
neighbour, so a shared wall appeared as two unrelated parallel edges. This
branch pairs those faces into walls, reports wall thickness where it can be
measured, and checks how much of the floor area that belongs to no room is
wall thickness.

**No accuracy is claimed.** No reference measurements exist. Thickness is an
`uncalibrated` measurement inferred from room outlines.

## Method (`src/property_capture/rooms/shared_walls.py`)

1. **Face pairs.** An edge of room A and an edge of room B are paired if they:
   - are nearly parallel (≤ 5°)
   - face each other (each room's outward normal points at the other)
   - are 3–40 cm apart
   - overlap along the wall by ≥ 30 cm
2. **Walls.** Pairs between the same two rooms with collinear centrelines form one wall:
   - Thickness is the overlap-weighted median of the pairs.
   - Pairs more than 5 cm from it are **outline jogs** (one room snapped to a different feature): listed and excluded.
3. **Measurability.** Thickness is reported only if the kept faces stay parallel (thickness varies ≤ 4 cm along the overlap). Otherwise `not_measurable` with the variation and a candidate value.
4. **Outputs:**
   - `property.json` `shared_walls[]`: rooms, face pairs, centrelines, overlap, thickness status, excluded jogs
   - `wall_thickness` measurements
   - `metrics.json` summary
   - centrelines with `t N cm` (or `t ~N cm?`) labels on `floorplan.png`

## Results (`sw2_*` runs)

| | single_room | floor_only | with_ceiling |
|---|---|---|---|
| shared walls | 0 | 3 | 3 |
| thickness measured | — | **7, 6 cm** | **11 cm** |
| not measurable (faces not parallel) | — | 1 (~14 cm) | 2 (~14, ~16 cm) |
| outline jogs excluded | — | 1 | 2 (incl. a 34 cm stretch) |
| area between paired faces | 0 | 0.44 m² | 0.85 m² |
| outline area in no room | 2.12 m² | 10.21 m² | 8.71 m² |

Full generated table: `comparison.md`.

## Findings

- **Measured thicknesses (6–11 cm) are typical of interior partitions.**
- **Half the shared walls have faces that are not parallel.** The two rooms snapped to their own detected wall lines, whose directions differ by roughly 1–2°; over 2–4 m that exceeds 4 cm of divergence. So the stitched plan does not yet agree with itself on shared wall directions, which also limits wall-length accuracy.
- **Grouping fixed the first-pass artifacts.** The first pass reported a 34 cm wall (with_ceiling) where one room's outline jogged; it is now excluded as a jog. A face pair diverging by 14 cm (floor_only) is now `not_measurable` instead of a misleading single number.
- **Wall thickness is a small part of the floor area in no room** (0.44 of 10.2 m², 0.85 of 8.7 m²). The rest comes from the 6 cm wall barriers drawn during room segmentation along every wall, plus seen-but-unassigned regions.
- **single_room has no shared walls:** its two rooms meet only through the connecting space, with no facing edges within 40 cm.

## Limitations and next step

1. **Shared walls are detected, not enforced.** Next: snap both faces of a shared wall to one common direction (and optionally a common centreline with the measured thickness). This would make the stitched plan consistent and the thickness measurable on every shared wall. It is a geometry change, so it needs a before/after comparison of wall lengths and room areas.
2. **Thickness comes from outlines,** which are snapped to detected wall lines at 2 cm grid resolution. There is no direct measurement through the wall.
3. **Thresholds** (5°, 3–40 cm, 30 cm overlap, 5 cm jog, 4 cm parallelism) are design choices, in `configs/default.yaml` `floorplan.shared_walls`.

## Reproduce

```powershell
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/sw2_single_room --set rgb.enabled=false --set drift.enabled=false
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/sw2_floor_only --set rgb.enabled=false --set drift.enabled=false
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/sw2_with_ceiling --set rgb.enabled=false --set drift.enabled=false
python scripts/compare_runs.py outputs/sw2_single_room outputs/sw2_floor_only outputs/sw2_with_ceiling --out reports/shared_walls/comparison.md
```
(RGB check and drift correction are off only for speed: they do not change the floor plan on these captures, because the drift correction is rejected.)
