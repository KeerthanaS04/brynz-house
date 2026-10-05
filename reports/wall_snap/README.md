# Wall-snapped room outline (branch `feat/wall-plane-floorplan`)

Addresses known failure 1 in `reports/baseline/phase0_baseline.md`: the Phase 0
outline followed everything at wall height (furniture, depth seen through
openings) and produced 106–190 short edges instead of walls.

**No accuracy is claimed.** The supplied captures have no reference
measurements, so this compares the two outline methods on structure and
internal consistency only. All measurements remain `uncalibrated`.

## Method (`src/property_capture/rooms/walls.py`)

1. **Wall evidence:** 4 cm floor cells whose points cover at least 50% of the wall-height band. Walls and tall cabinets pass; low furniture does not.
2. **Wall lines:** sequential RANSAC lines on those cells, split into segments at gaps over 0.25 m (doors, windows), grouped when collinear (3°, 5 cm).
3. **Outline:** the occupancy contour (all wall-height points, visible floor and the camera path) is snapped to the walls:
   - a stretch between two runs of the same wall is bridged and recorded as a **gap candidate** (possible door or window)
   - neighbouring runs on near-parallel lines within 10 cm are merged into one wall
   - a short unexplained stretch within 0.6 m of where two walls meet is closed as a **corner fill** (inferred)
   - corners are wall-line intersections
   - stretches no wall explains are kept, simplified at 15 cm, with edges marked inferred
4. **Validity:** slivers narrower than 15 cm are removed only if the room keeps at least 95% of its area. A self-crossing outline is repaired on a 1 cm raster, then retried with finer simplification, before falling back to the occupancy outline.

No Manhattan or parallel-wall prior is used. `floorplan.polygon_method: occupancy` reproduces the Phase 0 outline on the same code.

## Results

Same code and config, only `floorplan.polygon_method` differs. Edge support (observed vs inferred) uses the tall-cell evidence for both methods. The half-split check builds independent plans from the first and second half of each scan.

| metric | single_room occupancy | single_room wall_snap | floor_only occupancy | floor_only wall_snap | with_ceiling occupancy | with_ceiling wall_snap |
|---|---|---|---|---|---|---|
| outline edges | 106 | **42** | 190 | **70** | 185 | **86** |
| observed edges | 34 | 21 | 67 | 31 | 65 | 41 |
| edges ≥ 0.5 m | 24 | 19 | 39 | 29 | 43 | 46 |
| outline area (m²) | 21.94 | 24.82 | 66.93 | 63.95 | 77.83 | 72.72 |
| perimeter (m) | 41.92 | 32.50 | 72.39 | 59.08 | 75.67 | 71.44 |
| wall lines detected | — | 19 | — | 56 | — | 55 |
| outline explained by walls | — | 57% | — | 60% | — | 53% |
| gap candidates / corner fills | — | 5 / 0 | — | 4 / 2 | — | 6 / 2 |
| fallbacks (main + 2 halves) | — | 0 | — | 0 | — | 0 |
| half-split area diff (m²) | 4.17 | **1.01** | 9.31 | 9.54 | 0.48 | 0.76 |
| half-split perimeter diff (m) | 4.37 | 4.76 | 7.77 | **1.93** | 9.79 | **2.97** |

Full generated table: `comparison.md`.

## Reading the numbers

- **Structure:** 54–63% fewer edges on every capture. Outer walls become single long edges, e.g. 3.45, 2.70, 2.74 and 6.34 m in with_ceiling, and 4.50 m along the right wall in single_room.
- **Half-split consistency is mixed, not uniformly better:**
  - better: area for single_room (4.17 → 1.01 m²); perimeter for floor_only (7.77 → 1.93 m) and with_ceiling (9.79 → 2.97 m)
  - unchanged: area for floor_only (9.31 → 9.54 m²)
  - slightly worse: area for with_ceiling (0.48 → 0.76 m²)
  - The remaining area differences come from coverage: each half sees different parts of the space.
- **Only 53–60% of the outline lies on a detected wall.** The rest borders space seen through openings or partly scanned areas (e.g. the north room in with_ceiling), and those edges are marked inferred.

## Limitations and next steps

1. **floor_only and with_ceiling are multi-room spaces** (64–73 m²). Interior dividing walls are detected (pink lines in `floorplan.png`) but not yet used to split rooms. Next branch: room segmentation.
2. **Gap candidates are not openings yet:** they carry position and width but no door/window classification and no height check.
3. **Thresholds are design choices, not calibrated:** tall fraction 0.5, gap 0.25 m, corner fill 0.6 m, sliver width 0.15 m. Each is documented in `configs/default.yaml` and needs validation once reference captures exist.
4. **Poses are still used as-is** (drift gate `would_fail`).

## Reproduce

```powershell
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/cmp_occupancy_single_room --set floorplan.polygon_method=occupancy
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/cmp_wall_snap_single_room
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/cmp_occupancy_floor_only --set floorplan.polygon_method=occupancy
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/cmp_wall_snap_floor_only
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/cmp_occupancy_with_ceiling --set floorplan.polygon_method=occupancy
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/cmp_wall_snap_with_ceiling
python scripts/compare_runs.py outputs/cmp_occupancy_single_room outputs/cmp_wall_snap_single_room outputs/cmp_occupancy_floor_only outputs/cmp_wall_snap_floor_only outputs/cmp_occupancy_with_ceiling outputs/cmp_wall_snap_with_ceiling --out reports/wall_snap/comparison.md
```
