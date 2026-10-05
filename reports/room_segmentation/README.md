# Room segmentation (branch `feat/room-segmentation`)

Addresses known failure 2 in `reports/baseline/phase0_baseline.md` and the
multi-room limitation in `reports/wall_snap/README.md`: floor_only and
with_ceiling are multi-room spaces but came out as one outline, with one
ceiling coverage figure diluted across rooms where no ceiling was scanned.

**No accuracy is claimed.** The supplied captures have no reference
measurements or room labels, so room counts, areas and openings are
unverified. All measurements remain `uncalibrated`.

## Method (`src/property_capture/rooms/segmentation.py`)

1. **Walls as barriers.** Detected wall segments (already split at gaps) are drawn 6 cm thick into the occupancy grid. Doorways stay open.
2. **Room seeds.** Free space at least 0.5 m (half of `door_max_width_m` = 1.0 m) from any obstacle. Passages narrower than 1.0 m close, so each room becomes its own seed. Open-plan openings wider than 1.0 m do not split rooms.
3. **Growth.** Seeds grow back over free space level by level, in order of decreasing distance to obstacles. Rooms meet where free space is narrowest and never through walls or outside space. Regions under 2 m² join the neighbour they touch most.
4. **Per-room outline.** The same wall snapping as the single outline, then exact polygon subtraction (`shapely`) so no two rooms overlap. A room loses only the area a larger neighbour covers; all other edges keep their geometry.
5. **Per-room ceiling.** Height and coverage use only the ceiling-plane points inside each room. The 30% coverage rule now applies per room.
6. **Connections** between room pairs:
   - **opening:** the rooms meet through free space (width measured), and/or the camera moved between them. If seen only from the camera path, the width is `not_measurable`.
   - **shared wall:** the rooms are within 30 cm of each other across a wall (both wall faces are detected).
7. **Rooms not entered.** A room holding under 2% of the camera path is flagged `room_not_entered`.

`floorplan.segmentation.enabled: false` restores the single outline on the same code.

## Results

Segmentation off vs on, same code (`seg_off_*` vs `seg_on3_*`).

| metric | single_room off | on | floor_only off | on | with_ceiling off | on |
|---|---|---|---|---|---|---|
| rooms | 1 | 3 | 1 | 5 | 1 | 4 |
| room areas (m²) | 24.8 | 7.3, 8.3, 7.1 | 64.0 | 6.2, 7.9, 21.2, 9.6, 8.9 | 72.7 | 13.4, 19.2, 14.4, 17.1 |
| openings (width m; ? = camera path only) | — | 0.50, ? | — | 0.99, 0.55, ?, 1.55 | — | 0.95, ?, 0.84 |
| shared walls | — | 1 | — | 0 | — | 0 |
| room overlap before / after clipping (m²) | — | 0.000 / 0.000 | — | 0.088 / 0.000 | — | 0.025 / 0.000 |
| outline area in no room (m²) | — | 2.12 | — | 10.21 | — | 8.71 |
| ceiling height measured (rooms) | 0 | 0 | 0 | 0 | 0 | **1 (3.08 m, 50%)** |
| edges ≥ 0.5 m | 19 | 28 | 29 | 55 | 46 | 48 |
| half-split rooms (1st / 2nd half) | 1 / 1 | 1 / 2 | 1 / 1 | 4 / 3 | 1 / 1 | 6 / 4 |

Full generated table: `comparison.md`.

## Reading the numbers

- **with_ceiling** reads as four rooms around a central room, joined by 0.95 m and 0.84 m doorways and one passage found from the camera path. One room now has a measurable ceiling: 3.08 m over 50% of its floor. The single outline's 23% coverage hid it. The other rooms' ceiling candidates (3.07–3.09 m) stay `not_measurable`, at 0–28% coverage.
- **floor_only:** five rooms. The 1.65 m horizontal surface is still rejected as a ceiling in every room (0–2% coverage).
- **single_room** splits into two rooms plus a connecting space. The two right-hand rooms are separated by a detected wall the camera never crosses, and connect only through the third region. Whether this is one room with a tall partition or two rooms is **not verifiable** from this data.
- **No room overlaps** after clipping, which the PDF stitch gate requires. Before clipping, overlaps were small (≤ 0.09 m²).
- **Outline area in no room** is 2.1 / 10.2 / 8.7 m², 9–16% of the outline. Most of it is the 6 cm wall barriers and the strip between the two faces of each wall, which are not room floor. In with_ceiling it also includes a region west of the central room that is seen but not assigned (grey in `floorplan.png`).
- **Room count is not stable across halves of a scan:** 1 vs 2 for single_room, 4/3 vs 5 for floor_only, and 6/4 vs 4 for with_ceiling. Each half sees different parts of the space, and partial coverage changes where the narrow passages appear.

## Limitations and next steps

1. **Room counts are unverified.** There are no room labels for the supplied captures, and the benchmark captures (PDF: 3+ rooms plus a connector, laser ground truth) are needed.
2. **Coverage-dependent splits.** The half-split instability shows segmentation depends on how much of each passage was scanned.
3. **The door threshold (1.0 m) is a design choice.** Wider doorways merge rooms; narrow furniture gaps along a wall can split one. The other thresholds are in `configs/default.yaml` (`floorplan.segmentation`).
4. **Openings are room-to-room only.** No door/window classification, no windows within a room, no opening height.
5. **Shared-wall consistency between neighbouring rooms is not enforced.** Both rooms snap to their own wall face; wall thickness is not reported.
6. **Poses are used as-is.** Drift between rooms is uncorrected (drift gate `would_fail`).

## Reproduce

```powershell
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/seg_off_single_room --set floorplan.segmentation.enabled=false
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/seg_on3_single_room
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/seg_off_floor_only --set floorplan.segmentation.enabled=false
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/seg_on3_floor_only
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/seg_off_with_ceiling --set floorplan.segmentation.enabled=false
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/seg_on3_with_ceiling
python scripts/compare_runs.py outputs/seg_off_single_room outputs/seg_on3_single_room outputs/seg_off_floor_only outputs/seg_on3_floor_only outputs/seg_off_with_ceiling outputs/seg_on3_with_ceiling --out reports/room_segmentation/comparison.md
```
