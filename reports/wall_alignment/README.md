# Shared-wall alignment (branch `feat/shared-wall-alignment`)

Follow-up to `reports/shared_walls/README.md`: half the shared walls had faces
that were not parallel. Each room snapped to its own detected wall line, and
the lines differed by 1–2°, so the stitched plan disagreed with itself on
shared wall directions and their thickness was not measurable.

**No accuracy is claimed.** No reference measurements exist. Alignment makes
the plan self-consistent; it does not make it correct.

## Method (`src/property_capture/rooms/wall_alignment.py`)

1. **Common direction:** the length-weighted mean direction of all faces of a shared wall, from both rooms.
2. **Rotate each face** to that direction about its own midpoint. The face keeps its position, so the mean wall thickness is preserved. Corners are re-intersected with the neighbouring edges, or projected onto the face when the neighbour is nearly parallel (< 20°). Between parallel neighbours the room's area is unchanged exactly: the two corner triangles cancel.
3. **Accept only if** both outlines stay simple, neither room's area changes by more than 2%, and no new overlap with any room appears. Before skipping, two fallbacks are tried:
   - a corner overlap of ≤ 0.05 m² between the wall's own two rooms is trimmed from the smaller room by exact subtraction
   - if aligning all faces fails, only the longest face pair is aligned
4. **Re-pair the walls,** so thickness is measured on the aligned faces. `floorplan.shared_walls.align: false` turns alignment off.

## Results (before: `sw2_*`; after: `align2_*`; otherwise identical code)

| | floor_only before | floor_only after | with_ceiling before | with_ceiling after |
|---|---|---|---|---|
| shared walls | 3 | 3 | 3 | 3 |
| thickness (cm; ? = faces not parallel) | 7, 6, ~14? | **7, 6, 13** | 11, ~14?, ~16? | **11, 14**, ~16? |
| walls aligned / skipped | — | 3 / 0 | — | 2 / 1 |
| largest rotation | — | 2.59° | — | 0.48° |
| room area changes | — | ≤ 0.026 m² | — | ≤ 0.0003 m² |
| room areas (m²) | 6.2, 7.9, 21.2, 9.6, 8.9 | unchanged | 13.4, 19.2, 14.4, 17.1 | unchanged |

Full generated table: `comparison.md`.

- **floor_only: all three shared walls are now parallel and measured** (7, 6, 13 cm). The wall that needed 2.59° had faces about 3.5° apart; the shorter face rotated most. It also needed a 0.011 m² corner trim from room-03. Both changes are recorded in `metrics.json` `wall_alignment.fallbacks`, and `floorplan.png` shows two straight parallel faces there.
- **with_ceiling: 2 of 3 walls are measured** (11, 14 cm). The third (room-02 / room-03, ~16 cm) is skipped: aligning it would make room-02's outline self-intersect, and after outline jogs are excluded it has a single face pair, so there is no smaller fallback. It stays `not_measurable` with that reason.
- **Room areas, openings, ceilings and outline totals are unchanged** to the reported precision.

## Limitations

1. **Consistency is not accuracy.** Each face keeps its own position, so a room whose outline is offset stays offset. Only direction is reconciled.
2. **Alignment is per wall.** A room corner shared by two aligned walls is recomputed by each in turn. Overlaps are checked after every wall, but walls are not solved jointly.
3. **One with_ceiling wall remains unaligned.** Handling it would need editing the notch in room-02's outline, not just rotating the face.
4. **Thresholds** (2% area, 0.05 m² trim, 20° corner) are design choices, in `configs/default.yaml` `floorplan.shared_walls`.

## Reproduce

```powershell
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/align2_floor_only --set rgb.enabled=false --set drift.enabled=false
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/align2_with_ceiling --set rgb.enabled=false --set drift.enabled=false
# before: the same commands with --set floorplan.shared_walls.align=false
python scripts/compare_runs.py outputs/sw2_floor_only outputs/align2_floor_only outputs/sw2_with_ceiling outputs/align2_with_ceiling outputs/align_single_room --out reports/wall_alignment/comparison.md
```
