# Opening widths: measure only between observed jambs (branch `feat/opening-widths`)

Follow-up to `reports/openings/README.md`. The same doorway seen from both
rooms of with_ceiling disagreed by up to 37 cm. Under the PDF opening gate
(widths ≤ 2 cm on ≥ 85% of openings) that is a miss whichever side is right.

**No accuracy is claimed.** No reference widths exist. The rule below decides
where a width can be claimed at all; whether claimed widths are within 2 cm
needs laser-measured openings.

## Diagnosis (with_ceiling, `open2` run)

| doorway | widths from each room | jambs seen (each room, each end) |
|---|---|---|
| room-01 / room-02 | 0.825 / 0.818 m | both ends, from both rooms |
| room-00 / room-02 | 0.80 / 0.74 m | 0.00–0.08 everywhere: almost none |
| room-02 / room-03 | 0.69 / 1.06 m | one end from both rooms; **the other end from neither** |

- **Where both jambs were seen, the two rooms agree within 0.7 cm.**
- **The 37 cm disagreement is one unobserved end.** At the end where both rooms saw the jamb, their estimates agree. At the other end, each room's opening just ran until the evidence ran out (−0.09 vs +0.21 m along the wall). That end of the doorway was never observed, so neither number is a measurement.

## Rule (`fuse_physical` in `src/property_capture/rooms/openings.py`)

- Each end of a detection is a **jamb** only if wall was seen beside it (≥ 30% of the opening's height within 10 cm). Otherwise it is open-ended and its position is not used.
- **Physical opening.** The detections of one doorway from different rooms form one physical opening. Each end's position is the mean of the estimates from rooms that saw a jamb there; a room can contribute just the jamb it saw.
- **Width = distance between the two ends,** reported only if both ends have at least one jamb estimate. Otherwise it is `not_measurable` with the reason and a candidate width.
- **End spread:** the largest disagreement between rooms on an end's position. A consistency figure, not an interval.
- **Outputs:**
  - one `opening_width` measurement per measurable physical opening
  - unmeasurable widths listed in `unobservable`
  - `property.json` top-level `openings[]` (physical) linked from each room's detections
  - plan labels `D 0.82`, or `D ~0.69?` when not measurable

## Results (before: `open2_*`, one width per detection; after: `ow_*`)

| | single_room | floor_only | with_ceiling |
|---|---|---|---|
| physical openings | 2 | 9 | 8 |
| width measured / not measurable | 2 / 0 | 5 / 4 | 6 / 2 |
| measured widths (m) | 0.549, 0.669 | 1.094, 0.861, 0.647, 0.508, 0.642 (W) | 1.053 (W), 0.836 (W), 0.570, **0.822 (D)**, 2.354 (W), 1.292 |
| not measurable (candidate) | — | 1.11, 0.94, 0.66 (W), 0.77 | 0.77 (D), 0.87 (D) |
| end spread for doorways seen from both rooms | 0.2 cm | — | 0.0, 3.3, 0.7 cm |

Full generated table: `comparison.md`.

- **13 of 19 physical openings have a measured width,** each between two observed jambs. The 37 cm doorway and the room-00/room-02 doorway are now `not_measurable` (one unobserved end, and no jambs seen, respectively) instead of reporting two contradicting numbers.
- **The room-01/room-02 doorway measures 0.822 m.** The two rooms' widths agree within 0.7 cm, but each end's position differs by 3.3 cm between rooms in the same direction. That is a small along-wall offset between the two rooms' outlines, not a width error.
- **Under the PDF gate, unmeasurable widths are still misses:** 6 of 19 here, plus any openings not detected at all. Measurability depends on how the capture covered each doorway's sides, which the capture protocol can address.

## Limitations

1. **Accuracy of measured widths is unverified.** Synthetic openings measure within 2 cm; real ones need laser references (benchmark captures).
2. **Edge precision is one 2 cm cell,** refined by the through fraction. RGB refinement of jamb positions was not added: without reference widths its benefit cannot be shown, so it waits for the benchmark captures.
3. **Coverage, not the method, limits measurability.** A doorway whose jamb was never in view cannot be measured. The capture protocol should ask for a look at both sides of each doorway.

## Reproduce

```powershell
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/ow_single_room --set rgb.enabled=false --set drift.enabled=false
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/ow_floor_only --set rgb.enabled=false --set drift.enabled=false
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/ow_with_ceiling --set rgb.enabled=false --set drift.enabled=false
python scripts/compare_runs.py outputs/open2_single_room outputs/ow_single_room outputs/open2_floor_only outputs/ow_floor_only outputs/open2_with_ceiling outputs/ow_with_ceiling --out reports/opening_widths/comparison.md
```
