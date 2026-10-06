# Benchmark capture and reference measurement protocol

The PDF requires a benchmark set we capture ourselves, "specified so it cannot be
flattered", with laser or tape ground truth on everything. Every accuracy gate
waits for it. This document says what to capture and how to measure it so the
pipeline can be scored.

## Composition (PDF Part 2, verbatim requirements)

| # | Required | How we meet it |
|---|---|---|
| 1 | One multi-room capture, three or more rooms plus a connector | e.g. 3 rooms + hallway, one 3D scan following `docs/capture_protocol.md` |
| 2 | One furnished room with staged damage spanning two damage classes | e.g. a stain plus a crack (or hole) in one room, photographed close up as well |
| 3 | The same rooms at all three tiers, the multi-room set included; photos as per-room folders | photo + video + 3D scan of the same property, same day |
| 4 | At least one room captured twice at the same tier (repeatability gate) | a second, separate 3D scan of one room |
| 5 | Laser or tape ground truth on everything; raw sensor data and measurements submitted | reference CSV below + raw recordings in `data/raw/` |

## Reference measurements

**Tools:** a laser distance meter (record its model) and a tape for short items.

**Measure each item twice.** The two readings show the reference's own precision. If they differ by more than 5 mm, measure a third time.

| Quantity | Where exactly | Per |
|---|---|---|
| `wall_length` | corner to corner along the wall, at about 1 m height | every wall of every room |
| `ceiling_height` | floor to ceiling, at three spots: room centre and two opposite corners about 0.5 m from the walls | every room |
| `opening_width` | jamb to jamb (inside of the frame), at about 1 m height | every door, window and open passage |
| `opening_height` | floor (doors) or sill (windows) to the top of the frame | every door and window |
| `sill_height` | floor to the bottom of the window frame | every window |
| `damage_extent` | longest dimension of each damage region | each staged damage |

Record everything in a copy of `docs/templates/references_template.csv`:
- one row per reading
- labels as you name things on site (`room-A`, `door-1`, `wall-north`)
- a sketch or photo of where each label is

## Linking references to pipeline output

The pipeline names rooms, walls and openings itself (`room-02`, `room-02-wall-10`, `opening-04`). After a run:
1. Open `outputs/<run>/floorplan.png`.
2. Write the pipeline id of each referenced item into the `pipeline_id` column.
3. If the pipeline did not detect an item, leave `pipeline_id` empty: that is a **miss** under the opening gate. A detected opening with no reference row is a **phantom**.

This manual link is deliberate: automatic matching could flatter the results.

## Scoring

Once the reference CSV has `pipeline_id`s filled in, run:

```powershell
python -m property_capture evaluate --references data/references.csv --run scan-01=outputs/<run of scan-01> --run scan-02=outputs/<run of scan-02>
```

This takes one `--run` per `capture_id` in the CSV. It writes a fresh `evaluation_<time>/` folder inside the first run:
- `per_measurement.csv`: every reference with its pipeline value, error and status
- `gate_results.json`: the result of every gate
- `evaluation.md`: a summary

Add `--strict` to exit with code 1 if any gate fails.

Where the PDF's wording is ambiguous, both readings are scored, and the gate reports `ambiguous` if they disagree:
- what counts in the opening gate's denominator (assumptions.md A-03)
- whether repeatability means "1 cm or 0.5%" either way (A-04)

## Repeat capture (repeatability gate)

- Scan the chosen room twice, as separate recordings, starting from a different doorway.
- Do not move anything between the two scans.
- Record both recording names in the reference CSV `notes`.
