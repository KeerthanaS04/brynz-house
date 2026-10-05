# Ceiling height measurability rule (branch `fix/ceiling-coverage-rule`)

Replaces known failure 3 in `reports/baseline/phase0_baseline.md`: a room's
ceiling height was reported only if the ceiling plane covered >= 30% of the
room, a threshold with no basis. Coverage says how much was seen, not whether
the height is trustworthy. In with_ceiling, two rooms with clean, level
ceiling planes were rejected by it.

**No accuracy is claimed.** No reference measurements exist. The standard error
is a precision indicator from the data, not a calibrated interval.

## Rule (`src/property_capture/rooms/ceiling.py`)

Per room, ceiling-plane points and floor points inside the room are reduced to
one level per 25 cm cell. Neighbouring points are not independent; cells are.
The ceiling height is reported only if all hold:

| Check | Threshold | Why |
|---|---|---|
| ceiling seen in enough cells | ≥ 8 cells (0.5 m²) | enough independent evidence |
| ceiling is flat | per-cell levels IQR ≤ 1.5 cm | stepped ceilings, soffits and beams have no single height; 1.5 cm is the PDF tolerance |
| height is precise | standard error ≤ 0.5 cm | 3 standard errors fit inside the 1.5 cm gate |
| height is plausible | 2.0–6.0 m | rejects the tops of tall furniture just above the camera |

- **Height** = median of the ceiling's per-cell levels − median of the floor's per-cell levels in the same room. With fewer than 8 floor cells in the room, the floor comes from all floor points, and the output says which was used.
- **Standard error** = robust spread of per-cell levels (1.4826 × MAD) / √cells, combined for ceiling and floor. A 2 mm noise floor on the spread stops a few identical cells from claiming zero error.
- **The precision check mostly guards the floor.** With ≥ 8 cells, a ceiling cannot exceed the standard-error limit without first failing the flatness check, so this check mainly catches an uneven or poorly seen floor (e.g. a raised platform).
- **Coverage is still reported,** for information only.

## Results (old rule: `drift6_*` runs; new rule: `ceil_*` runs; otherwise identical code)

| | old rule (30% coverage) | new rule |
|---|---|---|
| with_ceiling | 1 of 4 rooms measured: 3.08 m (50%) | **3 of 4 measured**: 3.073 m (SE 0.06 cm, 50%), 3.073 m (SE 0.11 cm, 28%), 3.061 m (SE 0.05 cm, 18%); 1 room seen in 6 cells < 8 |
| floor_only | 0 measured (coverage 0–2%) | 0 measured: 4 rooms *implausible ceiling height 1.64–1.67 m (furniture top)*, 1 room seen in 1 cell |
| single_room | no ceiling plane | no ceiling plane |

Full generated table: `comparison.md`.

- **The two with_ceiling rooms the old rule rejected (28% and 18% coverage) are now measured,** with standard errors of about 1 mm.
- **floor_only is rejected for the right reason.** The 1.65 m "ceiling" is a horizontal surface just above the camera; it is now named as implausible rather than "low coverage".
- **Heights moved about 1 cm** (3.08 → 3.07, 3.07 → 3.06) because the floor level is now measured inside each room instead of taken from one global floor plane. Without references we cannot say which is closer to the truth.
- **The three measured rooms differ by 1.2 cm** (3.061 vs 3.073 m). This may be real (floors or ceilings at slightly different levels) or estimation error. It is within the tolerance either way.

## Limitations

1. **Thresholds are tied to the PDF tolerance but not yet validated.** Whether a 1 mm standard error means 1.5 cm accuracy needs laser references (benchmark captures).
2. **One ceiling plane per scan.** Rooms whose ceiling is at a different height from the scan's dominant ceiling are not detected. Their candidate is `no ceiling points in this room`.
3. **Sloped ceilings are not modelled.** They fail the flatness check and are reported `not_measurable`.
4. **The standard error is not an interval.** Calibrated uncertainty needs reference measurements (REQ-07).

## Reproduce

```powershell
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/ceil_single_room
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/ceil_floor_only
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/ceil_with_ceiling
python scripts/compare_runs.py outputs/drift6_single_room outputs/ceil_single_room outputs/drift6_floor_only outputs/ceil_floor_only outputs/drift6_with_ceiling outputs/ceil_with_ceiling --out reports/ceiling/comparison.md
```
