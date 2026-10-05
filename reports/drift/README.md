# Drift accountability (branch `feat/drift-handling`)

PDF drift gate: "Your report states what you do about accumulated drift on the
multi-room capture (loop closure, pose graph, plane-anchored correction,
anything), and an ablation shows the stitched footprint with it on and off.
'Poses used as-is' is an automatic fail on this row."

**Result on the supplied captures:** drift is measured, a pose-graph correction
is computed and validated against held-out revisits, and on every capture the
validation **rejects** the correction. On this data the device poses are more
consistent than any correction we could measure. The final poses are therefore
the device odometry, and the gate is reported honestly as `would_fail` with the
evidence. It should be re-evaluated on the benchmark multi-room capture.

**No accuracy is claimed.** No ground-truth trajectory or reference measurements
exist. All drift evidence is internal: agreement between frames that see the
same surfaces at different times.

## What the pipeline does (`drift.enabled: true`, default)

1. **Tracking jumps.** Odometry steps over 0.10 m are detected (floor_only: frames 5199–5200, 0.31 m) and get keyframes on both sides.
2. **Correction: 4-DoF pose graph** (`registration/pose_graph.py`, VINS-Mono style; heading about gravity plus position, tilt from the device):
   - **Nodes:** keyframes, every 0.3 m or 15° of camera motion.
   - **Odometry edges:** keep the device's relative motion, weighted by an assumed drift rate (assumptions.md B-22).
   - **Loop edges:** keyframes ≥ 10 s apart with ≥ 30% view overlap, registered to each other by 4-DoF point-to-plane ICP. A loop is used only if it passes all of:
     - fit quality: inlier fraction, residual, non-degenerate geometry
     - plausibility for the distance walked
     - forward-backward consistency: a→b and b→a must undo each other within 0.5° and 3 cm
   - **Solve:** loops are robustly weighted, and loops still inconsistent after solving are removed.
3. **Held-out validation.** Revisit pairs use only non-keyframe frames, so they are never the pairs the loops were built from:
   - A pair counts only if the raw or the corrected poses make it agree within 25 cm; occluded or different-surface pairs are dropped for both trajectories.
   - The correction is **applied only if it beats the device poses on both median residual and fraction within 5 cm**, over ≥ 10 informative pairs.
4. **Ablation.** The plan is built with both trajectories in the same run (`floorplan.png` from the kept poses, `floorplan_<other>_poses.png` from the other). Raw and corrected trajectories are saved in `intermediates/trajectories.csv/.npz`, and loops and their verdicts in `intermediates/drift_report.json`.

## Results (final configuration, `drift6_*` runs)

| | single_room | floor_only | with_ceiling |
|---|---|---|---|
| keyframes | 77 | 237 | 457 |
| tracking jumps | — | frames 5199, 5200 | — |
| loop candidates / used | 24 / 1 | 67 / 1 | 37 / 2 |
| informative revisit pairs | 0 of 32 | 16 of 40 | 38 of 38 |
| **device poses**: median residual, within 5 cm | — | **0.95 cm, 83%** | **2.6 cm, 76%** |
| corrected poses: median residual, within 5 cm | — | 3.6 cm, 76% | 6.8 cm, 41% |
| largest correction (m / deg) | 0.11 / 1.5 | 0.10 / 4.3 | 0.32 / 1.3 |
| decision | rejected: no informative pairs | rejected: worse | rejected: worse |
| plan, rooms total area device / corrected (m²) | 22.7 / 23.2 | 53.7 / 54.2 | 64.0 / 64.2 |

Full generated table: `comparison.md`.

## What was tried (six rounds on the three captures)

| Round | Method | Outcome |
|---|---|---|
| 1 | Keyframe-to-map ICP, fixed limit of 10 cm / 2° per keyframe | Accepted implausible steps (9 cm over 0.3 m of motion); correction wandered 3.7° over 37 s. No revisit pairs for evidence. |
| 2 | Same, limit scaled with distance walked since the matched map region was scanned | Correction shrank (0.07 m / 1.2°); revisit pairs still too strict to find any. |
| 3 | Revisit pairs chosen by view overlap | **First evidence: correction worse than device poses** (with_ceiling 2.7 cm → 63 cm). Rejected keyframes were contaminating the map. |
| 4 | Only accepted keyframes extend the map; validate before applying | Deadlock: the map stopped growing, so almost every keyframe was rejected. Still worse than device poses. |
| 5 | **4-DoF pose graph**: device odometry edges plus loop edges from pairwise ICP | Still worse (with_ceiling 2.6 → 5.6 cm). Loop log: heading errors of 2–5° measured where poses agree to 2.6 cm, so pairwise ICP heading is unreliable. |
| 6 | Forward-backward loop check; drop uninformative validation pairs | 1–2 loops survive per capture. With occluded pairs removed, device poses agree to 0.95–2.6 cm; correction still worse. |

## Why the correction loses here

- **The device tracker is already very consistent.** Revisits ≥ 10 s apart agree to 0.95 cm (floor_only) and 2.6 cm (with_ceiling) median. The tracker evidently performs its own relocalization; the 0.31 m "jump" in floor_only may be such a correction, not an error.
- **Depth-only pairwise registration at 256×192 cannot measure heading as accurately as the device tracks it.** Almost all loop registrations fail the forward-backward check.
- **The drift-rate assumption (B-22: 2%, 1°/m) is much looser than this device's actual drift.** That lets even one or two loops bend the trajectory by decimetres.

## Limitations and next steps

1. **Drift gate stays `would_fail` on the supplied captures.** The device poses are kept after validation. Whether "measured, attempted, validated and rejected" counts as accountability is the evaluator's call; this report gives the evidence either way.
2. **Re-evaluate on the benchmark multi-room capture.** A longer multi-room walk is where real drift is likely to show. If revisits there disagree, the same pipeline will apply the correction when it validates.
3. **Better loop measurements** would make the correction competitive:
   - RGB features (the video is not used yet)
   - full-resolution matching
   - registering larger submaps on both sides
4. **Tighten the drift model (B-22)** from measured device consistency instead of a generic rate.
5. **The revisit metric is internal consistency, not accuracy.** Pairs are chosen by overlap under device poses, which favours the device poses. single_room has no informative pairs: its revisits see different surfaces.

## Reproduce

```powershell
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/drift6_single_room
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/drift6_floor_only
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/drift6_with_ceiling
python scripts/compare_runs.py outputs/drift6_single_room outputs/drift6_floor_only outputs/drift6_with_ceiling --out reports/drift/comparison.md
# superseded keyframe-to-map method, for comparison:
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/drift_kf2map_with_ceiling --set drift.method=keyframe_to_map
```
