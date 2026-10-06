| metric | open2_single_room | ow_single_room | open2_floor_only | ow_floor_only | open2_with_ceiling | ow_with_ceiling |
|---|---|---|---|---|---|---|
| polygon method | wall_snap | wall_snap | wall_snap | wall_snap | wall_snap | wall_snap |
| outline edges | 42 | 42 | 70 | 70 | 86 | 86 |
| observed edges | 26 | 26 | 94 | 94 | 69 | 69 |
| edges >= 0.5 m | 28 | 28 | 55 | 55 | 48 | 48 |
| outline area (m2) | 24.821 | 24.821 | 63.953 | 63.953 | 72.717 | 72.717 |
| perimeter (m) | 32.503 | 32.503 | 59.084 | 59.084 | 71.444 | 71.444 |
| wall lines detected | 19 | 19 | 56 | 56 | 55 | 55 |
| contour explained by walls | 0.565 | 0.565 | 0.598 | 0.598 | 0.531 | 0.531 |
| sliver removal | rejected: would cut off area | rejected: would cut off area | rejected: would cut off area | rejected: would cut off area | applied | applied |
| outline repaired | — | — | — | — | — | — |
| gap candidates | 5 | 5 | 4 | 4 | 6 | 6 |
| corner fills | 0 | 0 | 2 | 2 | 2 | 2 |
| rooms | 3 | 3 | 5 | 5 | 4 | 4 |
| room areas (m2) | 7.3, 8.3, 7.1 | 7.3, 8.3, 7.1 | 6.2, 7.9, 21.2, 9.6, 8.9 | 6.2, 7.9, 21.2, 9.6, 8.9 | 13.4, 19.2, 14.4, 17.1 | 13.4, 19.2, 14.4, 17.1 |
| rooms not entered | 0 | 0 | 0 | 0 | 0 | 0 |
| room outlines falling back | 0 | 0 | 0 | 0 | 0 | 0 |
| openings (widths m; ? = camera path only) | 0.50, ? | 0.50, ? | 0.99, 0.55, ?, 1.55 | 0.99, 0.55, ?, 1.55 | 0.95, ?, 0.84 | 0.95, ?, 0.84 |
| shared walls | 1 | 1 | 0 | 0 | 0 | 0 |
| ceiling per room (candidate m: status) | no ceiling plane detected; no ceiling plane detected; no ceiling plane detected | no ceiling plane detected; no ceiling plane detected; no ceiling plane detected | 1.645: implausible ceiling height 1.65 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.643: ceiling seen in 1 cells < 8; 1.657: implausible ceiling height 1.66 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.647: implausible ceiling height 1.65 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.669: implausible ceiling height 1.67 m (outside 2.0-6.0 m): likely a furniture top or another level | 1.645: implausible ceiling height 1.65 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.643: ceiling seen in 1 cells < 8; 1.657: implausible ceiling height 1.66 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.647: implausible ceiling height 1.65 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.669: implausible ceiling height 1.67 m (outside 2.0-6.0 m): likely a furniture top or another level | 3.082: ceiling seen in 6 cells < 8; 3.073: measured, SE 0.06 cm; 3.073: measured, SE 0.11 cm; 3.061: measured, SE 0.05 cm | 3.082: ceiling seen in 6 cells < 8; 3.073: measured, SE 0.06 cm; 3.073: measured, SE 0.11 cm; 3.061: measured, SE 0.05 cm |
| shared walls (thickness cm; ? = faces not parallel) | 0 () | 0 () | 3 (7, 6, 13) | 3 (7, 6, 13) | 3 (11, 14, ~16?) | 3 (11, 14, ~16?) |
| shared walls aligned / skipped (max rotation deg) | 0 / 0 (0.000) | 0 / 0 (0.000) | 3 / 0 (2.594) | 3 / 0 (2.594) | 2 / 1 (0.482) | 2 / 1 (0.482) |
| shared-wall gap area (m2) / outline jogs excluded | 0.000 / 0 | 0.000 / 0 | 0.530 / 0 | 0.530 / 0 | 0.859 / 2 | 0.859 / 2 |
| openings in walls (type width m) | O 0.55, O 0.67*, O 0.67* | O 0.55, O 0.67*, O 0.67* | O 1.09, O 1.11, O 0.86, O 0.94, O 0.65, O 0.51, W 0.66, W 0.64, O 0.77 | O 1.09, O 1.11, O 0.86, O 0.94, O 0.65, O 0.51, W 0.66, W 0.64, O 0.77 | W 1.05, W 0.84, D 0.80*, O 0.57, D 0.82*, W 2.35, O 0.74*, D 0.82*, D 0.69*, O 1.29, D 1.06* | W 1.05, W 0.84, D 0.80*, O 0.57, D 0.82*, W 2.35, O 0.74*, D 0.82*, D 0.69*, O 1.29, D 1.06* |
| openings: physical / unbounded gaps excluded / confirmed by room connection | 2 / 3 / 2 | 2 / 3 / 2 | 9 / 11 / 5 | 9 / 11 / 5 | 8 / 13 / 6 | 8 / 13 / 6 |
| physical openings: width m (n = jambs not seen) [end spread cm] | — | O 0.549, O 0.669 [0.2] | — | O 1.094, O n~1.11, O 0.861, O n~0.94, O 0.647, O 0.508, W n~0.66, W 0.642, O n~0.77 | — | W 1.053, W 0.836, D n~0.77 [0.0], O 0.570, D 0.822 [3.3], W 2.354, D n~0.87 [0.7], O 1.292 |
| physical openings: width measured / not measurable | — | 2 / 0 | — | 5 / 4 | — | 6 / 2 |
| openings seen from both sides: width difference (cm) | — | 0 | — | — | — | 6, 1, 37 |
| room overlap before / after clipping (m2) | 0.000 / 0.000 | 0.000 / 0.000 | 0.088 / 0.000 | 0.088 / 0.000 | 0.025 / 0.000 | 0.025 / 0.000 |
| outline area not in any room (m2) | 2.123 | 2.123 | 10.214 | 10.214 | 8.712 | 8.712 |
| ceiling heights (m, coverage) | — | — | — | — | 3.07 (50%), 3.07 (28%), 3.06 (18%) | 3.07 (50%), 3.07 (28%), 3.06 (18%) |
| rgb: video pairing (exact) | — | — | — | — | — | — |
| rgb: frames checked / median shift (depth px) | — | — | — | — | — | — |
| rgb: quadrant deviation (depth px) / status | — | — | — | — | — | — |
| rgb: paired frame best (fraction of frames) | — | — | — | — | — | — |
| drift correction decision | — | — | — | — | — | — |
| fused voxels raw / corrected (fewer = sharper) | — | — | — | — | — | — |
| loops candidates / used / removed after solve | — | — | — | — | — | — |
| drift keyframes accepted / rejected | — | — | — | — | — | — |
| tracking jumps at frames | — | — | — | — | — | — |
| max correction (m / deg) | — | — | — | — | — | — |
| revisit residual raw / corrected (m), informative / all pairs | — | — | — | — | — | — |
| revisit within 5 cm raw / corrected | — | — | — | — | — | — |
| ablation rooms total area raw / corrected (m2) | — | — | — | — | — | — |
| ablation rooms raw / corrected | — | — | — | — | — | — |
| half-split rooms (1st / 2nd) | 1 / 2 | 1 / 2 | 4 / 3 | 4 / 3 | 6 / 4 | 6 / 4 |
| half-split methods (1st / 2nd) | wall_snap / wall_snap | wall_snap / wall_snap | wall_snap / wall_snap | wall_snap / wall_snap | wall_snap / wall_snap | wall_snap / wall_snap |
| half-split edges (1st / 2nd) | 41 / 29 | 41 / 29 | 116 / 94 | 116 / 94 | 81 / 73 | 81 / 73 |
| half-split area diff (m2) | 1.008 | 1.008 | 9.540 | 9.540 | 0.757 | 0.757 |
| half-split perimeter diff (m) | 4.759 | 4.759 | 1.926 | 1.926 | 2.967 | 2.967 |
