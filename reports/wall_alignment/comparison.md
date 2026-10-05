| metric | sw2_floor_only | align2_floor_only | sw2_with_ceiling | align2_with_ceiling | align_single_room |
|---|---|---|---|---|---|
| polygon method | wall_snap | wall_snap | wall_snap | wall_snap | wall_snap |
| outline edges | 70 | 70 | 86 | 86 | 42 |
| observed edges | 95 | 94 | 69 | 69 | 26 |
| edges >= 0.5 m | 55 | 55 | 48 | 48 | 28 |
| outline area (m2) | 63.953 | 63.953 | 72.717 | 72.717 | 24.821 |
| perimeter (m) | 59.084 | 59.084 | 71.444 | 71.444 | 32.503 |
| wall lines detected | 56 | 56 | 55 | 55 | 19 |
| contour explained by walls | 0.598 | 0.598 | 0.531 | 0.531 | 0.565 |
| sliver removal | rejected: would cut off area | rejected: would cut off area | applied | applied | rejected: would cut off area |
| outline repaired | — | — | — | — | — |
| gap candidates | 4 | 4 | 6 | 6 | 5 |
| corner fills | 2 | 2 | 2 | 2 | 0 |
| rooms | 5 | 5 | 4 | 4 | 3 |
| room areas (m2) | 6.2, 7.9, 21.2, 9.6, 8.9 | 6.2, 7.9, 21.2, 9.6, 8.9 | 13.4, 19.2, 14.4, 17.1 | 13.4, 19.2, 14.4, 17.1 | 7.3, 8.3, 7.1 |
| rooms not entered | 0 | 0 | 0 | 0 | 0 |
| room outlines falling back | 0 | 0 | 0 | 0 | 0 |
| openings (widths m; ? = camera path only) | 0.99, 0.55, ?, 1.55 | 0.99, 0.55, ?, 1.55 | 0.95, ?, 0.84 | 0.95, ?, 0.84 | 0.50, ? |
| shared walls | 0 | 0 | 0 | 0 | 1 |
| ceiling per room (candidate m: status) | 1.645: implausible ceiling height 1.65 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.643: ceiling seen in 1 cells < 8; 1.657: implausible ceiling height 1.66 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.647: implausible ceiling height 1.65 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.669: implausible ceiling height 1.67 m (outside 2.0-6.0 m): likely a furniture top or another level | 1.645: implausible ceiling height 1.65 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.643: ceiling seen in 1 cells < 8; 1.657: implausible ceiling height 1.66 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.647: implausible ceiling height 1.65 m (outside 2.0-6.0 m): likely a furniture top or another level; 1.669: implausible ceiling height 1.67 m (outside 2.0-6.0 m): likely a furniture top or another level | 3.082: ceiling seen in 6 cells < 8; 3.073: measured, SE 0.06 cm; 3.073: measured, SE 0.11 cm; 3.061: measured, SE 0.05 cm | 3.082: ceiling seen in 6 cells < 8; 3.073: measured, SE 0.06 cm; 3.073: measured, SE 0.11 cm; 3.061: measured, SE 0.05 cm | no ceiling plane detected; no ceiling plane detected; no ceiling plane detected |
| shared walls (thickness cm; ? = faces not parallel) | 3 (7, 6, ~14?) | 3 (7, 6, 13) | 3 (11, ~14?, ~16?) | 3 (11, 14, ~16?) | 0 () |
| shared walls aligned / skipped (max rotation deg) | — | 3 / 0 (2.594) | — | 2 / 1 (0.482) | 0 / 0 (0.000) |
| shared-wall gap area (m2) / outline jogs excluded | 0.441 / 1 | 0.530 / 0 | 0.854 / 2 | 0.859 / 2 | 0.000 / 0 |
| room overlap before / after clipping (m2) | 0.088 / 0.000 | 0.088 / 0.000 | 0.025 / 0.000 | 0.025 / 0.000 | 0.000 / 0.000 |
| outline area not in any room (m2) | 10.214 | 10.214 | 8.712 | 8.712 | 2.123 |
| ceiling heights (m, coverage) | — | — | 3.07 (50%), 3.07 (28%), 3.06 (18%) | 3.07 (50%), 3.07 (28%), 3.06 (18%) | — |
| rgb: video pairing (exact) | — | — | — | — | — |
| rgb: frames checked / median shift (depth px) | — | — | — | — | — |
| rgb: quadrant deviation (depth px) / status | — | — | — | — | — |
| rgb: paired frame best (fraction of frames) | — | — | — | — | — |
| drift correction decision | — | — | — | — | — |
| fused voxels raw / corrected (fewer = sharper) | — | — | — | — | — |
| loops candidates / used / removed after solve | — | — | — | — | — |
| drift keyframes accepted / rejected | — | — | — | — | — |
| tracking jumps at frames | — | — | — | — | — |
| max correction (m / deg) | — | — | — | — | — |
| revisit residual raw / corrected (m), informative / all pairs | — | — | — | — | — |
| revisit within 5 cm raw / corrected | — | — | — | — | — |
| ablation rooms total area raw / corrected (m2) | — | — | — | — | — |
| ablation rooms raw / corrected | — | — | — | — | — |
| half-split rooms (1st / 2nd) | 4 / 3 | 4 / 3 | 6 / 4 | 6 / 4 | 1 / 2 |
| half-split methods (1st / 2nd) | wall_snap / wall_snap | wall_snap / wall_snap | wall_snap / wall_snap | wall_snap / wall_snap | wall_snap / wall_snap |
| half-split edges (1st / 2nd) | 116 / 94 | 116 / 94 | 81 / 73 | 81 / 73 | 41 / 29 |
| half-split area diff (m2) | 9.540 | 9.540 | 0.757 | 0.757 | 1.008 |
| half-split perimeter diff (m) | 1.926 | 1.926 | 2.967 | 2.967 | 4.759 |
