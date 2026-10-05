| metric | drift6_single_room | drift6_floor_only | drift6_with_ceiling |
|---|---|---|---|
| polygon method | wall_snap | wall_snap | wall_snap |
| outline edges | 42 | 70 | 86 |
| observed edges | 26 | 95 | 69 |
| edges >= 0.5 m | 28 | 55 | 48 |
| outline area (m2) | 24.821 | 63.953 | 72.717 |
| perimeter (m) | 32.503 | 59.084 | 71.444 |
| wall lines detected | 19 | 56 | 55 |
| contour explained by walls | 0.565 | 0.598 | 0.531 |
| sliver removal | rejected: would cut off area | rejected: would cut off area | applied |
| outline repaired | — | — | — |
| gap candidates | 5 | 4 | 6 |
| corner fills | 0 | 2 | 2 |
| rooms | 3 | 5 | 4 |
| room areas (m2) | 7.3, 8.3, 7.1 | 6.2, 7.9, 21.2, 9.6, 8.9 | 13.4, 19.2, 14.4, 17.1 |
| rooms not entered | 0 | 0 | 0 |
| room outlines falling back | 0 | 0 | 0 |
| openings (widths m; ? = camera path only) | 0.50, ? | 0.99, 0.55, ?, 1.55 | 0.95, ?, 0.84 |
| shared walls | 1 | 0 | 0 |
| room overlap before / after clipping (m2) | 0.000 / 0.000 | 0.088 / 0.000 | 0.025 / 0.000 |
| outline area not in any room (m2) | 2.123 | 10.214 | 8.712 |
| ceiling heights (m, coverage) | — | 1.65 (1%), 1.64 (0%), 1.66 (2%), 1.64 (1%), 1.67 (2%) | 3.09 (0%), 3.08 (50%), 3.08 (28%), 3.07 (18%) |
| drift correction decision | rejected: too few informative revisit pairs to validate (0 of 32 < 10) | rejected: corrected poses agree worse at revisits than device poses | rejected: corrected poses agree worse at revisits than device poses |
| fused voxels raw / corrected (fewer = sharper) | 373034 / 358580 | 1265604 / 1266083 | 2388868 / 2493865 |
| loops candidates / used / removed after solve | 24 / 1 / 0 | 67 / 1 / 0 | 37 / 2 / 0 |
| drift keyframes accepted / rejected | 1 / 23 | 1 / 66 | 2 / 35 |
| tracking jumps at frames | [] | [5199, 5200] | [] |
| max correction (m / deg) | 0.112 / 1.497 | 0.101 / 4.339 | 0.320 / 1.342 |
| revisit residual raw / corrected (m), informative / all pairs | — / —, 0 / 32 | 0.009 / 0.036, 16 / 40 | 0.026 / 0.068, 38 / 38 |
| revisit within 5 cm raw / corrected | — | 0.834 / 0.759 | 0.759 / 0.411 |
| ablation rooms total area raw / corrected (m2) | 22.698 / 23.194 | 53.739 / 54.181 | 64.005 / 64.205 |
| ablation rooms raw / corrected | 3 / 3 | 5 / 5 | 4 / 5 |
| half-split rooms (1st / 2nd) | 1 / 2 | 4 / 3 | 6 / 5 |
| half-split methods (1st / 2nd) | wall_snap / wall_snap | wall_snap / wall_snap | wall_snap / wall_snap |
| half-split edges (1st / 2nd) | 44 / 23 | 93 / 89 | 79 / 71 |
| half-split area diff (m2) | 1.042 | 6.126 | 1.262 |
| half-split perimeter diff (m) | 6.798 | 7.731 | 1.868 |
