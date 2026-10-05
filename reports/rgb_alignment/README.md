# Depth-to-RGB alignment (branch `feat/rgb-alignment`)

Until now nothing read the RGB video. Anything detected in RGB (openings,
damage) will be placed in 3D through the depth, so two assumptions had to be
checked first (assumptions.md):

- **B-07:** video frame i pairs with depth/odometry frame i + 1, from the audit's gap-pattern match.
- **B-15:** depth intrinsics are the RGB intrinsics scaled to 256×192, i.e. depth is a resampled, aligned view of the RGB image.

## Method (`src/property_capture/calibration/rgb_alignment.py`)

1. **Pairing.** The pipeline decodes the video once and pairs frames by the dropped-frame gap pattern shared by video PTS and odometry timestamps (`ingestion/video.py`). This is the same function the audit uses, moved into the package.
2. **Spatial check.** 24 low-motion frames, spread over the capture. Depth edges (strongest 4% of log-depth gradients, at least ~5% relative depth jump) are compared with RGB edge strength at 2× depth resolution. The shift of the RGB edge image that maximizes edge strength at the depth edges is found on a ±3 depth px grid, refined by a parabola through neighbouring scores. Resolution is about ±0.25 depth px: edges are ~2 px wide, so the score peak is flat-topped.
3. **Scale and rotation check.** The same shift per image quadrant (all frames pooled). A scale or rotation error between depth and RGB makes the quadrants disagree.
4. **Temporal check.** 24 frames with moderate rotation (60th–90th percentile): the paired video frame should align better than the frames just before and after. Chance level is 1 in 3.
5. **Status:**
   - `aligned`: |offset| ≤ 0.5 depth px and quadrants agree within 0.5 px
   - `constant_offset`: quadrants agree but a larger offset; correctable, offset recorded
   - `inconsistent`: quadrants disagree, or offset > 2 px
6. **Correction.** The measured offset is stored as `depth_to_rgb_offset_px`, and `depth_pixel_to_rgb()` applies it when mapping depth pixels to RGB pixels.

Outputs: `rgb_alignment.png` (depth edges in red over RGB), `intermediates/rgb_alignment.json`, and `metrics.json` `rgb_alignment`.

## Results (`rgb2_*` runs)

| | single_room | floor_only | with_ceiling |
|---|---|---|---|
| frame pairing (gap pattern) | i + 1, exact | i + 1, exact | i + 1, exact |
| paired frame aligns best (chance 33%) | 63% | 67% | 77% |
| offset, horizontal / vertical (depth px) | −0.39 / 0.00 | −0.48 / 0.00 | −0.50 / 0.00 |
| quadrant deviation (depth px) | 0.27 | 0.09 | 0.06 |
| status (current rule) | aligned | aligned | constant_offset (−0.501) |

`comparison.md` was generated before the status field existed and shows the
earlier pass/fail flag (`aligned: False` for with_ceiling at −0.501 px).

## Findings

- **B-07 confirmed by a second, independent method.** The paired video frame lines up with the depth about twice as often as chance on every capture.
- **B-15 holds up to a constant horizontal offset.** All three captures show RGB content about 0.4–0.5 depth px (3–4 RGB px, ~0.12°, ~4 mm at 2 m) to the left of the depth pixel that sees it, with no vertical offset. Quadrants agree within 0.06–0.27 px, so there is no scale or rotation error. The offset has the same sign and size on every capture, so it looks like a property of the device's depth-to-RGB registration rather than noise.
- **Geometry is unaffected.** A constant offset acts like a uniform 0.12° turn of depth relative to the camera, which multi-view depth checks cannot see (convention residuals stay at 5–7 mm). It matters only when projecting between RGB and depth, and `depth_pixel_to_rgb()` corrects it there.
- **The overlay confirms it visually:** depth edges (red) sit on monitor, desk and wall edges in `rgb_alignment.png`.

## Limitations

1. The offset is measured per capture, not calibrated per device. A different phone or app may differ, so the check runs on every capture.
2. Rolling shutter and motion blur are not modelled; the spatial check uses low-motion frames to avoid them.
3. Runtime: each run decodes the whole video once (+20–60 s on these captures).

## Reproduce

```powershell
python -m property_capture run --input data/raw/single_room/c00a170fe1 --output outputs/rgb2_single_room
python -m property_capture run --input data/raw/single_scan_floor_only/1a8384c3f6 --output outputs/rgb2_floor_only
python -m property_capture run --input data/raw/single_scan_with_ceiling/c7d28f72c6 --output outputs/rgb2_with_ceiling
python scripts/compare_runs.py outputs/rgb2_single_room outputs/rgb2_floor_only outputs/rgb2_with_ceiling --out reports/rgb_alignment/comparison.md
```
