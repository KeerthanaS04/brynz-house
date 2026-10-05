# Data audit: c7d28f72c6

- Source ZIP: `single_scan_with_ceiling.zip` (sha256 `4bfbeb11ee21b114...`)
- Extracted to: `data\raw\single_scan_with_ceiling\c7d28f72c6`
- Audit time (UTC): 2026-10-05T06:19:54.988400+00:00
- Status: **COMPLETE**

## Observed
- [x] `OBSERVED` Archive CRC check: PASS over 19494 members; extracted sizes match archive: True
- [x] `OBSERVED` Stream counts: {'depth': 9745, 'confidence': 9745, 'odometry_rows': 9745, 'video_decoded': 9744}; all equal: False
- [x] `OBSERVED` depth filenames 0..9744: 0 missing, 0 duplicate
- [x] `OBSERVED` confidence filenames 0..9744: 0 missing, 0 duplicate
- [x] `OBSERVED` depth: 9745 decoded, 0 corrupt; dtypes {'uint16': 9745}, shapes {'[192, 256]': 9745}, raw range 48..12125, 0 zero pixels, 0 pixels at 65535
- [x] `OBSERVED` confidence: values [0, 1, 2] with fractions {0: 0.0425, 1: 0.0516, 2: 0.906}
- [x] `OBSERVED` video 1920x1440 hevc, container 45.340 fps, decoded 9744 frames (container says 9745)
- [x] `OBSERVED` odometry frame column contiguous: True; equals depth indices: True; equals confidence indices: True
- [x] `OBSERVED` odometry rate 59.99 Hz (median dt), 45.34 Hz average; 3149 gaps >1.5x median (largest 0.050 s), 0 non-increasing steps
- [x] `OBSERVED` IMU rate 99.30 Hz (median dt), 0 gaps >1.5x median
- [x] `OBSERVED` quaternion norms 1.000000..1.000000; max rotation step 4.14 deg; max translation step 0.0280
- [x] `OBSERVED` odometry columns present but empty in every row: ['distortion_center_x', 'distortion_center_y']
- [x] `OBSERVED` per-frame intrinsics unique values {'fx': 65, 'fy': 65, 'cx': 3852, 'cy': 4398}; max |diff| vs camera_matrix.csv {'fx': 19.884, 'fy': 19.884, 'cx': 0.542, 'cy': 0.292}
- [x] `OBSERVED` median |accel| = 1.003: consistent with units of g including gravity, not m/s^2
- [x] `OBSERVED` IMU-odometry lag estimate 0.0 s (peak correlation 1.0)
- [x] `OBSERVED` p90 |gyro| / p90 odometry angular rate (rad/s) = 1.011; ~1 supports rad/s, ~57 would indicate deg/s
- [x] `OBSERVED` video frame i = odometry/depth frame i + 1: video PTS gap pattern matches odometry exactly (max time residual 29.37 ms; offset 0: 6199 mismatches). Video lacks the first 1 odometry frame(s); see video_frame_map.csv

## Assumptions
- [ ] `ASSUMPTION` quaternion component order is (qx, qy, qz, qw) as the column names suggest; not independently verified (rotation-angle checks above do not depend on it)
- [ ] `ASSUMPTION` candidate depth intrinsics = RGB camera_matrix scaled by depth/RGB width; valid only if depth is a resampled, aligned view of the RGB image

## Unresolved
- [ ] `UNRESOLVED` near-perfect gyro/odometry correlation suggests odometry orientation is fused from this IMU, so the lag estimate does not independently validate camera-IMU synchronization
- [ ] `UNRESOLVED` depth units: raw values 48..12125 uint16; millimetres is plausible but needs a physical reference measurement
- [ ] `UNRESOLVED` depth invalid sentinel: confirm whether 0 (or another value) marks invalid depth; no zero pixels observed
- [ ] `UNRESOLVED` depth-to-RGB extrinsics and true depth intrinsics are not in the archive
- [ ] `UNRESOLVED` pose direction (camera-to-world vs world-to-camera) and world axis convention
- [ ] `UNRESOLVED` IMU axis orientation relative to the camera frame
- [ ] `UNRESOLVED` timestamp epoch (values are not Unix time) and how video PTS maps onto it
- [ ] `UNRESOLVED` lens distortion: no coefficients supplied
- [ ] `UNRESOLVED` confidence value semantics (e.g. ARKit low/medium/high) are not documented in the archive
- [ ] `UNRESOLVED` capture device, app, OS and settings are not recorded in the archive

## Previews
- [previews/rgb_contact_sheet.png](previews/rgb_contact_sheet.png)
- [previews/depth_contact_sheet.png](previews/depth_contact_sheet.png)
- [previews/confidence_contact_sheet.png](previews/confidence_contact_sheet.png)
- [previews/trajectory.png](previews/trajectory.png)

Full details: `manifest.json`; file listing: `inventory.csv`; video-to-odometry frame mapping: `video_frame_map.csv`.
