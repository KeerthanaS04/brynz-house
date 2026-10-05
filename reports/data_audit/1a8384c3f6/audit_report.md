# Data audit: 1a8384c3f6

- Source ZIP: `single_scan_floor_only.zip` (sha256 `f822287268297d2a...`)
- Extracted to: `data\raw\single_scan_floor_only\1a8384c3f6`
- Audit time (UTC): 2026-10-05T06:18:45.983988+00:00
- Status: **COMPLETE**

## Observed
- [x] `OBSERVED` Archive CRC check: PASS over 10506 members; extracted sizes match archive: True
- [x] `OBSERVED` Stream counts: {'depth': 5251, 'confidence': 5251, 'odometry_rows': 5251, 'video_decoded': 5250}; all equal: False
- [x] `OBSERVED` depth filenames 0..5250: 0 missing, 0 duplicate
- [x] `OBSERVED` confidence filenames 0..5250: 0 missing, 0 duplicate
- [x] `OBSERVED` depth: 5251 decoded, 0 corrupt; dtypes {'uint16': 5251}, shapes {'[192, 256]': 5251}, raw range 74..11406, 0 zero pixels, 0 pixels at 65535
- [x] `OBSERVED` confidence: values [0, 1, 2] with fractions {0: 0.0462, 1: 0.0562, 2: 0.8977}
- [x] `OBSERVED` video 1920x1440 hevc, container 45.747 fps, decoded 5250 frames (container says 5251)
- [x] `OBSERVED` odometry frame column contiguous: True; equals depth indices: True; equals confidence indices: True
- [x] `OBSERVED` odometry rate 59.99 Hz (median dt), 45.75 Hz average; 1635 gaps >1.5x median (largest 0.050 s), 0 non-increasing steps
- [x] `OBSERVED` IMU rate 99.31 Hz (median dt), 0 gaps >1.5x median
- [x] `OBSERVED` quaternion norms 1.000000..1.000000; max rotation step 4.18 deg; max translation step 0.3142
- [x] `OBSERVED` odometry columns present but empty in every row: ['distortion_center_x', 'distortion_center_y']
- [x] `OBSERVED` per-frame intrinsics unique values {'fx': 52, 'fy': 52, 'cx': 2199, 'cy': 2384}; max |diff| vs camera_matrix.csv {'fx': 17.172, 'fy': 17.172, 'cx': 0.359, 'cy': 0.19}
- [x] `OBSERVED` median |accel| = 1.001: consistent with units of g including gravity, not m/s^2
- [x] `OBSERVED` IMU-odometry lag estimate 0.005 s (peak correlation 0.999)
- [x] `OBSERVED` p90 |gyro| / p90 odometry angular rate (rad/s) = 1.002; ~1 supports rad/s, ~57 would indicate deg/s
- [x] `OBSERVED` video frame i = odometry/depth frame i + 1: video PTS gap pattern matches odometry exactly (max time residual 16.55 ms; offset 0: 3242 mismatches). Video lacks the first 1 odometry frame(s); see video_frame_map.csv

## Assumptions
- [ ] `ASSUMPTION` quaternion component order is (qx, qy, qz, qw) as the column names suggest; not independently verified (rotation-angle checks above do not depend on it)
- [ ] `ASSUMPTION` candidate depth intrinsics = RGB camera_matrix scaled by depth/RGB width; valid only if depth is a resampled, aligned view of the RGB image

## Unresolved
- [ ] `UNRESOLVED` 2 odometry translation step(s) > 0.1 (possible tracking jump/relocalization) into frame(s) [5199, 5200]
- [ ] `UNRESOLVED` near-perfect gyro/odometry correlation suggests odometry orientation is fused from this IMU, so the lag estimate does not independently validate camera-IMU synchronization
- [ ] `UNRESOLVED` depth units: raw values 74..11406 uint16; millimetres is plausible but needs a physical reference measurement
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
