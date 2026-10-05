# Data audit: c00a170fe1

- Source ZIP: `single_room.zip` (sha256 `0805f742d378e4bd...`)
- Extracted to: `data\raw\single_room\c00a170fe1`
- Audit time (UTC): 2026-10-05T06:18:21.109791+00:00
- Status: **COMPLETE**

## Observed
- [x] `OBSERVED` Archive CRC check: PASS over 3434 members; extracted sizes match archive: True
- [x] `OBSERVED` Stream counts: {'depth': 1715, 'confidence': 1715, 'odometry_rows': 1715, 'video_decoded': 1714}; all equal: False
- [x] `OBSERVED` depth filenames 0..1714: 0 missing, 0 duplicate
- [x] `OBSERVED` confidence filenames 0..1714: 0 missing, 0 duplicate
- [x] `OBSERVED` depth: 1715 decoded, 0 corrupt; dtypes {'uint16': 1715}, shapes {'[192, 256]': 1715}, raw range 55..5852, 0 zero pixels, 0 pixels at 65535
- [x] `OBSERVED` confidence: values [0, 1, 2] with fractions {0: 0.0179, 1: 0.044, 2: 0.9381}
- [x] `OBSERVED` video 1920x1440 hevc, container 46.123 fps, decoded 1714 frames (container says 1715)
- [x] `OBSERVED` odometry frame column contiguous: True; equals depth indices: True; equals confidence indices: True
- [x] `OBSERVED` odometry rate 59.99 Hz (median dt), 46.14 Hz average; 515 gaps >1.5x median (largest 0.050 s), 0 non-increasing steps
- [x] `OBSERVED` IMU rate 99.32 Hz (median dt), 0 gaps >1.5x median
- [x] `OBSERVED` quaternion norms 1.000000..1.000000; max rotation step 4.91 deg; max translation step 0.0247
- [x] `OBSERVED` odometry columns present but empty in every row: ['distortion_center_x', 'distortion_center_y']
- [x] `OBSERVED` per-frame intrinsics unique values {'fx': 54, 'fy': 54, 'cx': 1211, 'cy': 1310}; max |diff| vs camera_matrix.csv {'fx': 18.076, 'fy': 18.076, 'cx': 0.331, 'cy': 0.236}
- [x] `OBSERVED` median |accel| = 1.003: consistent with units of g including gravity, not m/s^2
- [x] `OBSERVED` IMU-odometry lag estimate 0.005 s (peak correlation 1.0)
- [x] `OBSERVED` p90 |gyro| / p90 odometry angular rate (rad/s) = 0.998; ~1 supports rad/s, ~57 would indicate deg/s
- [x] `OBSERVED` video frame i = odometry/depth frame i + 1: video PTS gap pattern matches odometry exactly (max time residual 5.70 ms; offset 0: 1021 mismatches). Video lacks the first 1 odometry frame(s); see video_frame_map.csv

## Assumptions
- [ ] `ASSUMPTION` quaternion component order is (qx, qy, qz, qw) as the column names suggest; not independently verified (rotation-angle checks above do not depend on it)
- [ ] `ASSUMPTION` candidate depth intrinsics = RGB camera_matrix scaled by depth/RGB width; valid only if depth is a resampled, aligned view of the RGB image

## Unresolved
- [ ] `UNRESOLVED` near-perfect gyro/odometry correlation suggests odometry orientation is fused from this IMU, so the lag estimate does not independently validate camera-IMU synchronization
- [ ] `UNRESOLVED` depth units: raw values 55..5852 uint16; millimetres is plausible but needs a physical reference measurement
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
