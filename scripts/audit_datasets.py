#!/usr/bin/env python3
"""Phase 0 data audit for the supplied capture ZIPs.

For every ZIP in the repo root (or those passed with --zip) writes
reports/data_audit/<capture_id>/:
  manifest.json     machine-readable audit; findings labelled
                    OBSERVED / ASSUMPTION / UNRESOLVED
  inventory.csv     every file path and size in the extracted capture
  video_frame_map.csv  video frame -> PTS -> odometry/depth frame, when the
                    dropped-frame pattern pins the alignment down exactly
  audit_report.md   human-readable summary
  previews/         RGB, depth and confidence contact sheets, trajectory plot
and docs/data_dictionary.md built from what was observed across captures.

Raw ZIPs are only read. Extraction goes to data/raw/<zip stem>/ and is
skipped when an extraction with matching file sizes already exists.
Nothing else is written under data/raw/.

Usage:
  python scripts/audit_datasets.py
  python scripts/audit_datasets.py --zip single_room.zip
"""
import argparse
import csv
import datetime
import hashlib
import json
import platform
import shutil
import subprocess
import sys
import zipfile
from collections import Counter, OrderedDict
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import PIL
from PIL import Image

from property_capture.ingestion.video import align_video_to_odometry

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / 'data' / 'raw'
DEFAULT_OUT = ROOT / 'reports' / 'data_audit'
DATA_DICTIONARY = ROOT / 'docs' / 'data_dictionary.md'

N_PREVIEW = 12
THUMB_W = 320
JUMP_THRESHOLD = 0.1  # odometry translation step (stored units) flagged as a possible tracking jump
FONT = cv2.FONT_HERSHEY_SIMPLEX


# ---- utilities ----

def sha256(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(chunk), b''):
            h.update(block)
    return h.hexdigest()


def finding(label, text):
    return {'label': label, 'text': text}


def to_json(o):
    if hasattr(o, 'item'):
        return o.item()
    if isinstance(o, (set, tuple)):
        return list(o)
    return str(o)


def git_commit():
    try:
        out = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT,
                             capture_output=True, text=True, check=True)
        return out.stdout.strip()
    except Exception:
        return 'unavailable (not a git repository or git not installed)'


def provenance():
    return {
        'command': ' '.join([sys.executable] + sys.argv),
        'python': sys.version,
        'platform': platform.platform(),
        'packages': {'numpy': np.__version__, 'pandas': pd.__version__,
                     'pillow': PIL.__version__, 'opencv': cv2.__version__},
        'script_sha256': sha256(Path(__file__)),
        'git_commit': git_commit(),
        'random_seed': 0,
    }


def read_csv_stripped(path):
    df = pd.read_csv(path, skipinitialspace=True)
    df.columns = [c.strip() for c in df.columns]
    return df


def ts_stats(ts):
    ts = np.asarray(ts, dtype=float)
    dt = np.diff(ts)
    med = float(np.median(dt)) if dt.size else float('nan')
    return {
        'first': float(ts[0]), 'last': float(ts[-1]),
        'duration_s': float(ts[-1] - ts[0]),
        'rate_hz_from_median_dt': (1.0 / med) if med > 0 else None,
        'dt_median_s': med, 'dt_min_s': float(dt.min()), 'dt_max_s': float(dt.max()),
        'dt_std_s': float(dt.std()),
        'non_increasing_steps': int((dt <= 0).sum()),
        'gaps_over_1_5x_median': int((dt > 1.5 * med).sum()),
    }


# ---- archive ----

def check_and_extract(zip_path):
    with zipfile.ZipFile(zip_path) as zf:
        tops = sorted({n.split('/')[0] for n in zf.namelist() if n.strip('/')})
        if len(tops) != 1:
            raise ValueError(f'{zip_path.name}: expected one top-level folder, found {tops}')
        cid = tops[0]
        members = [i for i in zf.infolist() if not i.is_dir()]
        print(f'  CRC-checking {len(members)} members ...')
        first_bad = zf.testzip()
        dest = RAW / zip_path.stem

        def extraction_matches():
            return all((dest / i.filename).is_file()
                       and (dest / i.filename).stat().st_size == i.file_size
                       for i in members)

        reused = extraction_matches()
        if first_bad is None and not reused:
            print(f'  extracting to {dest} ...')
            zf.extractall(dest)
        size_match = extraction_matches()
    return cid, dest / cid, {
        'zip_sha256': sha256(zip_path),
        'zip_size_bytes': zip_path.stat().st_size,
        'member_count': len(members),
        'crc_check': 'PASS' if first_bad is None else f'FAIL (first bad member: {first_bad})',
        'extraction': 'reused existing' if reused else ('extracted' if first_bad is None else 'skipped (CRC failure)'),
        'extracted_sizes_match_archive': size_match,
    }


def frame_sequence(paths):
    idx, non_numeric = [], []
    for p in paths:
        (idx.append(int(p.stem)) if p.stem.isdigit() else non_numeric.append(p.name))
    present = set(idx)
    missing = sorted(set(range(min(idx), max(idx) + 1)) - present) if idx else []
    return {
        'count': len(paths),
        'first_index': min(idx) if idx else None,
        'last_index': max(idx) if idx else None,
        'missing_count': len(missing), 'missing_indices_first_50': missing[:50],
        'duplicate_indices': sorted(k for k, v in Counter(idx).items() if v > 1),
        'non_numeric_names_first_20': non_numeric[:20],
    }, present


# ---- depth / confidence ----

def scan_depth(paths):
    rng = np.random.default_rng(0)
    shapes, dtypes, modes = Counter(), Counter(), Counter()
    corrupt, mins, maxs, zero_frac, samples = [], [], [], [], []
    n_zero = n_max16 = n_px = 0
    for p in paths:
        try:
            im = Image.open(p)
            modes[im.mode] += 1
            a = np.array(im)
        except Exception as e:
            corrupt.append({'file': p.name, 'error': str(e)})
            continue
        shapes[str(list(a.shape))] += 1
        dtypes[str(a.dtype)] += 1
        z = a == 0
        n_zero += int(z.sum())
        n_max16 += int((a == 65535).sum())
        n_px += a.size
        mins.append(int(a.min()))
        maxs.append(int(a.max()))
        zero_frac.append(float(z.mean()))
        v = a[~z].ravel()
        if v.size:
            samples.append(rng.choice(v, size=min(2000, v.size), replace=False))
    s = np.concatenate(samples) if samples else np.array([0])
    pct = {f'p{q}': float(np.percentile(s, q)) for q in (0.1, 1, 50, 99, 99.9)}
    return {
        'frames_decoded': len(paths) - len(corrupt), 'corrupt_frames': corrupt,
        'shapes': dict(shapes), 'dtypes': dict(dtypes), 'pil_modes': dict(modes),
        'min_over_all_frames': min(mins) if mins else None,
        'max_over_all_frames': max(maxs) if maxs else None,
        'nonzero_value_percentiles_sampled': pct,
        'pixels_equal_0': n_zero, 'pixels_equal_65535': n_max16, 'pixels_total': n_px,
        'frames_with_any_zero': int(sum(f > 0 for f in zero_frac)),
        'max_zero_fraction_in_a_frame': max(zero_frac) if zero_frac else None,
    }, (pct['p1'], pct['p99'])


def scan_confidence(paths):
    shapes, dtypes, hist = Counter(), Counter(), Counter()
    corrupt = []
    for p in paths:
        try:
            a = np.array(Image.open(p))
        except Exception as e:
            corrupt.append({'file': p.name, 'error': str(e)})
            continue
        shapes[str(list(a.shape))] += 1
        dtypes[str(a.dtype)] += 1
        vals, counts = np.unique(a, return_counts=True)
        hist.update({int(v): int(c) for v, c in zip(vals, counts)})
    total = sum(hist.values()) or 1
    return {
        'frames_decoded': len(paths) - len(corrupt), 'corrupt_frames': corrupt,
        'shapes': dict(shapes), 'dtypes': dict(dtypes),
        'unique_values_all_frames': sorted(hist),
        'value_fraction_all_frames': {k: round(v / total, 4) for k, v in sorted(hist.items())},
    }


# ---- video ----

def ffprobe(mp4):
    exe = shutil.which('ffprobe')
    if not exe:
        return 'ffprobe not on PATH; container metadata from OpenCV only'
    try:
        out = subprocess.run([exe, '-v', 'error', '-select_streams', 'v:0', '-show_entries',
                              'stream=codec_name,pix_fmt,width,height,r_frame_rate,avg_frame_rate,nb_frames,duration:stream_side_data',
                              '-of', 'json', str(mp4)], capture_output=True, text=True, check=True)
        return json.loads(out.stdout)
    except Exception as e:
        return f'ffprobe failed: {e}'


def scan_video(mp4, sample_idx):
    cap = cv2.VideoCapture(str(mp4))
    if not cap.isOpened():
        return {'readable': False}, {}, []
    fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    info = {
        'readable': True,
        'container_frame_count': int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),
        'container_fps': float(cap.get(cv2.CAP_PROP_FPS)),
        'width': int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        'height': int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        'fourcc': ''.join(chr((fourcc >> s) & 0xFF) for s in (0, 8, 16, 24)),
    }
    want, thumbs, pts = set(sample_idx), {}, []
    print('  decoding video for frame count and timestamps ...')
    while cap.grab():
        i = len(pts)
        pts.append(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)
        if i in want:
            ok, frame = cap.retrieve()
            if ok:
                thumbs[i] = frame
    cap.release()
    info['decoded_frame_count'] = len(pts)
    info['decoded_matches_container_count'] = len(pts) == info['container_frame_count']
    if len(pts) > 1:
        info['frame_pts_as_reported_by_opencv'] = ts_stats(pts)
    return info, thumbs, pts


# ---- odometry / imu ----

def angular_rate_from_quats(ts, q):
    # Angle between consecutive unit quaternions; independent of component
    # order and of camera-to-world vs world-to-camera direction.
    d = np.abs(np.sum(q[1:] * q[:-1], axis=1)).clip(0, 1)
    ang = 2 * np.arccos(d)
    dt = np.diff(ts)
    ok = dt > 0
    return (ts[1:] + ts[:-1])[ok] / 2, ang[ok] / dt[ok], ang


def estimate_imu_lag(t_odo, w_odo, t_imu, w_imu, step=0.005, max_lag_s=0.5):
    t0, t1 = max(t_odo[0], t_imu[0]), min(t_odo[-1], t_imu[-1])
    if t1 - t0 < 5:
        return None
    grid = np.arange(t0, t1, step)
    a = np.interp(grid, t_odo, w_odo)
    b = np.interp(grid, t_imu, w_imu)
    if a.std() == 0 or b.std() == 0:
        return None
    a, b = (a - a.mean()) / a.std(), (b - b.mean()) / b.std()
    n, best = len(a), (None, -np.inf)
    for k in range(-int(max_lag_s / step), int(max_lag_s / step) + 1):
        c = np.mean(a[:n - k] * b[k:]) if k >= 0 else np.mean(a[-k:] * b[:n + k])
        if c > best[1]:
            best = (k, c)
    lag = best[0] * step
    return {
        'method': 'cross-correlation of |gyro| vs angular rate from consecutive odometry quaternions, 5 ms grid, +/-0.5 s search',
        'imu_lag_s': round(lag, 4),
        'peak_normalized_correlation': round(float(best[1]), 3),
        'interpretation': 'positive = a rotation event appears later on the IMU timestamps than on odometry; aligned_t_imu = t_imu - imu_lag_s',
        'at_search_boundary': abs(lag) >= max_lag_s - step,
    }


def analyse_odometry(df, K, depth_idx, conf_idx):
    out = OrderedDict()
    out['rows'] = len(df)
    out['columns'] = list(df.columns)
    out['empty_columns'] = [c for c in df.columns if df[c].isna().all()]
    out['nan_counts'] = {c: int(n) for c, n in df.isna().sum().items() if n and c not in out['empty_columns']}
    ts = df['timestamp'].to_numpy(float)
    out['timestamps'] = ts_stats(ts)
    if 'frame' in df:
        fr = df['frame'].to_numpy(int)
        out['frame_column'] = {
            'first': int(fr[0]), 'last': int(fr[-1]),
            'contiguous_increasing': bool(np.all(np.diff(fr) == 1)),
            'equals_depth_indices': set(fr.tolist()) == depth_idx,
            'equals_confidence_indices': set(fr.tolist()) == conf_idx,
        }
    intr = {}
    for c, (r, col) in {'fx': (0, 0), 'fy': (1, 1), 'cx': (0, 2), 'cy': (1, 2)}.items():
        if c in df:
            v = df[c].to_numpy(float)
            intr[c] = {'min': float(v.min()), 'max': float(v.max()), 'unique': int(np.unique(v).size),
                       'camera_matrix_value': float(K[r, col]) if K is not None else None,
                       'max_abs_diff_vs_camera_matrix': float(np.abs(v - K[r, col]).max()) if K is not None else None}
    out['per_frame_intrinsics'] = intr
    xyz = df[['x', 'y', 'z']].to_numpy(float)
    q = df[['qx', 'qy', 'qz', 'qw']].to_numpy(float)
    steps = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    norms = np.linalg.norm(q, axis=1)
    _, w, ang = angular_rate_from_quats(ts, q)
    out['translation'] = {
        'range_per_axis': {a: [float(xyz[:, i].min()), float(xyz[:, i].max())] for i, a in enumerate('xyz')},
        'path_length': float(steps.sum()), 'max_step': float(steps.max()),
        'median_step': float(np.median(steps)),
        'jump_threshold': JUMP_THRESHOLD,
        'jumps_to_frame_index': [int(i + 1) for i in np.flatnonzero(steps > JUMP_THRESHOLD)],
    }
    out['quaternion'] = {
        'norm_min': float(norms.min()), 'norm_max': float(norms.max()),
        'all_unit_within_1e-3': bool(np.allclose(norms, 1, atol=1e-3)),
        'max_rotation_step_deg': float(np.degrees(ang.max())),
        'median_angular_rate_deg_s': float(np.degrees(np.median(w))),
    }
    return out, ts, xyz, q


def analyse_imu(df):
    out = OrderedDict()
    out['rows'] = len(df)
    out['columns'] = list(df.columns)
    out['nan_counts'] = {c: int(n) for c, n in df.isna().sum().items() if n}
    ts = df['timestamp'].to_numpy(float)
    out['timestamps'] = ts_stats(ts)
    acc = df[['a_x', 'a_y', 'a_z']].to_numpy(float)
    gyr = df[['alpha_x', 'alpha_y', 'alpha_z']].to_numpy(float)
    am = np.linalg.norm(acc, axis=1)
    out['accel_magnitude'] = {'median': float(np.median(am)), 'p5': float(np.percentile(am, 5)),
                              'p95': float(np.percentile(am, 95))}
    out['accel_mean_per_axis'] = dict(zip(['a_x', 'a_y', 'a_z'], acc.mean(axis=0).round(4).tolist()))
    gm = np.linalg.norm(gyr, axis=1)
    out['gyro_magnitude'] = {'median': float(np.median(gm)), 'p95': float(np.percentile(gm, 95))}
    return out, ts, gm


# ---- previews ----

def put_label(img, text):
    (tw, th), _ = cv2.getTextSize(text, FONT, 0.5, 1)
    cv2.rectangle(img, (0, 0), (tw + 10, th + 10), (0, 0, 0), -1)
    cv2.putText(img, text, (5, th + 4), FONT, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def sheet(tiles, cols=4):
    blank = np.zeros_like(tiles[0])
    rows = []
    for i in range(0, len(tiles), cols):
        row = tiles[i:i + cols]
        rows.append(np.hstack(row + [blank] * (cols - len(row))))
    return np.vstack(rows)


def thumb(img, interp):
    h, w = img.shape[:2]
    return cv2.resize(img, (THUMB_W, round(THUMB_W * h / w)), interpolation=interp)


def depth_tile(a, lo, hi):
    n = ((a.astype(float) - lo) / max(hi - lo, 1) * 255).clip(0, 255).astype(np.uint8)
    c = cv2.applyColorMap(n, cv2.COLORMAP_TURBO)
    c[a == 0] = 0
    return thumb(c, cv2.INTER_NEAREST)


CONF_LUT = np.full((256, 3), (255, 0, 255), np.uint8)
CONF_LUT[0], CONF_LUT[1], CONF_LUT[2] = (40, 40, 220), (0, 200, 230), (60, 180, 60)


def trajectory_plot(xyz, size=420, pad=34):
    panels = []
    for i, j, name in [(0, 1, 'x-y'), (0, 2, 'x-z'), (1, 2, 'y-z')]:
        img = np.full((size, size, 3), 255, np.uint8)
        p = xyz[:, [i, j]]
        lo = p.min(0)
        span = max(float((p.max(0) - lo).max()), 1e-6)
        pts = (p - lo) * ((size - 2 * pad) / span) + pad
        pts[:, 1] = size - pts[:, 1]
        pts = pts.astype(np.int32)
        cv2.polylines(img, [pts.reshape(-1, 1, 2)], False, (90, 90, 90), 1, cv2.LINE_AA)
        cv2.circle(img, tuple(int(v) for v in pts[0]), 5, (0, 160, 0), -1)
        cv2.circle(img, tuple(int(v) for v in pts[-1]), 5, (0, 0, 200), -1)
        cv2.putText(img, f'{name} raw odometry axes, span {span:.2f}', (8, 18), FONT, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.rectangle(img, (0, 0), (size - 1, size - 1), (200, 200, 200), 1)
        panels.append(img)
    out = np.hstack(panels)
    cv2.putText(out, 'green = start, red = end; units as stored (assumed metres)', (8, size - 10),
                FONT, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    return out


def write_previews(prev_dir, sample_idx, rgb_thumbs, depth_dir, conf_dir, depth_range, xyz):
    prev_dir.mkdir(parents=True, exist_ok=True)
    written = []
    if rgb_thumbs:
        tiles = [put_label(thumb(rgb_thumbs[i], cv2.INTER_AREA), f'video frame {i}')
                 for i in sample_idx if i in rgb_thumbs]
        cv2.imwrite(str(prev_dir / 'rgb_contact_sheet.png'), sheet(tiles))
        written.append('rgb_contact_sheet.png')
    lo, hi = depth_range
    tiles = []
    for i in sample_idx:
        p = depth_dir / f'{i:06d}.png'
        if p.exists():
            tiles.append(put_label(depth_tile(np.array(Image.open(p)), lo, hi), f'depth {i}'))
    if tiles:
        cv2.imwrite(str(prev_dir / 'depth_contact_sheet.png'), sheet(tiles))
        written.append('depth_contact_sheet.png')
    tiles = []
    for i in sample_idx:
        p = conf_dir / f'{i:06d}.png'
        if p.exists():
            tiles.append(put_label(thumb(CONF_LUT[np.array(Image.open(p))], cv2.INTER_NEAREST), f'conf {i}'))
    if tiles:
        cv2.imwrite(str(prev_dir / 'confidence_contact_sheet.png'), sheet(tiles))
        written.append('confidence_contact_sheet.png')
    cv2.imwrite(str(prev_dir / 'trajectory.png'), trajectory_plot(xyz))
    written.append('trajectory.png')
    return {
        'files': written,
        'sample_indices': sample_idx,
        'depth_colormap': f'TURBO, fixed range {lo:.0f}..{hi:.0f} raw units (p1..p99 of non-zero sample); black = 0',
        'confidence_colors': '0 red, 1 yellow, 2 green, any other value magenta',
    }


# ---- findings ----

def build_findings(m):
    obs, asm, unres = [], [], []
    a, d, c, v = m['archive'], m['depth'], m['confidence'], m['rgb_video']
    o, imu, s = m['odometry'], m['imu'], m['synchronization']

    obs.append(finding('OBSERVED', f"Archive CRC check: {a['crc_check']} over {a['member_count']} members; "
                                   f"extracted sizes match archive: {a['extracted_sizes_match_archive']}"))
    counts = s['stream_counts']
    obs.append(finding('OBSERVED', f"Stream counts: {counts}; all equal: {s['stream_counts_all_equal']}"))
    for name, seq in (('depth', d['sequence']), ('confidence', c['sequence'])):
        obs.append(finding('OBSERVED', f"{name} filenames {seq['first_index']}..{seq['last_index']}: "
                                       f"{seq['missing_count']} missing, {len(seq['duplicate_indices'])} duplicate"))
    obs.append(finding('OBSERVED', f"depth: {d['stats']['frames_decoded']} decoded, {len(d['stats']['corrupt_frames'])} corrupt; "
                                   f"dtypes {d['stats']['dtypes']}, shapes {d['stats']['shapes']}, "
                                   f"raw range {d['stats']['min_over_all_frames']}..{d['stats']['max_over_all_frames']}, "
                                   f"{d['stats']['pixels_equal_0']} zero pixels, {d['stats']['pixels_equal_65535']} pixels at 65535"))
    obs.append(finding('OBSERVED', f"confidence: values {c['stats']['unique_values_all_frames']} "
                                   f"with fractions {c['stats']['value_fraction_all_frames']}"))
    if v.get('readable'):
        obs.append(finding('OBSERVED', f"video {v['width']}x{v['height']} {v['fourcc']}, container {v['container_fps']:.3f} fps, "
                                       f"decoded {v['decoded_frame_count']} frames (container says {v['container_frame_count']})"))
    else:
        unres.append(finding('UNRESOLVED', 'rgb.mp4 could not be opened by OpenCV; codec support needs checking'))
    if 'frame_column' in o:
        fc = o['frame_column']
        obs.append(finding('OBSERVED', f"odometry frame column contiguous: {fc['contiguous_increasing']}; "
                                       f"equals depth indices: {fc['equals_depth_indices']}; "
                                       f"equals confidence indices: {fc['equals_confidence_indices']}"))
    ot = o['timestamps']
    obs.append(finding('OBSERVED', f"odometry rate {ot['rate_hz_from_median_dt']:.2f} Hz (median dt), "
                                   f"{o['rows'] / ot['duration_s']:.2f} Hz average; "
                                   f"{ot['gaps_over_1_5x_median']} gaps >1.5x median (largest {ot['dt_max_s']:.3f} s), "
                                   f"{ot['non_increasing_steps']} non-increasing steps"))
    obs.append(finding('OBSERVED', f"IMU rate {imu['timestamps']['rate_hz_from_median_dt']:.2f} Hz (median dt), "
                                   f"{imu['timestamps']['gaps_over_1_5x_median']} gaps >1.5x median"))
    qn = o['quaternion']
    obs.append(finding('OBSERVED', f"quaternion norms {qn['norm_min']:.6f}..{qn['norm_max']:.6f}; "
                                   f"max rotation step {qn['max_rotation_step_deg']:.2f} deg; "
                                   f"max translation step {o['translation']['max_step']:.4f}"))
    jumps = o['translation']['jumps_to_frame_index']
    if jumps:
        unres.append(finding('UNRESOLVED', f"{len(jumps)} odometry translation step(s) > {JUMP_THRESHOLD} "
                                           f"(possible tracking jump/relocalization) into frame(s) {jumps[:20]}"))
    if o['empty_columns']:
        obs.append(finding('OBSERVED', f"odometry columns present but empty in every row: {o['empty_columns']}"))
    intr = o['per_frame_intrinsics']
    if intr:
        diffs = {k: round(x['max_abs_diff_vs_camera_matrix'], 3) for k, x in intr.items()}
        uniq = {k: x['unique'] for k, x in intr.items()}
        obs.append(finding('OBSERVED', f"per-frame intrinsics unique values {uniq}; max |diff| vs camera_matrix.csv {diffs}"))

    med_a = imu['accel_magnitude']['median']
    if 0.9 <= med_a <= 1.1:
        obs.append(finding('OBSERVED', f"median |accel| = {med_a:.3f}: consistent with units of g including gravity, not m/s^2"))
    elif 9.3 <= med_a <= 10.3:
        obs.append(finding('OBSERVED', f"median |accel| = {med_a:.3f}: consistent with m/s^2 including gravity"))
    else:
        unres.append(finding('UNRESOLVED', f"median |accel| = {med_a:.3f}: matches neither g nor m/s^2 with gravity; may be gravity-removed"))

    lag = s.get('imu_vs_odometry_lag')
    if lag:
        obs.append(finding('OBSERVED', f"IMU-odometry lag estimate {lag['imu_lag_s']} s "
                                       f"(peak correlation {lag['peak_normalized_correlation']}"
                                       f"{', AT SEARCH BOUNDARY' if lag['at_search_boundary'] else ''})"))
        if lag['peak_normalized_correlation'] > 0.98:
            unres.append(finding('UNRESOLVED', 'near-perfect gyro/odometry correlation suggests odometry orientation is '
                                               'fused from this IMU, so the lag estimate does not independently validate '
                                               'camera-IMU synchronization'))
        ratio = s.get('gyro_to_odometry_rate_ratio_p90')
        if ratio:
            obs.append(finding('OBSERVED', f"p90 |gyro| / p90 odometry angular rate (rad/s) = {ratio:.3f}; "
                                           f"~1 supports rad/s, ~57 would indicate deg/s"))

    asm.append(finding('ASSUMPTION', 'quaternion component order is (qx, qy, qz, qw) as the column names suggest; '
                                     'not independently verified (rotation-angle checks above do not depend on it)'))
    n_vid, n_depth = counts.get('video_decoded'), counts.get('depth')
    al = s.get('video_to_odometry_alignment')
    if al and al['best_is_exact'] and al['best_is_unique']:
        k, missing = al['best_offset'], n_depth - n_vid
        lacks = (f"first {k} and last {missing - k}" if k and missing - k else
                 f"first {k}" if k else f"last {missing}")
        others = '; '.join(f"offset {c['odometry_offset']}: {c['gap_pattern_mismatches']} mismatches"
                           for c in al['candidates'] if c['odometry_offset'] != k)
        obs.append(finding('OBSERVED', f"{al['mapping']}: video PTS gap pattern matches odometry exactly "
                                       f"(max time residual {al['candidates'][k]['max_abs_time_residual_s'] * 1000:.2f} ms; "
                                       f"{others}). Video lacks the {lacks} odometry frame(s); see video_frame_map.csv"))
    elif n_vid == n_depth:
        asm.append(finding('ASSUMPTION', 'video frame i corresponds to depth/confidence/odometry frame i; supported only by equal counts'))
    else:
        unres.append(finding('UNRESOLVED', f"video decodes to {n_vid} frames vs {n_depth} depth/odometry frames and no "
                                           f"end offset matches the gap pattern exactly (interior drop suspected); "
                                           f"video-to-depth mapping is not established"))
    if m['camera_matrix'].get('candidate_depth_intrinsics'):
        asm.append(finding('ASSUMPTION', 'candidate depth intrinsics = RGB camera_matrix scaled by depth/RGB width; '
                                         'valid only if depth is a resampled, aligned view of the RGB image'))

    unres += [
        finding('UNRESOLVED', f"depth units: raw values {d['stats']['min_over_all_frames']}..{d['stats']['max_over_all_frames']} "
                              f"uint16; millimetres is plausible but needs a physical reference measurement"),
        finding('UNRESOLVED', 'depth invalid sentinel: confirm whether 0 (or another value) marks invalid depth'
                              + ('' if d['stats']['pixels_equal_0'] else '; no zero pixels observed')),
        finding('UNRESOLVED', 'depth-to-RGB extrinsics and true depth intrinsics are not in the archive'),
        finding('UNRESOLVED', 'pose direction (camera-to-world vs world-to-camera) and world axis convention'),
        finding('UNRESOLVED', 'IMU axis orientation relative to the camera frame'),
        finding('UNRESOLVED', 'timestamp epoch (values are not Unix time) and how video PTS maps onto it'),
        finding('UNRESOLVED', 'lens distortion: no coefficients supplied'),
        finding('UNRESOLVED', 'confidence value semantics (e.g. ARKit low/medium/high) are not documented in the archive'),
        finding('UNRESOLVED', 'capture device, app, OS and settings are not recorded in the archive'),
    ]
    return obs, asm, unres


# ---- per-capture audit ----

def audit_capture(zip_path, out_root):
    cid, cap_dir, archive = check_and_extract(zip_path)
    m = OrderedDict()
    m['capture_id'] = cid
    m['source_zip'] = zip_path.name
    m['extracted_to'] = str(cap_dir.relative_to(ROOT))
    m['audit_time_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    m['provenance'] = provenance()
    m['archive'] = archive
    out_dir = out_root / cid
    out_dir.mkdir(parents=True, exist_ok=True)
    if archive['crc_check'] != 'PASS':
        m['status'] = 'FAILED: archive integrity'
        return m, out_dir

    files = sorted(p for p in cap_dir.rglob('*') if p.is_file())
    with open(out_dir / 'inventory.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['path', 'size_bytes'])
        for p in files:
            w.writerow([p.relative_to(cap_dir).as_posix(), p.stat().st_size])
    by_dir = Counter(p.parent.relative_to(cap_dir).as_posix() for p in files)
    m['inventory'] = {'file_count': len(files), 'files_per_directory': dict(by_dir),
                      'top_level_files': {p.name: p.stat().st_size for p in files if p.parent == cap_dir},
                      'full_listing': 'inventory.csv'}

    print('  scanning depth and confidence frames ...')
    depth_files = sorted((cap_dir / 'depth').glob('*.png'))
    conf_files = sorted((cap_dir / 'confidence').glob('*.png'))
    depth_seq, depth_idx = frame_sequence(depth_files)
    conf_seq, conf_idx = frame_sequence(conf_files)
    depth_stats, depth_range = scan_depth(depth_files)
    m['depth'] = {'sequence': depth_seq, 'stats': depth_stats}
    m['confidence'] = {'sequence': conf_seq, 'stats': scan_confidence(conf_files)}

    n_frames = len(depth_files) or len(conf_files)
    sample_idx = sorted({int(i) for i in np.linspace(0, max(n_frames - 1, 0), N_PREVIEW)})
    mp4 = cap_dir / 'rgb.mp4'
    video_pts = []
    if mp4.exists():
        video, rgb_thumbs, video_pts = scan_video(mp4, sample_idx)
        video['sha256'] = sha256(mp4)
        video['ffprobe'] = ffprobe(mp4)
    else:
        video, rgb_thumbs = {'readable': False, 'note': 'rgb.mp4 not found'}, {}
    m['rgb_video'] = video

    K = None
    km = cap_dir / 'camera_matrix.csv'
    m['camera_matrix'] = {}
    if km.exists():
        K = np.loadtxt(km, delimiter=',')
        cm = {'shape': list(K.shape), 'has_header_row': False, 'matrix': K.tolist()}
        if K.shape == (3, 3):
            cm.update({'fx_equals_fy': bool(np.isclose(K[0, 0], K[1, 1])), 'skew': float(K[0, 1]),
                       'bottom_row_is_0_0_1': bool(np.allclose(K[2], [0, 0, 1]))})
            if video.get('readable'):
                cm['principal_point_offset_from_image_centre_px'] = [
                    float(K[0, 2] - video['width'] / 2), float(K[1, 2] - video['height'] / 2)]
                shape = next(iter(depth_stats['shapes']), None)
                if shape:
                    dh, dw = json.loads(shape)[:2]
                    if np.isclose(dw / video['width'], dh / video['height']):
                        sc = dw / video['width']
                        cm['candidate_depth_intrinsics'] = {
                            'label': 'ASSUMPTION', 'scale': sc,
                            'fx': K[0, 0] * sc, 'fy': K[1, 1] * sc,
                            'cx': (K[0, 2] + 0.5) * sc - 0.5, 'cy': (K[1, 2] + 0.5) * sc - 0.5}
        m['camera_matrix'] = cm

    odo_df = read_csv_stripped(cap_dir / 'odometry.csv')
    imu_df = read_csv_stripped(cap_dir / 'imu.csv')
    m['odometry'], t_odo, xyz, q = analyse_odometry(odo_df, K, depth_idx, conf_idx)
    m['imu'], t_imu, gyro_mag = analyse_imu(imu_df)

    sync = OrderedDict()
    counts = {'depth': len(depth_files), 'confidence': len(conf_files), 'odometry_rows': len(odo_df),
              'video_decoded': video.get('decoded_frame_count')}
    sync['stream_counts'] = counts
    sync['stream_counts_all_equal'] = len(set(counts.values())) == 1
    sync['odometry_vs_imu'] = {
        'imu_start_minus_odometry_start_s': float(t_imu[0] - t_odo[0]),
        'imu_end_minus_odometry_end_s': float(t_imu[-1] - t_odo[-1]),
        'overlap_s': float(min(t_imu[-1], t_odo[-1]) - max(t_imu[0], t_odo[0])),
    }
    if video.get('frame_pts_as_reported_by_opencv'):
        sync['video_duration_minus_odometry_duration_s'] = (
            video['frame_pts_as_reported_by_opencv']['duration_s'] - m['odometry']['timestamps']['duration_s'])
    align = align_video_to_odometry(video_pts, t_odo)
    sync['video_to_odometry_alignment'] = align
    if video_pts:
        off = align['best_offset'] if align and align['best_is_exact'] and align['best_is_unique'] else None
        frames = odo_df['frame'].to_numpy(int) if 'frame' in odo_df else None
        with open(out_dir / 'video_frame_map.csv', 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['video_index', 'pts_s', 'odometry_frame', 'odometry_timestamp'])
            for i, t in enumerate(video_pts):
                j = i + off if off is not None else None
                w.writerow([i, f'{t:.6f}',
                            int(frames[j]) if j is not None and frames is not None else '',
                            f'{t_odo[j]:.9f}' if j is not None else ''])
    t_w, w_odo, _ = angular_rate_from_quats(t_odo, q)
    sync['imu_vs_odometry_lag'] = estimate_imu_lag(t_w, w_odo, t_imu, gyro_mag)
    if np.percentile(w_odo, 90) > 0:
        sync['gyro_to_odometry_rate_ratio_p90'] = float(np.percentile(gyro_mag, 90) / np.percentile(w_odo, 90))
    m['synchronization'] = sync

    m['previews'] = write_previews(out_dir / 'previews', sample_idx, rgb_thumbs,
                                   cap_dir / 'depth', cap_dir / 'confidence', depth_range, xyz)
    m['observed'], m['assumptions'], m['unresolved'] = build_findings(m)
    m['status'] = 'COMPLETE'
    return m, out_dir


# ---- reports ----

def write_report(m, out_dir):
    L = [f"# Data audit: {m['capture_id']}", '',
         f"- Source ZIP: `{m['source_zip']}` (sha256 `{m['archive']['zip_sha256'][:16]}...`)",
         f"- Extracted to: `{m.get('extracted_to')}`",
         f"- Audit time (UTC): {m['audit_time_utc']}",
         f"- Status: **{m['status']}**", '']
    for title, key, box in (('Observed', 'observed', 'x'), ('Assumptions', 'assumptions', ' '),
                            ('Unresolved', 'unresolved', ' ')):
        if m.get(key):
            L.append(f'## {title}')
            L += [f"- [{box}] `{f['label']}` {f['text']}" for f in m[key]]
            L.append('')
    if m.get('previews'):
        L.append('## Previews')
        L += [f"- [previews/{f}](previews/{f})" for f in m['previews']['files']]
        L.append('')
    L.append('Full details: `manifest.json`; file listing: `inventory.csv`; '
             'video-to-odometry frame mapping: `video_frame_map.csv`.')
    (out_dir / 'audit_report.md').write_text('\n'.join(L) + '\n', encoding='utf-8')


def write_data_dictionary(manifests, path=DATA_DICTIONARY):
    done = [m for m in manifests if m['status'] == 'COMPLETE']
    L = ['# Data dictionary: supplied RGB-D captures', '',
         'Generated by `scripts/audit_datasets.py`; do not edit by hand. '
         'Labels follow CLAUDE_updated.md: OBSERVED (measured by the audit), ASSUMPTION, UNRESOLVED.', '',
         '## Captures', '',
         '| capture_id | ZIP | frames (depth/conf/odom/video) | depth shape | video | odometry Hz | IMU Hz |',
         '|---|---|---|---|---|---|---|']
    for m in done:
        c, v = m['synchronization']['stream_counts'], m['rgb_video']
        vid = f"{v['width']}x{v['height']} {v['fourcc']} {v['container_fps']:.2f} fps" if v.get('readable') else 'unreadable'
        L.append(f"| {m['capture_id']} | {m['source_zip']} | {c['depth']}/{c['confidence']}/{c['odometry_rows']}/{c['video_decoded']} "
                 f"| {', '.join(m['depth']['stats']['shapes'])} | {vid} "
                 f"| {m['odometry']['timestamps']['rate_hz_from_median_dt']:.2f} | {m['imu']['timestamps']['rate_hz_from_median_dt']:.2f} |")
    if done:
        m = done[0]
        L += ['', '## Files (common layout)', '',
              '| path | content | observed format | status of semantics |', '|---|---|---|---|',
              f"| `depth/NNNNNN.png` | depth map | {', '.join(m['depth']['stats']['dtypes'])}, PIL mode {', '.join(m['depth']['stats']['pil_modes'])} | units UNRESOLVED (mm plausible); invalid sentinel UNRESOLVED |",
              f"| `confidence/NNNNNN.png` | per-pixel depth confidence | {', '.join(m['confidence']['stats']['dtypes'])}, values {m['confidence']['stats']['unique_values_all_frames']} | semantics UNRESOLVED |",
              '| `rgb.mp4` | RGB video | see table above | frame-to-depth mapping ASSUMPTION (1:1 by index) |',
              '| `camera_matrix.csv` | 3x3 RGB intrinsics, no header row | comma-separated floats | distortion not supplied |',
              f"| `odometry.csv` | per-frame pose + intrinsics | columns: {', '.join(m['odometry']['columns'])} | quaternion order ASSUMPTION (x,y,z,w); pose direction UNRESOLVED |",
              f"| `imu.csv` | accelerometer + gyroscope | columns: {', '.join(m['imu']['columns'])} | see per-capture units evidence; axes UNRESOLVED |",
              '', '## Per-capture findings', '']
        for m in done:
            L.append(f"### {m['capture_id']}")
            L += [f"- `{f['label']}` {f['text']}" for f in m['observed'] + m['assumptions'] + m['unresolved']]
            L.append('')
    failed = [m for m in manifests if m['status'] != 'COMPLETE']
    if failed:
        L += ['', '## Captures that failed the audit', '']
        L += [f"- {m['capture_id']}: {m['status']}" for m in failed]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(L) + '\n', encoding='utf-8')


# ---- main ----

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--zip', nargs='*', help='ZIP files to audit (default: every *.zip in the repo root)')
    ap.add_argument('--output', default=str(DEFAULT_OUT), help='audit output root')
    ap.add_argument('--data-dictionary', default=str(DATA_DICTIONARY), help='where to write the data dictionary')
    args = ap.parse_args()

    zips = [Path(z) if Path(z).is_absolute() else ROOT / z for z in args.zip] if args.zip else sorted(ROOT.glob('*.zip'))
    if not zips:
        sys.exit(f'no ZIP files found in {ROOT}')
    out_root = Path(args.output)
    manifests, failed = [], False
    for zip_path in zips:
        print(f'\n===== AUDITING {zip_path.name} =====')
        try:
            m, out_dir = audit_capture(zip_path, out_root)
        except Exception as e:
            print(f'  ERROR: {e!r}')
            failed = True
            continue
        with open(out_dir / 'manifest.json', 'w', encoding='utf-8') as f:
            json.dump(m, f, indent=2, default=to_json)
        write_report(m, out_dir)
        manifests.append(m)
        failed |= m['status'] != 'COMPLETE'
        print(f"  {m['status']} -> {out_dir}")

    if manifests:
        write_data_dictionary(manifests, Path(args.data_dictionary))
        print(f'\nwrote {args.data_dictionary}')
    print('\nAUDIT FINISHED WITH FAILURES' if failed else '\nAUDIT COMPLETE')
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
