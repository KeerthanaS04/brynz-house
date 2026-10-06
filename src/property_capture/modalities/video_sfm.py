"""Video tier: camera motion and sparse structure from an RGB video alone.

Inputs: one video file. No depth, no device poses, no intrinsics, no IMU (PDF video tier:
"a handheld walkthrough clip from any iPhone 15 or newer"). This module must never read
anything else from a capture (CLAUDE_updated.md rule 6; enforced by tests/unit/test_video_sfm.py).

1. Frame selection: the sharpest frame (variance of the Laplacian) in each window of
   1 / target_fps seconds; frames much blurrier than the median are dropped.
2. Structure from motion with COLMAP (pycolmap, CPU): SIFT features, sequential matching,
   incremental mapping with one shared camera whose focal length is estimated.
3. Monocular reconstruction has no metric scale: positions are in arbitrary SfM units and
   are reported as such (scale_status 'unknown').
"""
import json
from pathlib import Path

import cv2
import numpy as np
import pycolmap


def select_frames(video_path, out_dir, cfg):
    """Write the sharpest frame of each window as JPEG; returns kept frames and all frame PTS."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise IOError(f'cannot open video {video_path}')
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    window = max(1, int(round(fps / cfg['target_fps'])))
    pts, kept, best = [], [], None
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        pts.append(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)
        h, w = frame.shape[:2]
        small = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
                           (cfg['analysis_width'], int(round(h * cfg['analysis_width'] / w))), interpolation=cv2.INTER_AREA)
        sharp = float(cv2.Laplacian(small, cv2.CV_64F).var())
        if best is None or sharp > best[0]:
            best = (sharp, i, frame)
        if (i + 1) % window == 0:
            kept.append(_write(out_dir, best, cfg, pts))
            best = None
        i += 1
    if best is not None:
        kept.append(_write(out_dir, best, cfg, pts))
    cap.release()
    median = float(np.median([k['sharpness'] for k in kept])) if kept else 0.0
    for k in kept:
        k['kept'] = k['sharpness'] >= cfg['min_sharpness_ratio'] * median
        if not k['kept']:
            (out_dir / k['file']).unlink()
    return kept, pts, {'fps': fps, 'frames_decoded': i, 'window_frames': window, 'median_sharpness': median}


def _write(out_dir, best, cfg, pts):
    sharp, idx, frame = best
    h, w = frame.shape[:2]
    s = min(1.0, cfg['max_image_size'] / max(h, w))
    img = cv2.resize(frame, (int(round(w * s)), int(round(h * s))), interpolation=cv2.INTER_AREA) if s < 1 else frame
    name = f'{idx:06d}.jpg'
    cv2.imwrite(str(out_dir / name), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
    return {'video_index': idx, 'pts_s': pts[idx], 'sharpness': sharp, 'file': name}


def reconstruct(image_dir, work_dir, cfg):
    """COLMAP sparse reconstruction; returns the model with the most registered images and stats."""
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    db = work_dir / 'database.db'
    reader = pycolmap.ImageReaderOptions()
    reader.camera_model = cfg['camera_model']
    ext = pycolmap.FeatureExtractionOptions()
    ext.max_image_size = cfg['max_image_size']
    ext.use_gpu = False
    ext.sift.max_num_features = cfg['max_features']
    pycolmap.extract_features(db, image_dir, camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader,
                              extraction_options=ext, device=pycolmap.Device.cpu)
    if cfg['matcher'] == 'exhaustive':
        # every pair of frames: lets revisits of the same walls join pieces that sequential
        # matching leaves apart (turning on the spot gives no parallax to grow a model across)
        pycolmap.match_exhaustive(db, device=pycolmap.Device.cpu)
    elif cfg['matcher'] == 'sequential':
        pairing = pycolmap.SequentialPairingOptions()
        pairing.overlap = cfg['sequential_overlap']
        pairing.quadratic_overlap = True
        pycolmap.match_sequential(db, pairing_options=pairing, device=pycolmap.Device.cpu)
    else:
        raise ValueError(f"unknown video.matcher {cfg['matcher']!r}")
    opts = pycolmap.IncrementalPipelineOptions()
    opts.random_seed = cfg['random_seed']
    # COLMAP's defaults suit photo collections with wide baselines; a walkthrough moves a little
    # between frames, so starting and growing a model needs smaller angles and fewer matches.
    for key, value in (cfg.get('pipeline') or {}).items():
        setattr(opts, key, value)
    for key, value in (cfg.get('mapper') or {}).items():
        setattr(opts.mapper, key, value)
    sparse = work_dir / 'sparse'
    sparse.mkdir(exist_ok=True)
    models = pycolmap.incremental_mapping(db, image_dir, sparse, opts)
    if not models:
        return None, {'models': 0}, []
    rec = max(models.values(), key=lambda r: r.num_reg_images())
    all_models = sorted(models.values(), key=lambda m: min(int(Path(im.name).stem) for im in m.images.values()
                                                           if im.has_pose))
    pieces = []
    for m in models.values():
        idx = sorted(int(Path(im.name).stem) for im in m.images.values() if im.has_pose)
        pieces.append({'frames': len(idx), 'first_video_index': idx[0], 'last_video_index': idx[-1]})
    pieces.sort(key=lambda p: p['first_video_index'])
    in_any = sum(p['frames'] for p in pieces)
    return rec, {'models': len(models), 'pieces': pieces, 'frames_in_any_model': in_any,
                 'model_sizes': sorted((m.num_reg_images() for m in models.values()), reverse=True),
                 'registered_images': rec.num_reg_images(), 'points3D': rec.num_points3D(),
                 'mean_reprojection_error_px': rec.compute_mean_reprojection_error(),
                 'mean_track_length': rec.compute_mean_track_length()}, all_models


def _cam_from_world(image):
    cfw = image.cam_from_world
    return cfw() if callable(cfw) else cfw


def trajectory(rec):
    """Per registered image: video index, camera centre and camera-to-world rotation (SfM frame)."""
    rows = []
    for img in rec.images.values():
        if not img.has_pose:
            continue
        R_cw = np.asarray(_cam_from_world(img).rotation.matrix())
        rows.append({'video_index': int(Path(img.name).stem), 'centre': np.asarray(img.projection_center()).tolist(),
                     'R_c2w': R_cw.T.tolist()})
    return sorted(rows, key=lambda r: r['video_index'])


def run_video(video_path, out_dir, cfg):
    """Video tier entry point. Only `video_path` is read."""
    out_dir = Path(out_dir)
    if out_dir.exists() and any(out_dir.iterdir()):
        raise FileExistsError(f'{out_dir} exists and is not empty; runs never overwrite')
    frames_dir = out_dir / 'frames'
    kept, pts, sel = select_frames(video_path, frames_dir, cfg)
    rec, stats, all_models = reconstruct(frames_dir, out_dir / 'colmap', cfg)
    traj = trajectory(rec) if rec is not None else []
    # every piece, in video order; each has its own frame and scale
    pieces = [trajectory(m) for m in all_models]
    used = [k for k in kept if k['kept']]
    camera = None
    if rec is not None:
        cam = next(iter(rec.cameras.values()))
        camera = {'model': str(cam.model), 'width': cam.width, 'height': cam.height,
                  'params': np.asarray(cam.params).tolist()}
        rec.export_PLY(str(out_dir / 'sparse_points.ply'))
    summary = {
        'input_tier': 'video', 'inputs_read': [str(video_path)], 'config': cfg,
        'scale_status': 'unknown: monocular video has no metric scale; positions are in arbitrary SfM units',
        'frame_selection': {**sel, 'windows': len(kept), 'used_for_sfm': len(used)},
        'sfm': stats, 'camera': camera,
        'registered_fraction': (stats.get('registered_images', 0) / len(used)) if used else 0.0,
        'fraction_in_any_model': (stats.get('frames_in_any_model', 0) / len(used)) if used else 0.0,
    }
    (out_dir / 'video_sfm.json').write_text(json.dumps(summary, indent=2, default=str), encoding='utf-8')
    (out_dir / 'trajectory.json').write_text(json.dumps({'frames': traj, 'pieces': pieces, 'all_frame_pts_s': pts,
                                                         'selected': kept}, default=str), encoding='utf-8')
    return summary
