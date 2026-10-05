"""RGB video access, paired with depth/odometry frames.

The video can hold fewer frames than the depth/odometry streams (all supplied
captures lack the first one; assumptions.md B-07). The pairing is found from
the dropped-frame gap pattern, which video PTS and odometry timestamps share.
"""
import cv2
import numpy as np


def align_video_to_odometry(pts, t_odo, max_offsets=50):
    """Match the dropped-frame gap pattern in video PTS against odometry timestamps.

    Both streams are quantized to multiples of the odometry median dt; the
    offset k where video frame i lines up with odometry frame i + k is the
    one whose gap sequences agree. Only end offsets are searched, so an
    interior drop shows up as mismatches at every offset.
    """
    pts, t_odo = np.asarray(pts, float), np.asarray(t_odo, float)
    n_v, n_o = len(pts), len(t_odo)
    if n_v < 3 or n_o < n_v:
        return None
    unit = float(np.median(np.diff(t_odo)))
    qv = np.rint(np.diff(pts) / unit).astype(int)
    qo = np.rint(np.diff(t_odo) / unit).astype(int)
    cands = []
    for k in range(min(n_o - n_v, max_offsets) + 1):
        resid = (pts - pts[0]) - (t_odo[k:k + n_v] - t_odo[k])
        cands.append({'odometry_offset': k,
                      'gap_pattern_mismatches': int((qo[k:k + n_v - 1] != qv).sum()),
                      'max_abs_time_residual_s': float(np.abs(resid).max())})
    best = min(cands, key=lambda c: c['gap_pattern_mismatches'])
    return {
        'method': 'compare video PTS gaps with odometry timestamp gaps, both quantized to the odometry median dt',
        'video_frames': n_v, 'odometry_frames': n_o,
        'candidates': cands,
        'best_offset': best['odometry_offset'],
        'best_is_exact': best['gap_pattern_mismatches'] == 0,
        'best_is_unique': sum(c['gap_pattern_mismatches'] == best['gap_pattern_mismatches'] for c in cands) == 1,
        'mapping': f"video frame i = odometry/depth frame i + {best['odometry_offset']}",
    }


def scan_video(path, keep, size):
    """Decode the whole video once: per-frame PTS (s) and grayscale copies of the frames in
    `keep` (video indices), resized to `size` (w, h) with area averaging."""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise IOError(f'cannot open {path}')
    keep = set(int(k) for k in keep)
    pts, frames = [], {}
    while cap.grab():
        i = len(pts)
        pts.append(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)
        if i in keep:
            ok, bgr = cap.retrieve()
            if ok:
                frames[i] = cv2.resize(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), size, interpolation=cv2.INTER_AREA)
    cap.release()
    return np.array(pts), frames
