import ast
from pathlib import Path

import cv2
import numpy as np

from property_capture.evaluation.video_oracle import umeyama
from property_capture.modalities.video_sfm import select_frames

SRC = Path(__file__).resolve().parents[2] / 'src' / 'property_capture'


def test_video_tier_never_reads_device_data():
    """CLAUDE_updated.md rule 6: the video tier must not touch depth, poses or IMU."""
    tree = ast.parse((SRC / 'modalities' / 'video_sfm.py').read_text(encoding='utf-8'))
    imports = {n.module or '' for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)} | \
              {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    assert not any('ingestion' in m or 'registration' in m or 'evaluation' in m for m in imports), imports
    text = (SRC / 'modalities' / 'video_sfm.py').read_text(encoding='utf-8').split('"""', 2)[2]
    for word in ('odometry', 'depth', 'confidence', 'imu.csv', 'camera_matrix'):
        assert word not in text, word


def test_frame_selection_keeps_sharpest_per_window(tmp_path):
    video = tmp_path / 'v.mp4'
    w = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'mp4v'), 30, (320, 240))
    rng = np.random.default_rng(0)
    base = (rng.random((240, 320)) > 0.5).astype(np.uint8) * 255
    for i in range(90):                                     # 3 s at 30 fps
        img = np.roll(base, i, axis=1)
        if i % 10 != 4:                                     # every frame blurred except index 4 of each 10
            img = cv2.GaussianBlur(img, (15, 15), 6)
        w.write(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
    w.release()
    cfg = {'target_fps': 3, 'analysis_width': 160, 'min_sharpness_ratio': 0.5, 'max_image_size': 320}
    kept, pts, sel = select_frames(video, tmp_path / 'frames', cfg)
    assert sel['frames_decoded'] == 90 and len(pts) == 90
    assert len(kept) == 9
    assert all(k['video_index'] % 10 == 4 for k in kept)
    assert len(list((tmp_path / 'frames').glob('*.jpg'))) == sum(k['kept'] for k in kept)


def test_umeyama_recovers_similarity():
    rng = np.random.default_rng(1)
    src = rng.normal(size=(50, 3))
    th = 0.7
    R = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    dst = 2.5 * src @ R.T + [1, -2, 3]
    s, R2, t = umeyama(src, dst)
    assert np.isclose(s, 2.5) and np.allclose(R2, R) and np.allclose(t, [1, -2, 3])
