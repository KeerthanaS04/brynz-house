"""Video tier on a video with nothing to reconstruct: it must stop with the reason, not a plan."""
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from property_capture.video_pipeline import run_video_tier

ROOT = Path(__file__).resolve().parents[2]


def test_featureless_video_stops_with_reason(tmp_path):
    video = tmp_path / 'blank.mp4'
    w = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'mp4v'), 30, (320, 240))
    for i in range(60):                                   # 2 s of a plain grey wall
        w.write(np.full((240, 320, 3), 128, np.uint8))
    w.release()
    cfg = yaml.safe_load((ROOT / 'configs' / 'default.yaml').read_text(encoding='utf-8'))
    with pytest.raises(RuntimeError, match='reconstructed nothing'):
        run_video_tier(video, tmp_path / 'run', cfg, ROOT / 'configs' / 'evaluation' / 'gates.yaml')
    assert not (tmp_path / 'run' / 'property.json').exists()
