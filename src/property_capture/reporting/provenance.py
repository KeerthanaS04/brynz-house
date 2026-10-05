"""Run provenance: command, code version, package versions, input hashes."""
import hashlib
import json
import platform
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import PIL


def sha256_file(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(chunk), b''):
            h.update(block)
    return h.hexdigest()


def sha256_tree(folder):
    """Hash of every file's relative path and content, in sorted order."""
    h = hashlib.sha256()
    for p in sorted(Path(folder).rglob('*')):
        if p.is_file():
            h.update(p.relative_to(folder).as_posix().encode())
            h.update(sha256_file(p).encode())
    return h.hexdigest()


def config_hash(cfg):
    return hashlib.sha256(json.dumps(cfg, sort_keys=True).encode()).hexdigest()


def git_info(root):
    def run(*args):
        return subprocess.run(['git', *args], cwd=root, capture_output=True, text=True, check=True).stdout.strip()
    try:
        return {'commit': run('rev-parse', 'HEAD'), 'dirty': bool(run('status', '--porcelain'))}
    except Exception:
        return {'commit': 'unavailable', 'dirty': None}


def collect(repo_root, capture_root, cfg):
    capture_root = Path(capture_root)
    return {
        'command': ' '.join([sys.executable] + sys.argv),
        'python': sys.version,
        'platform': platform.platform(),
        'packages': {'numpy': np.__version__, 'pandas': pd.__version__,
                     'pillow': PIL.__version__, 'opencv': cv2.__version__},
        'git': git_info(repo_root),
        'config_sha256': config_hash(cfg),
        'random_seed': cfg['seed'],
        'inputs': {
            'capture_root': str(capture_root),
            **{f'{name}_sha256': sha256_file(capture_root / name)
               for name in ('odometry.csv', 'imu.csv', 'camera_matrix.csv', 'rgb.mp4')
               if (capture_root / name).exists()},
            'depth_tree_sha256': sha256_tree(capture_root / 'depth'),
            'confidence_tree_sha256': sha256_tree(capture_root / 'confidence'),
        },
    }
