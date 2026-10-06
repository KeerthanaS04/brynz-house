"""Download pretrained model weights into weights/ (never committed; PDF: weights fetched by script).

    python scripts/fetch_weights.py                 # default depth model for the video tier
    python scripts/fetch_weights.py --model NAME    # one of MODELS below

Each model is pinned to a Hugging Face commit so every run uses identical weights. Licences
are recorded in docs/requirements/assumptions.md (B-24).
"""
import argparse
import sys
from pathlib import Path

from huggingface_hub import snapshot_download

REPO_ROOT = Path(__file__).resolve().parents[1]
MODELS = {
    # Depth Anything V2, metric indoor (fine-tuned on Hypersim), ViT-S, 25M parameters, ~100 MB
    'depth-anything-v2-metric-indoor-small': ('depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf',
                                              '8078d68a9c75a972131914f6afd0c1723be0da7f'),
    # Same, ViT-B, 98M parameters, ~390 MB
    'depth-anything-v2-metric-indoor-base': ('depth-anything/Depth-Anything-V2-Metric-Indoor-Base-hf',
                                             'c6d9784685727bfc6d0a7b5452ce94afaee1e7f5'),
}
DEFAULT = 'depth-anything-v2-metric-indoor-small'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--model', default=DEFAULT, choices=sorted(MODELS))
    args = ap.parse_args(argv)
    repo_id, revision = MODELS[args.model]
    dest = REPO_ROOT / 'weights' / args.model
    snapshot_download(repo_id, revision=revision, local_dir=dest)
    print(f'{repo_id}@{revision[:12]} -> {dest}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
