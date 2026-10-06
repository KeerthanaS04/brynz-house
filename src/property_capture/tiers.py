"""Which input tier a capture belongs to, from its layout alone (nothing is parsed).

  lidar: a ZIP or folder with odometry.csv (directly or in its single subfolder), as exported by
         the LiDAR capture app (assumptions.md B-20)
  video: a video file (.mp4, .mov), or a folder holding exactly one and no odometry.csv
  photo: a folder of still images, or of per-room subfolders of still images (PDF: 2-8 per room)
"""
import zipfile
from pathlib import Path

VIDEO_EXT = {'.mp4', '.mov', '.m4v'}
IMAGE_EXT = {'.jpg', '.jpeg', '.png', '.heic', '.heif'}
TIERS = ('lidar', 'video', 'photo')


class UnknownInput(ValueError):
    pass


def _names_tier(names):
    """Tier from a list of relative file paths (ZIP members or folder contents)."""
    paths = [Path(n) for n in names if not n.endswith('/')]
    if any(p.name == 'odometry.csv' for p in paths):
        return 'lidar'
    videos = [p for p in paths if p.suffix.lower() in VIDEO_EXT]
    images = [p for p in paths if p.suffix.lower() in IMAGE_EXT]
    if len(videos) == 1 and not images:
        return 'video'
    if images and not videos:
        return 'photo'
    raise UnknownInput(f'cannot tell the tier: {len(videos)} video file(s), {len(images)} image(s), '
                       'no odometry.csv')


def detect_tier(path):
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)
    if path.is_file() and path.suffix.lower() in VIDEO_EXT:
        return 'video'
    if path.is_file() and path.suffix.lower() == '.zip':
        with zipfile.ZipFile(path) as zf:
            return _names_tier(zf.namelist())
    if path.is_dir():
        return _names_tier([p.relative_to(path).as_posix() for p in path.rglob('*') if p.is_file()])
    raise UnknownInput(f'{path}: not a ZIP, folder or video file')


def find_video(path):
    """The one video file in a folder (or its subfolders), e.g. a LiDAR capture's rgb.mp4."""
    path = Path(path)
    if path.is_file():
        return path
    videos = [p for p in path.rglob('*') if p.is_file() and p.suffix.lower() in VIDEO_EXT]
    if len(videos) != 1:
        raise UnknownInput(f'{path}: expected one video file, found {len(videos)}')
    return videos[0]
