import zipfile

import pytest

from property_capture.cli import main
from property_capture.tiers import UnknownInput, detect_tier, find_video


def _touch(p):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(b'x')
    return p


def test_lidar_zip_and_folder(tmp_path):
    z = tmp_path / 'scan.zip'
    with zipfile.ZipFile(z, 'w') as zf:
        for name in ('cap/odometry.csv', 'cap/rgb.mp4', 'cap/depth/000000.png'):
            zf.writestr(name, 'x')
    assert detect_tier(z) == 'lidar'
    for name in ('odometry.csv', 'rgb.mp4', 'depth/000000.png'):
        _touch(tmp_path / 'dir' / 'cap' / name)
    assert detect_tier(tmp_path / 'dir') == 'lidar'          # LiDAR capture wins although it holds a video


def test_video_file_and_folder(tmp_path):
    v = _touch(tmp_path / 'walk' / 'IMG_0001.MOV')
    assert detect_tier(v) == 'video'
    assert detect_tier(tmp_path / 'walk') == 'video'
    assert find_video(tmp_path / 'walk') == v


def test_photo_folders(tmp_path):
    for room in ('room-1', 'room-2'):
        for i in range(3):
            _touch(tmp_path / 'photos' / room / f'IMG_{i}.HEIC')
    assert detect_tier(tmp_path / 'photos') == 'photo'


def test_ambiguous_input_is_refused(tmp_path):
    _touch(tmp_path / 'mixed' / 'a.mp4')
    _touch(tmp_path / 'mixed' / 'b.jpg')
    with pytest.raises(UnknownInput):
        detect_tier(tmp_path / 'mixed')
    _touch(tmp_path / 'empty' / 'notes.txt')
    with pytest.raises(UnknownInput):
        detect_tier(tmp_path / 'empty')


def test_run_refuses_photo_and_unknown_inputs_without_output(tmp_path, capsys):
    _touch(tmp_path / 'photos' / 'room-1' / 'a.jpg')
    out = tmp_path / 'out'
    assert main(['run', '--input', str(tmp_path / 'photos'), '--output', str(out)]) == 2
    assert 'photo tier is not implemented' in capsys.readouterr().err and not out.exists()
    assert main(['run', '--input', str(tmp_path / 'missing'), '--output', str(out)]) == 2
