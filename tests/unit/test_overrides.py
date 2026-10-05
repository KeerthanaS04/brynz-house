import pytest

from property_capture.pipeline import apply_overrides


def test_override_parses_yaml_values():
    cfg = {'floorplan': {'polygon_method': 'wall_snap', 'walls': {'max_gap_m': 0.25}}, 'seed': 0}
    apply_overrides(cfg, ['floorplan.polygon_method=occupancy', 'floorplan.walls.max_gap_m=0.3', 'seed=7'])
    assert cfg == {'floorplan': {'polygon_method': 'occupancy', 'walls': {'max_gap_m': 0.3}}, 'seed': 7}


@pytest.mark.parametrize('bad', ['floorplan.nope=1', 'nosection.key=1', 'missing_equals'])
def test_override_rejects_unknown_keys(bad):
    with pytest.raises((KeyError, ValueError)):
        apply_overrides({'floorplan': {'polygon_method': 'wall_snap'}}, [bad])
