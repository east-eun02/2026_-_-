"""Real Gazebo camera fixtures, not injected classification results.

190454: baseline; 190716: unchanged; 190827: two orange P01 leaves.
222948: P01 misplaced by 60cm in the world; reject its distorted alignment.
"""
from pathlib import Path
from types import SimpleNamespace
from plant_observer_bringup import change_compare as cc

FIXTURES=Path(__file__).parent/'fixtures/vision'


def compare(a,b):
    return cc.compare_stop(str(FIXTURES/a),str(FIXTURES/b),SimpleNamespace(**cc.DEFAULTS))[0]


def test_same_camera_image_is_unchanged():
    assert compare('P01_baseline.png','P01_baseline.png')['status']=='unchanged'


def test_new_capture_without_change_is_unchanged():
    assert compare('P00_baseline.png','P00_unchanged.png')['status']=='unchanged'


def test_two_changed_leaf_colors_are_not_leaf_loss():
    result=compare('P01_baseline.png','P01_two_orange_leaves.png')
    assert result['status']=='changed'
    assert result['changes']=={'leaf_color':2}


def test_displaced_plant_is_not_reported_as_missing_leaves():
    assert compare('P01_baseline.png','P01_displaced.png')['status']=='align_failed'
