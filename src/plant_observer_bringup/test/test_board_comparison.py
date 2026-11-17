"""Real camera board fixtures, with deterministic exterior-only perturbations.

Registration images identify board geometry, never the temporal reference.
No ROS nodes, robot movement, production captures, or upload destinations used.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import yaml

from plant_observer_bringup import change_compare as cc
from plant_observer_bringup.board_mask import BoardNotFound, load_board_profile, polygon_mask

ROUTE = Path(__file__).parents[2] / 'plant_observer_gazebo/config/patrol_demo_real3.yaml'


def fixture(sid):
    profile = load_board_profile(str(ROUTE), sid)
    image = cv2.imread(profile['image'])
    return profile, image, polygon_mask(image.shape, profile['polygon'])


def background(image, mask, style):
    result = image.copy()
    if style == 'noise':
        exterior = np.random.default_rng(42).integers(0, 256, image.shape, dtype=np.uint8)
    elif style == 'checker':
        grid = np.indices(image.shape[:2]).sum(axis=0) // 32 % 2
        exterior = np.where(grid[..., None], [30, 160, 30], [20, 20, 200]).astype(np.uint8)
    else:
        colors = {'black': (0, 0, 0), 'white': (255, 255, 255),
                  'green': (30, 150, 30), 'red': (20, 20, 200)}
        exterior = np.full_like(image, colors[style])
    result[mask == 0] = exterior[mask == 0]
    assert np.array_equal(image[mask > 0], result[mask > 0])
    return result


def compare(tmp_path, before, after, profile):
    paths = [tmp_path / 'before.png', tmp_path / 'after.png']
    for p, im in zip(paths, (before, after)):
        assert cv2.imwrite(str(p), im)
    return cc.compare_stop(*(str(p) for p in paths), SimpleNamespace(**cc.DEFAULTS, board=profile))[0]


@pytest.mark.parametrize('sid', ['P00', 'P01'])
@pytest.mark.parametrize('style', ['black', 'white', 'green', 'red', 'noise', 'checker'])
@pytest.mark.parametrize('shifted', [False, True])
def test_exterior_only_changes_are_unchanged(tmp_path, sid, style, shifted):
    profile, before, mask = fixture(sid)
    current = before.copy()
    if shifted:
        # A known translation independent of the board detector's output.
        transform = np.float32([[1, 0, 18], [0, 1, -9]])
        current = cv2.warpAffine(current, transform, before.shape[1::-1], borderValue=(225, 225, 225))
        mask = cv2.warpAffine(mask, transform, before.shape[1::-1], flags=cv2.INTER_NEAREST)
    result = compare(tmp_path, before, background(current, mask, style), profile)
    assert result['status'] == 'unchanged', result  # a failure is NOT a passing negative control
    assert result['changes'] == {}
    assert result['comparison_scope'] == 'board'
    assert set(result['board']) == {'reference', 'current'}


def changed_leaf(image, sid, kind):
    result = image.copy()
    center = (183, 260) if sid == 'P00' else (141, 260)
    color = (25, 125, 215) if kind == 'leaf_color' else (225, 225, 225)
    cv2.ellipse(result, center, (15, 20), 0, 0, 360, color, -1)
    return result


@pytest.mark.parametrize('sid', ['P00', 'P01'])
@pytest.mark.parametrize('kind', ['leaf_color', 'leaf_loss'])
@pytest.mark.parametrize('exterior_changed', [False, True])
def test_interior_leaf_changes_remain_detectable(tmp_path, sid, kind, exterior_changed):
    profile, before, mask = fixture(sid)
    current = changed_leaf(before, sid, kind)
    assert np.array_equal(before[mask == 0], current[mask == 0])
    if exterior_changed:
        current = background(current, mask, 'checker')
    result = compare(tmp_path, before, current, profile)
    assert result['status'] == 'changed', result
    assert result['changes'] == {kind: 1}


@pytest.mark.parametrize('failure', ['missing_anchor', 'blank', 'cropped', 'missing_polygon'])
def test_unlocatable_board_never_falls_back_to_full_frame(tmp_path, failure):
    profile, before, _ = fixture('P00')
    current = before.copy()
    if failure == 'missing_anchor':
        profile['image'] = str(tmp_path / 'missing.png')
    elif failure == 'missing_polygon':
        del profile['polygon']
    elif failure == 'blank':
        current[:] = 225
    else:
        current = current[:450]
    result = compare(tmp_path, before, current, profile)
    assert result['status'] == 'board_not_found', result
    assert 'changes' not in result


def test_no_plant_never_returns_full_frame_roi(tmp_path):
    assert not cc.plant_roi(np.full((100, 100), cc.BG, np.uint8)).any()
    empty = np.full((480, 640, 3), 225, np.uint8)
    assert compare(tmp_path, empty, empty, None)['status'] == 'no_plant'


def test_color_normalization_ignores_exterior_pixels():
    _, before, mask = fixture('P00')
    _, _, expected = cc.normalize_colors(before, mask=mask)
    for style in ('black', 'white', 'red', 'green', 'checker', 'noise'):
        _, _, actual = cc.normalize_colors(background(before, mask, style), mask=mask)
        np.testing.assert_array_equal(expected[0], actual[0])
        assert expected[1] == actual[1]


@pytest.mark.parametrize('bad_config', [None, {'comparison_scope': 'boards'},
                                      {'comparison_scope': 'board', 'comparison_boards': {}}])
def test_invalid_required_board_config_fails_closed(tmp_path, bad_config):
    path = tmp_path / 'route.yaml'
    path.write_text(yaml.safe_dump(bad_config))
    with pytest.raises(BoardNotFound):
        load_board_profile(str(path), 'P00')


def test_registered_picture_does_not_replace_previous_patrol(tmp_path):
    profile, anchor, _ = fixture('P00')
    changed = changed_leaf(anchor, 'P00', 'leaf_color')
    laps = ['2026-10-31_010000', '2026-10-31_020000', '2026-10-31_030000']
    for name, image in zip(laps, (anchor, changed, changed)):
        folder = tmp_path / name
        folder.mkdir()
        cv2.imwrite(str(folder / 'P00.png'), image)
        (folder / 'result.json').write_text(json.dumps({
            'route': str(ROUTE), 'finished': '2026-10-31T00:00:00',
            'stops': [{'id': 'P00', 'status': 'ok', 'image': 'P00.png'}]}))
    folder = tmp_path / laps[-1]
    result = cc.compare_with_baseline(str(tmp_path), str(ROUTE), str(folder), 'P00', str(folder / 'P00.png'))
    assert result['status'] == 'unchanged', result
    assert result['reference_lap'] == laps[-2]
    assert result['comparison_scope'] == 'board'
    assert (folder / result['image']).is_file()


def test_board_failure_is_visible_and_gets_retry_priority():
    from plant_observer_bringup import patrol_manager as pm, review_server as review
    status = {'status': 'board_not_found'}
    stop = {'status': 'ok', 'compare': status, 'final': status}
    assert '보드' in pm.describe_change(status)
    assert pm.visit_priority(stop, None)[0] == 3
    assert review.classify(stop) == ('check', '비교 불가(보드 경계 확인 필요)')


def test_simulation_retains_unregistered_plant_mode():
    assert load_board_profile(str(ROUTE.with_name('patrol_demo.yaml')), 'P00') is None
