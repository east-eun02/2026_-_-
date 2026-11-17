"""Rolling references and the real patrol state machine, with no robot commands.

The image integration compares real Gazebo front-camera fixtures. Side photos
are administrator evidence and must never be passed to the image comparator.
"""
import json
from pathlib import Path
import shutil
from types import MethodType
from unittest.mock import Mock

import pytest

from plant_observer_bringup import change_compare as cc
from plant_observer_bringup import capture_uploader as cu
from plant_observer_bringup import review_server as rs
from plant_observer_bringup import patrol_manager as pm
from test_view_patrol import baseline_node, write_baseline
from test_patrol_behavior import order


def previous_node(root):
    n, events, state = baseline_node(root)
    n.capture_baseline = False
    n.compare_to = 'previous'
    n.compare = n.recheck = True
    n.log_result = Mock()
    n.compare_stop = Mock(side_effect=lambda folder, sid, image, **kw: {
        'status': 'unchanged', 'changes': {}, 'reference_lap': kw['reference_lap'],
        'reference_image': image, 'view_id': kw['view_id']})
    return n, events, state


def completed(root, name, views=('front', 'left', 'right'), **updates):
    folder = write_baseline(root, name=name, views=views)
    record = json.loads((folder/'result.json').read_text())
    record.update(mode='patrol', finished='2026-10-31T01:00:00', home='ok')
    record.update(updates)
    (folder/'result.json').write_text(json.dumps(record))
    return folder


def latest_result(root):
    return sorted(root.glob('*/result.json'))[-1]


def test_first_patrol_collects_all_views_without_comparison(tmp_path):
    n, events, _ = previous_node(tmp_path)
    result = pm.PatrolManager.run_lap(n, 1)
    assert result['reference_lap'] is None
    assert result['compare_to'] == 'previous' and result['home'] == 'ok'
    captures = [name for event, name in events if event == 'capture']
    assert captures[0] == 'P00.png' and captures[3] == 'P01.png'
    assert set(captures) == {'P00.png', 'P00_left.png', 'P00_right.png',
                             'P01.png', 'P01_left.png', 'P01_right.png'}
    assert len(captures) == 6
    assert captures == ['P00.png', 'P00_right.png', 'P00_left.png',
                        'P01.png', 'P01_right.png', 'P01_left.png']
    assert all(s['final']['status'] == 'no_reference' and 'recheck' not in s for s in result['stops'])
    n.compare_stop.assert_not_called()


def test_previous_ignores_registered_baseline_and_never_backfills_missing_direction(tmp_path):
    old = write_baseline(tmp_path)
    new = completed(tmp_path, '2026-10-30_120100', views=('front',))
    Path(cc.baseline_file(str(tmp_path), 'demo')).write_text(old.name)
    current = str(tmp_path/'2099-01-01_000000')
    assert cc.reference_image(str(tmp_path), 'demo', 'P00', current)[0] == new.name
    assert cc.reference_image(str(tmp_path), 'demo', 'P00', current, view_id='left') is None
    assert cc.reference_image(str(tmp_path), 'demo', 'P00', current, use_baseline=True)[0] == old.name


def test_standalone_comparison_uses_same_completed_previous_lap(tmp_path):
    previous = completed(tmp_path, '2026-10-30_120000')
    completed(tmp_path, '2026-10-30_120100', finished=None)
    current = completed(tmp_path, '2026-10-30_120200')
    assert cc.pick_before(str(tmp_path), str(current)) == str(previous)


@pytest.mark.parametrize('condition', ['unfinished', 'wrong_route', 'corrupt', 'incomplete_enrollment'])
def test_previous_skips_ineligible_laps(tmp_path, condition):
    old = completed(tmp_path, '2026-10-30_120000')
    new = completed(tmp_path, '2026-10-30_120100')
    path = new/'result.json'
    data = json.loads(path.read_text())
    if condition == 'unfinished':
        data.pop('finished')
    elif condition == 'wrong_route':
        data['route'] = 'other'
    elif condition == 'incomplete_enrollment':
        data.update(mode='baseline', baseline_complete=False)
    path.write_text('{' if condition == 'corrupt' else json.dumps(data))
    assert cc.previous_lap(str(tmp_path), 'demo', str(tmp_path/'2099-01-01_000000')) == old.name


def test_prior_failed_stop_is_not_replaced_by_an_older_image(tmp_path):
    completed(tmp_path, '2026-10-30_120000')
    new = completed(tmp_path, '2026-10-30_120100')
    record = json.loads((new/'result.json').read_text())
    record['stops'][0] = {'id': 'P00', 'status': 'nav_failed'}
    (new/'result.json').write_text(json.dumps(record))
    assert cc.reference_image(str(tmp_path), 'demo', 'P00', str(tmp_path/'2099-01-01_000000')) is None


def test_failed_recapture_keeps_valid_initial_view_as_reference(tmp_path):
    folder = completed(tmp_path, '2026-10-30_120000')
    record = json.loads((folder/'result.json').read_text())
    record['stops'][0]['recheck'] = {'views': {'left': {'status': 'no_image'}}}
    (folder/'result.json').write_text(json.dumps(record))
    reference = cc.reference_image(str(tmp_path), 'demo', 'P00', str(tmp_path/'2099-01-01_000000'), view_id='left')
    assert Path(reference[1]).name == 'P00_left.png'


def test_successful_recapture_is_latest_reference_with_its_own_pose(tmp_path, monkeypatch):
    folder = completed(tmp_path, '2026-10-30_120000')
    record = json.loads((folder/'result.json').read_text())
    pose = {'x': .42, 'y': .61, 'yaw': .04}
    record['stops'][0]['recheck'] = {'views': {'left': {
        'status': 'ok', 'image': 'P00_left_recheck.png', 'pose': pose}}}
    (folder/'P00_left_recheck.png').write_bytes(b'newest')
    (folder/'result.json').write_text(json.dumps(record))
    compare = Mock(return_value=({'status': 'unchanged'}, None))
    monkeypatch.setattr(cc, 'compare_stop', compare)
    result = cc.compare_with_baseline(str(tmp_path), 'demo', str(tmp_path/'2099-01-01_000000'),
                                     'P00', 'unused.png', view_id='left', current_pose=pose)
    assert result['reference_image'] == 'P00_left_recheck.png'
    assert result['pose_distance_m'] == 0.
    assert compare.call_args.args[0] == str(folder/'P00_left_recheck.png')


def test_later_unchanged_patrol_only_captures_fronts(tmp_path):
    n, events, _ = previous_node(tmp_path)
    pm.PatrolManager.run_lap(n, 1)
    previous = latest_result(tmp_path).parent.name
    events.clear()
    result = pm.PatrolManager.run_lap(n, 2)
    assert [name for event, name in events if event == 'capture'] == ['P00.png', 'P01.png']
    assert result['reference_lap'] == previous
    assert all(set(stop['views']) == {'front'} and 'recheck' not in stop for stop in result['stops'])
    assert n.compare_stop.call_count == 2
    assert all(call.kwargs['view_id'] == 'front' for call in n.compare_stop.call_args_list)


@pytest.mark.parametrize('mode', ['previous', 'baseline'])
def test_changed_front_gets_admin_sides_before_next_stop(tmp_path, mode):
    n, events, _ = previous_node(tmp_path)
    n.compare_to = mode
    pm.PatrolManager.run_lap(n, 1)
    previous = latest_result(tmp_path).parent.name
    events.clear()
    n.compare_stop.side_effect = lambda folder, sid, image, **kw: {
        'status': 'changed' if sid == 'P00' else 'unchanged',
        'changes': {'leaf_color': 1} if sid == 'P00' else {},
        'reference_lap': kw['reference_lap'], 'view_id': kw['view_id']}
    result = pm.PatrolManager.run_lap(n, 2)
    p00 = result['stops'][0]
    assert result['reference_lap'] == previous
    assert p00['final'] == {'status': 'changed', 'changes': {'leaf_color': 1}, 'source': 'front'}
    next_go = events.index(('go', 'P01'))
    for name in ('P00_left.png', 'P00_right.png'):
        assert events.index(('capture', name)) < next_go
    assert events.index(('capture', 'P00.png')) < events.index(('capture', 'P00_right.png'))
    assert events.index(('capture', 'P00_right.png')) < events.index(('capture', 'P00_left.png'))
    assert [name for event, name in events if event == 'capture'].count('P00.png') == 1
    assert len([event for event in events if event[0] == 'capture']) == 4
    assert set(p00['recheck']['views']) == {'left', 'right'}
    assert p00['recheck']['purpose'] == 'admin_review'
    assert not any('compare' in view for view in p00['recheck']['views'].values())
    assert n.compare_stop.call_count == 2
    assert all(c.kwargs['view_id'] == 'front' and c.kwargs['reference_lap'] == previous
               for c in n.compare_stop.call_args_list)
    assert order(n) == ['P00', 'P01']


@pytest.mark.parametrize('status', ['align_failed', 'no_plant', 'error'])
def test_front_comparison_failure_is_not_treated_as_change(tmp_path, status):
    n, events, _ = previous_node(tmp_path)
    pm.PatrolManager.run_lap(n, 1)
    events.clear()
    n.compare_stop.side_effect = lambda folder, sid, image, **kw: {
        'status': status if sid == 'P01' else 'unchanged', 'reference_lap': kw['reference_lap']}
    result = pm.PatrolManager.run_lap(n, 2)
    assert result['stops'][1]['final']['status'] == status
    assert 'recheck' not in result['stops'][1]
    assert len([event for event in events if event[0] == 'capture']) == 2
    assert order(n) == ['P01', 'P00']


def test_failed_admin_photo_preserves_front_change_and_priority(tmp_path):
    n, _, _ = previous_node(tmp_path)
    pm.PatrolManager.run_lap(n, 1)
    original = n.capture.side_effect
    n.capture.side_effect = lambda path: False if '_left' in path else original(path)
    n.compare_stop.side_effect = lambda folder, sid, image, **kw: {
        'status': 'changed' if sid == 'P01' else 'unchanged',
        'changes': {'leaf_color': 1} if sid == 'P01' else {},
        'reference_lap': kw['reference_lap']}
    result = pm.PatrolManager.run_lap(n, 2)
    p01 = result['stops'][1]
    assert p01['final'] == {'status': 'changed', 'changes': {'leaf_color': 1}, 'source': 'front'}
    assert p01['recheck']['status'] == 'incomplete'
    assert order(n) == ['P01', 'P00']
    row = next(i for i in rs.Store(str(tmp_path)).observations()
               if i['lap'] == latest_result(tmp_path).parent.name and i['stop'] == 'P01')
    assert row['side_capture_status'] == 'incomplete'
    assert row['side_images'][0]['purpose'] == 'admin_review'
    assert row['verdict'] is None and row['recheck_compare_image'] is None


def test_previous_without_sides_still_compares_front_normally(tmp_path):
    completed(tmp_path, '2026-10-30_120000', views=('front',))
    n, events, _ = previous_node(tmp_path)
    result = pm.PatrolManager.run_lap(n, 1)
    assert all(stop['final']['status'] == 'unchanged' for stop in result['stops'])
    assert len([event for event in events if event[0] == 'capture']) == 2


def test_new_plant_gets_initial_overview_independently(tmp_path):
    folder = completed(tmp_path, '2026-10-30_120000')
    path = folder/'result.json'
    data = json.loads(path.read_text())
    data['stops'][1] = {'id': 'P01', 'status': 'nav_failed'}
    path.write_text(json.dumps(data))
    n, _, _ = previous_node(tmp_path)
    result = pm.PatrolManager.run_lap(n, 1)
    by_id = {stop['id']: stop for stop in result['stops']}
    assert set(by_id['P00']['views']) == {'front'}
    assert set(by_id['P01']['views']) == {'front', 'left', 'right'}
    assert by_id['P01']['initial_observation'] and 'overview' in by_id['P01']


def test_interrupt_during_admin_capture_preserves_change_and_saved_side(tmp_path):
    n, _, _ = previous_node(tmp_path)
    pm.PatrolManager.run_lap(n, 1)
    previous = latest_result(tmp_path).parent.name
    n.compare_stop.side_effect = lambda folder, sid, image, **kw: {
        'status': 'changed', 'changes': {'leaf_color': 1}, 'reference_lap': kw['reference_lap']}
    original = n.go.side_effect
    def interrupt(target):
        if target.get('view_id') == 'left':
            raise RuntimeError('interrupted navigation')
        return original(target)
    n.go.side_effect = interrupt
    with pytest.raises(RuntimeError):
        pm.PatrolManager.run_lap(n, 2)
    data = json.loads(latest_result(tmp_path).read_text())
    assert not data.get('finished') and data['stops'][0]['final']['status'] == 'changed'
    assert data['stops'][0]['views']['right']['image'] == 'P00_right.png'
    assert cc.previous_lap(str(tmp_path), 'demo', str(tmp_path/'2099-01-01_000000')) == previous


def test_four_laps_real_front_comparison_admin_photos_and_priority(tmp_path):
    n, events, _ = previous_node(tmp_path)
    fixtures = Path(__file__).parent/'fixtures/vision'
    photos = {'P00': 'P01_baseline.png', 'P01': 'P01_baseline.png'}
    def capture(path):
        sid = Path(path).name.split('_')[0].split('.')[0]
        events.append(('capture', Path(path).name))
        # An unsuitable side photo must not affect the front's detection.
        photo = 'P01_displaced.png' if '_left' in path or '_right' in path else photos[sid]
        shutil.copyfile(fixtures/photo, path)
        return True
    n.capture.side_effect = capture
    n.compare_stop = Mock(wraps=MethodType(pm.PatrolManager.compare_stop, n))
    records = []
    capture_counts = []
    for lap in range(1, 5):
        events.clear()
        if lap == 2:
            photos['P01'] = 'P01_two_orange_leaves.png'
        if lap == 4:
            photos['P00'] = 'P01_two_orange_leaves.png'
        record = pm.PatrolManager.run_lap(n, lap)
        records.append((latest_result(tmp_path).parent.name, record))
        capture_counts.append(len([event for event in events if event[0] == 'capture']))
        assert record['home'] == 'ok'
        if lap > 1:
            assert record['reference_lap'] == records[-2][0]
            assert all(s['compare']['reference_lap'] == records[-2][0] for s in record['stops'])
        for stop in record['stops']:
            assert all('compare' not in view for view_id, view in stop['views'].items() if view_id != 'front')
            assert 'verdict' not in stop['final']
    assert capture_counts == [6, 4, 2, 4]
    assert n.compare_stop.call_count == 6
    assert all(call.kwargs['view_id'] == 'front' for call in n.compare_stop.call_args_list)
    assert records[1][1]['stops'][1]['final']['changes'] == {'leaf_color': 2}
    assert [s['id'] for s in records[2][1]['visit_order']] == ['P01', 'P00']
    assert all(s['final']['status'] == 'unchanged' and 'recheck' not in s for s in records[2][1]['stops'])
    assert records[2][1]['stops'][0]['compare']['reference_image'] == 'P01.png'
    assert [s['id'] for s in records[3][1]['visit_order']] == ['P01', 'P00']
    assert records[3][1]['stops'][1]['final']['changes'] == {'leaf_color': 2}
    assert order(n) == ['P00', 'P01']
    row = next(i for i in rs.Store(str(tmp_path)).observations()
               if i['lap'] == records[1][0] and i['stop'] == 'P01')
    assert row['verdict'] is None and row['recheck_compare_image'] is None
    assert len(row['side_images']) == 2 and all(v['purpose'] == 'admin_review' for v in row['side_images'])
    assert row['compare_to'] == 'previous'
    index = cu.build_index(str(tmp_path))
    indexed = next(i for i in index['laps'] if i['lap'] == records[1][0])
    assert indexed['capture_policy'] == 'front_compare_sides_for_admin'
    assert indexed['reference_lap'] == records[0][0]
    assert indexed['stops'][1]['compare']['reference_image'] == records[0][0] + '/P01.png'
    assert indexed['stops'][1]['recheck']['views']['left']['image'] == records[1][0] + '/P01_left.png'
    initial = next(i for i in index['laps'] if i['lap'] == records[0][0])
    assert initial['stops'][0]['overview']['views']['left']['image'] == records[0][0] + '/P00_left.png'
