"""Direction-specific reference selection, enrollment, and temporal confirmation."""
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

from plant_observer_bringup import change_compare as cc
from plant_observer_bringup.patrol_manager import PatrolManager, TaskResult
from test_patrol_behavior import STOPS, moving_node, order


def write_baseline(root, name='2026-10-30_120000', route='demo', views=('front', 'left', 'right')):
    folder = root / name
    folder.mkdir()
    record = {'route': route, 'mode': 'baseline', 'baseline_complete': True, 'stops': []}
    for stop in STOPS:
        entry = {'id': stop['id'], 'status': 'ok', 'image': stop['id'] + '.png', 'views': {}}
        for view in views:
            image = stop['id'] + ('' if view == 'front' else '_' + view) + '.png'
            (folder/image).write_bytes(b'image')
            entry['views'][view] = {'status': 'ok', 'image': image,
                                     'pose': {'x': .3, 'y': .6, 'yaw': 0.}}
        record['stops'].append(entry)
    (folder/'result.json').write_text(json.dumps(record))
    Path(cc.baseline_file(str(root), route)).write_text(name)
    return folder


def reference(root, view):
    return cc.reference_image(str(root), 'demo', 'P00', str(root/'2099-01-01_000000'), view_id=view)


@pytest.mark.parametrize('view', ['front', 'left', 'right'])
def test_references_use_same_direction(tmp_path, view):
    baseline = write_baseline(tmp_path)
    lap, image = reference(tmp_path, view)
    assert lap == baseline.name
    assert Path(image).name == 'P00' + ('' if view == 'front' else '_' + view) + '.png'


def test_missing_side_reference_never_falls_back_to_front(tmp_path):
    write_baseline(tmp_path, views=('front',))
    assert reference(tmp_path, 'front') is not None
    assert reference(tmp_path, 'left') is None


def test_reference_missing_file_and_wrong_route_are_rejected(tmp_path):
    folder = write_baseline(tmp_path)
    (folder/'P00_left.png').unlink()
    assert reference(tmp_path, 'left') is None
    record = json.loads((folder/'result.json').read_text())
    record['route'] = 'another_map'
    (folder/'result.json').write_text(json.dumps(record))
    assert reference(tmp_path, 'front') is None


def test_previous_mode_can_bootstrap_from_complete_enrollment_without_marker(tmp_path):
    folder = write_baseline(tmp_path)
    Path(cc.baseline_file(str(tmp_path), 'demo')).unlink()
    assert reference(tmp_path, 'left')[0] == folder.name


def test_incomplete_baseline_cannot_become_previous_front_reference(tmp_path):
    previous = write_baseline(tmp_path)
    incomplete = write_baseline(tmp_path, name='2026-10-30_120100')
    record = json.loads((incomplete/'result.json').read_text())
    record['baseline_complete'] = False
    (incomplete/'result.json').write_text(json.dumps(record))
    Path(cc.baseline_file(str(tmp_path), 'demo')).write_text(previous.name)
    assert reference(tmp_path, 'front')[0] == previous.name


@pytest.mark.parametrize('pose', [None, {'x': 1., 'y': .6, 'yaw': 0.},
                                  {'x': .3, 'y': .6, 'yaw': 1.}])
def test_side_wrong_actual_pose_does_not_reach_image_comparison(tmp_path, monkeypatch, pose):
    write_baseline(tmp_path)
    compare = Mock()
    monkeypatch.setattr(cc, 'compare_stop', compare)
    result = cc.compare_with_baseline(str(tmp_path), 'demo', str(tmp_path/'2099-01-01_000000'),
                                      'P00', 'unused.png', view_id='left', current_pose=pose)
    assert result['status'] == 'view_mismatch'
    compare.assert_not_called()


def test_comparison_records_exact_side_reference(tmp_path, monkeypatch):
    baseline = write_baseline(tmp_path)
    compare = Mock(return_value=({'status': 'unchanged'}, None))
    monkeypatch.setattr(cc, 'compare_stop', compare)
    result = cc.compare_with_baseline(str(tmp_path), 'demo', str(tmp_path/'2099-01-01_000000'),
                                      'P00', 'current_left.png', view_id='left',
                                      current_pose={'x': .3, 'y': .6, 'yaw': .01})
    assert compare.call_args.args[0] == str(baseline/'P00_left.png')
    assert result['reference_image'] == 'P00_left.png'
    assert result['view_id'] == 'left'


def baseline_node(tmp_path):
    n, events, state = moving_node(tmp_path)
    n.route_path = 'test.yaml'
    n.image_rotation = 0
    n.capture_baseline = True
    return n, events, state


def test_enrollment_registers_six_images_only_after_complete_return(tmp_path):
    n, events, _ = baseline_node(tmp_path)
    result = PatrolManager.run_lap(n, 1)
    assert result['baseline_complete']
    assert len([e for e in events if e[0] == 'capture']) == 6
    assert all(set(s['views']) == {'front', 'left', 'right'} for s in result['stops'])
    baseline = Path(cc.baseline_file(str(tmp_path), 'demo')).read_text().strip()
    assert (tmp_path/baseline/'result.json').exists()
    n.compare_stop.assert_not_called()
    assert order(n) == ['P00', 'P01']


@pytest.mark.parametrize('failure', ['camera', 'navigation', 'home'])
def test_incomplete_enrollment_preserves_previous_baseline(tmp_path, failure):
    previous = write_baseline(tmp_path)
    n, events, _ = baseline_node(tmp_path)
    if failure == 'camera':
        original = n.capture.side_effect
        n.capture.side_effect = lambda path: False if 'P00_left' in path else original(path)
    else:
        original = n.go.side_effect
        n.go.side_effect = lambda target: TaskResult.FAILED if (
            (failure == 'navigation' and target.get('view_id') == 'left')
            or (failure == 'home' and target == n.home)) else original(target)
    result = PatrolManager.run_lap(n, 1)
    assert not result['baseline_complete']
    assert Path(cc.baseline_file(str(tmp_path), 'demo')).read_text() == previous.name
    assert len(list(tmp_path.glob('*/result.json'))) == 2


def test_normal_unchanged_patrol_captures_only_fronts(tmp_path):
    write_baseline(tmp_path)
    n, events, _ = baseline_node(tmp_path)
    n.capture_baseline = False
    n.compare = n.recheck = True
    n.log_result = Mock()
    n.compare_stop.return_value = {'status': 'unchanged'}
    result = PatrolManager.run_lap(n, 1)
    assert [e[1] for e in events if e[0] == 'capture'] == ['P00.png', 'P01.png']
    assert not any('recheck' in s for s in result['stops'])


def test_admin_sides_never_call_comparison(tmp_path):
    n, events, _ = moving_node(tmp_path)
    n.compare_stop.side_effect = AssertionError('Administrator photos must not be compared')
    result = n.recheck_stop(str(tmp_path), STOPS[0])
    assert len(result['attempts']) == 2
    assert result['status'] == 'ok' and result['purpose'] == 'admin_review'
    assert set(result['views']) == {'left', 'right'}
    n.compare_stop.assert_not_called()


def test_both_admin_photos_missing_are_reported_without_verdict(tmp_path):
    n, _, _ = moving_node(tmp_path)
    n.capture.return_value = False
    n.capture.side_effect = None
    result = n.recheck_stop(str(tmp_path), STOPS[0])
    assert result['status'] == 'no_image'
    assert 'compare' not in result and 'verdict' not in result
    n.compare_stop.assert_not_called()


def change(lap, kind='leaf_color'):
    return {'status': 'changed', 'changes': {kind: 1}, 'reference_lap': lap}


def test_changed_p00_is_rechecked_before_p01_and_prioritized_next_lap(tmp_path):
    baseline = write_baseline(tmp_path)
    n, events, _ = baseline_node(tmp_path)
    n.capture_baseline = False
    n.compare = n.recheck = True
    n.log_result = Mock()
    n.compare_stop.side_effect = [change(baseline.name), {'status': 'unchanged'}]
    result = PatrolManager.run_lap(n, 1)
    assert events.index(('capture', 'P00_left.png')) < events.index(('go', 'P01'))
    assert events.index(('capture', 'P00_right.png')) < events.index(('go', 'P01'))
    assert result['stops'][0]['final'] == {'status': 'changed', 'changes': {'leaf_color': 1}, 'source': 'front'}
    assert n.compare_stop.call_count == 2
    assert n.compare_stop.call_args_list[0].kwargs['reference_lap'] == baseline.name
    assert order(n) == ['P00', 'P01']


@pytest.mark.parametrize('target,tree', [(STOPS[0], 'capture.xml'),
                                       ({**STOPS[0], 'view_id':'front'}, 'capture.xml'),
                                       ({**STOPS[0], 'view_id':'right'}, 'side_capture.xml'),
                                       ({**STOPS[0], 'view_id':'left'}, 'side_capture.xml'),
                                       ({'x':0.,'y':0.,'yaw':0.}, '')])
def test_capture_uses_precise_navigation_but_home_uses_normal(tmp_path, target, tree):
    n, _, _ = baseline_node(tmp_path)
    n.nav_retries = 0
    n.capture_behavior_tree = 'capture.xml'
    n.side_capture_behavior_tree = 'side_capture.xml'
    n.capture_nav_timeout = 60.
    n.nav_timeout = 120.
    n.make_pose = lambda *a: a
    n.goToPose = Mock(return_value=True)
    n.isTaskComplete = lambda: True
    n.getResult = lambda: TaskResult.SUCCEEDED
    assert PatrolManager.go(n, target) == TaskResult.SUCCEEDED
    assert n.goToPose.call_args.kwargs['behavior_tree'] == tree


def test_precise_navigation_timeout_cancels_goal(tmp_path, monkeypatch):
    import plant_observer_bringup.patrol_manager as pm
    n, _, _ = baseline_node(tmp_path)
    n.nav_retries = 1
    n.capture_behavior_tree = 'capture.xml'
    n.capture_nav_timeout = 1.
    n.make_pose = lambda *a: a
    n.goToPose = Mock(return_value=True)
    n.isTaskComplete = lambda: False
    n.cancel_navigation = Mock(return_value=True)
    monkeypatch.setattr(pm.time, 'monotonic', Mock(side_effect=[0., 2.]))
    assert PatrolManager.go(n, STOPS[0]) == TaskResult.FAILED
    n.cancel_navigation.assert_called_once()


def test_all_navigation_trees_select_a_configured_goal_checker():
    import xml.etree.ElementTree as ET
    import yaml
    config=Path(__file__).resolve().parents[2]/'plant_observer_gazebo'/'config'
    params=yaml.safe_load((config/'nav2_params.yaml').read_text())['controller_server']['ros__parameters']
    for filename,expected in [('capture_bt.xml','capture_goal_checker'),
                              ('side_capture_bt.xml','side_capture_goal_checker'),
                              ('navigate_bt.xml','general_goal_checker'),
                              ('navigate_through_poses_bt.xml','general_goal_checker')]:
        nodes=ET.parse(config/filename).findall('.//FollowPath')
        assert nodes
        assert all(n.attrib['goal_checker_id'] == expected for n in nodes)
        assert expected in params['goal_checker_plugins']
    assert params['general_goal_checker']['xy_goal_tolerance'] == .15
    assert params['capture_goal_checker']['xy_goal_tolerance'] == .06
    assert params['capture_goal_checker']['yaw_goal_tolerance'] == .12
    assert params['side_capture_goal_checker']['xy_goal_tolerance'] == .10
    assert params['side_capture_goal_checker']['yaw_goal_tolerance'] == .20
    assert params['FollowPath']['use_collision_detection']


@pytest.mark.parametrize('during', ['recheck','home'])
def test_interrupted_navigation_keeps_photos_and_comparisons(tmp_path, during):
    write_baseline(tmp_path)
    n, _, _ = baseline_node(tmp_path)
    n.capture_baseline=False
    n.compare=n.recheck=True
    n.log_result=Mock()
    n.compare_stop.return_value=change('baseline') if during=='recheck' else {'status':'unchanged'}
    original=n.go.side_effect
    def go(target):
        if (during=='recheck' and target.get('view_id') in ('left','right')) or target==n.home:
            raise RuntimeError('lost navigation response')
        return original(target)
    n.go.side_effect=go
    with pytest.raises(RuntimeError):
        PatrolManager.run_lap(n,1)
    result=json.loads(sorted(tmp_path.glob('*/result.json'))[-1].read_text())
    assert result['home']=='pending'
    assert len(result['stops'])==(1 if during=='recheck' else 2)
    assert result['stops'][0]['compare']['status']==('changed' if during=='recheck' else 'unchanged')
    assert result['stops'][0]['image']=='P00.png'


def test_cancel_has_bounded_wait_for_missing_response(tmp_path, monkeypatch):
    import plant_observer_bringup.patrol_manager as pm
    n, _, _=baseline_node(tmp_path)
    future=Mock()
    future.done.return_value=False
    n.goal_handle=Mock()
    n.goal_handle.cancel_goal_async.return_value=future
    spin=Mock()
    monkeypatch.setattr(pm.rclpy,'spin_until_future_complete',spin)
    assert not PatrolManager.cancel_navigation(n)
    assert spin.call_args.kwargs['timeout_sec']==2.


