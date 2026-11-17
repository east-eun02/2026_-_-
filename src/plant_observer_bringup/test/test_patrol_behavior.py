"""No hardware: exercise policy and the real patrol state machine with fake Nav2."""
import json
import math
from pathlib import Path
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import pytest

from plant_observer_bringup.patrol_manager import PatrolManager, TaskResult

STOPS = [dict(id='P00', x=.3, y=-.6, yaw=0.), dict(id='P01', x=.3, y=.6, yaw=0.)]


def node(tmp_path):
    n = SimpleNamespace(capture_dir=str(tmp_path), route='demo', stops=STOPS,
                        home=dict(x=0., y=0., yaw=0.), prioritize=True,
                        plant_distance=.5, recheck_angle=math.radians(35), recheck_clearance=.15,
                        recheck_min_distance=.10, recheck_min_view_angle=math.radians(10),
                        map_frame='map', odom_frame='odom', capture_baseline=False, compare_to='baseline',
                        get_logger=lambda: Mock())
    for name in ('plan_order', 'history_laps', 'recheck_poses', 'capture_view', 'run_baseline_lap',
                 'recheck_stop', 'recheck_motion'):
        if hasattr(PatrolManager, name):
            setattr(n, name, MethodType(getattr(PatrolManager, name), n))
    return n


def entry(sid, status='unchanged'):
    return dict(id=sid, status='ok', compare=dict(status=status, changes={'leaf_color': 1} if status == 'changed' else {}))


def lap(tmp_path, name, stops, route='demo'):
    folder = tmp_path / name
    folder.mkdir()
    (folder / 'result.json').write_text(json.dumps(dict(route=route, stops=stops)))
    return folder


def order(n):
    return [s['id'] for s, _ in n.plan_order(str(Path(n.capture_dir) / '2099-01-01_000000'))]


def test_latest_change_survives_unchanged_lap(tmp_path):
    lap(tmp_path, '2026-10-30_120000', [entry('P00'), entry('P01', 'changed')])
    lap(tmp_path, '2026-10-30_120100', [entry('P01'), entry('P00')])
    assert order(node(tmp_path)) == ['P01', 'P00']


def test_same_rank_latest_observation_first(tmp_path):
    lap(tmp_path, '2026-10-30_120000', [entry('P00', 'changed'), entry('P01', 'changed')])
    assert order(node(tmp_path)) == ['P01', 'P00']


def test_no_motion_cannot_count_as_recheck(tmp_path):
    n = node(tmp_path)
    n.go = Mock(return_value=TaskResult.SUCCEEDED)
    n.robot_pose = Mock(return_value=(.3, -.6, 0.))
    n.fresh_pose = Mock(return_value=(.3, -.6, 0.))
    n.capture = Mock(return_value=True)
    n.compare_stop = Mock(return_value=dict(status='changed', changes={'leaf_color': 1}))
    result = n.recheck_stop(str(tmp_path), STOPS[0])
    assert result['status'] == 'insufficient_motion'
    assert n.capture.call_count == 0
    assert n.go.call_count == 2


def test_new_change_replaces_old_priority(tmp_path):
    lap(tmp_path, '2026-10-30_120000', [entry('P00'), entry('P01', 'changed')])
    lap(tmp_path, '2026-10-30_120100', [entry('P01'), entry('P00', 'changed')])
    assert order(node(tmp_path)) == ['P00', 'P01']


def test_manual_request_overrides_newest_change(tmp_path):
    name = '2026-10-30_120000'
    lap(tmp_path, name, [entry('P00'), entry('P01', 'changed')])
    (tmp_path / 'reviews.json').write_text(json.dumps({name + '/P00': {'result': '조치 필요'}}))
    assert order(node(tmp_path)) == ['P00', 'P01']


def test_acknowledgment_clears_old_but_not_new_change(tmp_path):
    name = '2026-10-30_120000'
    lap(tmp_path, name, [entry('P00'), entry('P01', 'changed')])
    (tmp_path / 'reviews.json').write_text(json.dumps({name + '/P01': {'result': '확인 완료'}}))
    assert order(node(tmp_path)) == ['P00', 'P01']
    lap(tmp_path, '2026-10-30_120100', [entry('P00'), entry('P01', 'changed')])
    assert order(node(tmp_path)) == ['P01', 'P00']


def test_success_clears_old_transient_failure(tmp_path):
    lap(tmp_path, '2026-10-30_120000', [entry('P00'), entry('P01', 'align_failed')])
    lap(tmp_path, '2026-10-30_120100', [entry('P01'), entry('P00')])
    assert order(node(tmp_path)) == ['P00', 'P01']


def test_history_excludes_other_routes_and_skips_corrupt_json(tmp_path):
    lap(tmp_path, '2026-10-30_120000', [entry('P00'), entry('P01', 'changed')])
    lap(tmp_path, '2026-10-30_120100', [entry('P00', 'changed')], route='other')
    broken = lap(tmp_path, '2026-10-30_120200', [])
    (broken / 'result.json').write_text('{')
    assert order(node(tmp_path)) == ['P01', 'P00']


def test_prioritize_false_keeps_yaml_order(tmp_path):
    lap(tmp_path, '2026-10-30_120000', [entry('P00'), entry('P01', 'changed')])
    n = node(tmp_path)
    n.prioritize = False
    assert order(n) == ['P00', 'P01']


def test_fixed_views_do_not_change_with_first_arrival_error(tmp_path):
    n = node(tmp_path)
    origin = (.18, -.52, -.1)
    cx, cy = .8, -.6
    for target in n.recheck_poses(STOPS[0]):
        assert math.hypot(target['x'] - cx, target['y'] - cy) == pytest.approx(.65)
        assert math.hypot(target['x'] - origin[0], target['y'] - origin[1]) > .30
        assert target['yaw'] == pytest.approx(math.atan2(cy-target['y'], cx-target['x']))


def test_close_first_capture_does_not_make_recheck_close_to_pot(tmp_path):
    n = node(tmp_path)
    origin = (.38, -.487, -.1)  # Gazebo에서 회전 중 충돌 예측으로 막혔던 접근 위치
    for target in n.recheck_poses(STOPS[0]):
        assert math.hypot(target['x']-.8, target['y']+.6) == pytest.approx(.65)
        # 직선 이동 중에도 시작 위치보다 화분에 더 다가가는 목표를 만들지 않는다.
        for fraction in (0.25, .5, .75, 1.):
            x=origin[0]+fraction*(target['x']-origin[0])
            y=origin[1]+fraction*(target['y']-origin[1])
            assert math.hypot(x-.8,y+.6) >= math.hypot(origin[0]-.8,origin[1]+.6)


def test_capture_at_plant_center_rejects_invalid_recheck_geometry(tmp_path):
    n = node(tmp_path)
    n.fresh_pose = lambda frame: (.8,-.6,0.)
    n.go = Mock()
    result = n.recheck_stop(str(tmp_path), STOPS[0])
    assert result['status'] == 'invalid_geometry'
    n.go.assert_not_called()


def moving_node(tmp_path, outcomes=None):
    n = node(tmp_path)
    state = {'pose': (.3, -.6, 0.)}
    events = []
    n.fresh_pose = lambda frame: state['pose']
    n.robot_pose = lambda: state['pose']
    n.get_logger = lambda: Mock()
    def go(target):
        events.append(('go', target.get('id', 'recheck')))
        outcome = next(outcomes) if outcomes else 'ok'
        if outcome == 'fail':
            return TaskResult.FAILED
        if outcome != 'stationary':
            state['pose'] = (target['x'], target['y'], target['yaw'])
        return TaskResult.SUCCEEDED
    def capture(path):
        events.append(('capture', Path(path).name))
        Path(path).write_bytes(b'fake image')
        return True
    n.go = Mock(side_effect=go)
    n.capture = Mock(side_effect=capture)
    n.compare_stop = Mock(return_value={'status':'changed', 'changes':{'leaf_color':1}})
    return n, events, state


def test_real_movement_is_recorded(tmp_path):
    n, _, _ = moving_node(tmp_path)
    result = n.recheck_stop(str(tmp_path), STOPS[0])
    assert result['status'] == 'ok'
    assert result['movement_m'] > .30
    assert result['viewpoint_change_deg'] == pytest.approx(35.)
    assert 'pose' in result and 'origin_odom_pose' in result
    # 변화가 의심되면 왼쪽·오른쪽을 모두 찍는다
    assert n.capture.call_count == 2
    assert set(result['views']) == {'left', 'right'}


@pytest.mark.parametrize('first', ['fail', 'stationary'])
def test_second_side_after_failed_first_side(tmp_path, first):
    n, _, _ = moving_node(tmp_path, iter([first, 'ok']))
    result = n.recheck_stop(str(tmp_path), STOPS[0])
    assert result['status'] == 'incomplete'
    assert n.go.call_count == 2
    assert n.capture.call_count == 1
    assert len(result['attempts']) == 2


def test_map_jump_without_odom_motion_is_not_recheck(tmp_path):
    n = node(tmp_path)
    original = (.3, -.6, 0.)
    target = n.recheck_poses(STOPS[0])[0]
    n.fresh_pose = lambda frame: (target['x'], target['y'], target['yaw']) if frame == 'map' else original
    enough, measured = n.recheck_motion(STOPS[0], original, original)
    assert not enough
    assert measured['movement_m'] == 0.
    assert measured['viewpoint_change_deg'] > 10.


def test_missing_pose_prevents_recheck_navigation(tmp_path):
    n = node(tmp_path)
    n.fresh_pose = lambda frame: None
    n.go = Mock()
    assert n.recheck_stop(str(tmp_path), STOPS[0])['status'] == 'pose_unavailable'
    n.go.assert_not_called()


def test_pose_loss_during_capture_not_confirmed(tmp_path):
    n, _, _ = moving_node(tmp_path)
    n.capture = Mock(side_effect=lambda path: setattr(n, 'fresh_pose', lambda frame: None) or True)
    result = n.recheck_stop(str(tmp_path), STOPS[0])
    assert result['status'] == 'pose_unavailable'
    n.compare_stop.assert_not_called()


def test_recheck_completes_before_next_stop(tmp_path):
    from test_view_patrol import write_baseline
    write_baseline(tmp_path)
    n, events, _ = moving_node(tmp_path)
    n.route_path = 'test.yaml'
    n.image_rotation = 0
    n.compare = n.recheck = True
    n.plan_order = lambda folder: [(s, None) for s in STOPS]
    n.log_result = Mock()
    n.compare_stop = Mock(side_effect=[{'status':'changed','changes':{'leaf_color':1}},
                                      {'status':'unchanged','changes':{}}])
    PatrolManager.run_lap(n, 1)
    assert events.index(('capture', 'P00_left.png')) < events.index(('go', 'P01'))
    assert events.index(('capture', 'P00_right.png')) < events.index(('go', 'P01'))
    result = json.loads(sorted(tmp_path.glob('*/result.json'))[-1].read_text())
    assert result['stops'][0]['final'] == {'status':'changed','changes':{'leaf_color':1},'source':'front'}
    assert n.compare_stop.call_count == 2
    assert not any('compare' in v for v in result['stops'][0]['recheck']['views'].values())
    assert result['stops'][0]['recheck']['movement_m'] > .30
    assert result['stops'][0]['observed_at']


@pytest.mark.parametrize('status', ['changed','align_failed','no_plant','no_image','error'])
def test_only_front_change_requires_admin_side_photos(status):
    from plant_observer_bringup.patrol_manager import needs_recheck
    assert needs_recheck('ok', {'status':status}) == (status == 'changed')


def test_first_baseline_does_not_require_recheck():
    from plant_observer_bringup.patrol_manager import needs_recheck
    assert not needs_recheck('ok', {'status':'no_reference'})
    assert not needs_recheck('ok', {'status':'unchanged'})


def test_failed_image_write_is_not_capture_success(tmp_path, monkeypatch):
    import numpy as np
    import plant_observer_bringup.patrol_manager as pm
    from rclpy.time import Time
    from rclpy.clock import ClockType
    n = node(tmp_path)
    n.settle_time, n.image_timeout, n.image_rotation = 0., 1., 0
    n.spin_for = lambda seconds: None
    n.get_clock = lambda: SimpleNamespace(now=lambda: Time(seconds=1,clock_type=ClockType.ROS_TIME))
    n.last_image = SimpleNamespace(header=SimpleNamespace(stamp=Time(seconds=2).to_msg()))
    n.bridge = SimpleNamespace(imgmsg_to_cv2=lambda *a: np.zeros((10,10,3),np.uint8))
    monkeypatch.setattr(pm.rclpy, 'spin_once', lambda *a, **kw: None)
    assert not PatrolManager.capture(n, str(tmp_path/'missing_directory'/'P00.png'))


def test_change_found_only_on_recheck_keeps_priority(tmp_path):
    recovered = dict(id='P01', status='no_image',
                     final=dict(verdict='recheck_used', status='changed', changes={'fruit_added':1}))
    lap(tmp_path, '2026-10-30_120000', [entry('P00'), recovered])
    lap(tmp_path, '2026-10-30_120100', [entry('P00'), entry('P01')])
    assert order(node(tmp_path)) == ['P01', 'P00']


@pytest.mark.parametrize('stamp,expected', [(9.9,True),(8.,False),(12.,False)])
def test_recheck_pose_requires_recent_tf(tmp_path, monkeypatch, stamp, expected):
    import plant_observer_bringup.patrol_manager as pm
    monkeypatch.setattr(pm.rclpy, 'spin_once', lambda *a, **kw: None)
    from geometry_msgs.msg import TransformStamped
    from rclpy.clock import ClockType
    from rclpy.time import Time
    n=node(tmp_path)
    n.base_frame='base_footprint'
    n.pose_max_age=.5
    transform=TransformStamped()
    transform.header.stamp=Time(seconds=stamp).to_msg()
    transform.transform.rotation.w=1.
    n.tf_buffer=Mock()
    n.tf_buffer.lookup_transform.return_value=transform
    n.get_clock=lambda: SimpleNamespace(now=lambda:Time(seconds=10,clock_type=ClockType.ROS_TIME))
    assert (PatrolManager.fresh_pose(n,'odom',wait=0.) is not None) == expected


def test_recheck_pose_waits_for_tf_after_slow_compare(tmp_path, monkeypatch):
    """사진 비교로 TF 버퍼가 묵어 있어도 노드를 돌려 새 TF가 들어오면 쓴다 (실물 2026-10-30_193421)."""
    import plant_observer_bringup.patrol_manager as pm
    from geometry_msgs.msg import TransformStamped
    from rclpy.clock import ClockType
    from rclpy.time import Time
    n=node(tmp_path)
    n.base_frame='base_footprint'
    n.pose_max_age=.5
    def transform(stamp):
        t=TransformStamped()
        t.header.stamp=Time(seconds=stamp).to_msg()
        t.transform.rotation.w=1.
        return t
    stale, fresh = transform(9.), transform(10.)
    n.tf_buffer=Mock()
    n.tf_buffer.lookup_transform.return_value=stale
    monkeypatch.setattr(pm.rclpy, 'spin_once',
                        lambda *a, **kw: setattr(n.tf_buffer.lookup_transform, 'return_value', fresh)
                        if n.tf_buffer.lookup_transform.call_count >= 2 else None)
    n.get_clock=lambda: SimpleNamespace(now=lambda:Time(seconds=10,clock_type=ClockType.ROS_TIME))
    assert PatrolManager.fresh_pose(n,'map') is not None
