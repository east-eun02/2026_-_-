"""Side-photo ordering, Nav2 configuration readiness, and movement records."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from plant_observer_bringup import patrol_manager as pm
from test_patrol_behavior import node, STOPS, moving_node
from test_view_patrol import baseline_node


@pytest.mark.parametrize('home_x', [-10., 0., 10.])
def test_right_then_left_is_independent_of_home_distance(tmp_path, home_x):
    n = node(tmp_path)
    n.home['x'] = home_x
    for stop in STOPS:
        assert [p['view_id'] for p in n.recheck_poses(stop)] == ['right', 'left']


def test_enrollment_uses_center_right_left(tmp_path):
    n, events, _ = baseline_node(tmp_path)
    pm.PatrolManager.run_lap(n, 1)
    assert [name for event, name in events if event == 'capture'] == [
        'P00.png', 'P00_right.png', 'P00_left.png', 'P01.png', 'P01_right.png', 'P01_left.png']


@pytest.mark.parametrize('plugins,valid', [
    (['general_goal_checker', 'capture_goal_checker'], False),
    (['general_goal_checker', 'capture_goal_checker', 'side_capture_goal_checker'], True)])
def test_old_nav2_config_is_rejected_before_patrol(plugins, valid):
    future = Mock()
    future.done.return_value = True
    future.result.return_value = SimpleNamespace(values=[SimpleNamespace(string_array_value=plugins)])
    client = Mock()
    client.wait_for_service.return_value = True
    client.call_async.return_value = future
    n = SimpleNamespace(create_client=Mock(return_value=client), destroy_client=Mock())
    if valid:
        pm.PatrolManager.check_capture_goal_checkers(n)
    else:
        with pytest.raises(RuntimeError, match='터미널 2'):
            pm.PatrolManager.check_capture_goal_checkers(n)
    n.destroy_client.assert_called_once_with(client)
    assert client.call_async.call_args.args[0].names == ['goal_checker_plugins']


def test_readiness_lookup_has_deadline(monkeypatch):
    client = Mock()
    client.wait_for_service.return_value = False
    n = SimpleNamespace(create_client=Mock(return_value=client), destroy_client=Mock())
    clock = iter([0., 0., 7.])
    monkeypatch.setattr(pm.time, 'monotonic', lambda: next(clock))
    with pytest.raises(TimeoutError):
        pm.PatrolManager.check_capture_goal_checkers(n)
    n.destroy_client.assert_called_once_with(client)


def test_dropped_readiness_response_is_cancelled_and_retried(monkeypatch):
    lost, good = Mock(), Mock()
    lost.done.return_value = False
    good.done.return_value = True
    good.result.return_value = SimpleNamespace(values=[SimpleNamespace(
        string_array_value=['capture_goal_checker', 'side_capture_goal_checker'])])
    client = Mock()
    client.wait_for_service.return_value = True
    client.call_async.side_effect = [lost, good]
    n = SimpleNamespace(create_client=Mock(return_value=client), destroy_client=Mock())
    clock = iter([0., 0., 0., 3., 3., 3.])
    monkeypatch.setattr(pm.time, 'monotonic', lambda: next(clock))
    pm.PatrolManager.check_capture_goal_checkers(n)
    lost.cancel.assert_called_once()
    assert client.call_async.call_count == 2


def test_side_records_navigation_time_and_target_error(tmp_path):
    n, _, _ = moving_node(tmp_path)
    target = n.recheck_poses(STOPS[0])[0]
    result = n.capture_view(str(tmp_path), STOPS[0], target)
    assert result['status'] == 'ok'
    assert result['navigation_elapsed_s'] >= 0.
    assert result['error_xy_m'] == 0. and result['error_yaw_deg'] == 0.


def test_side_tree_keeps_collision_protection():
    config = Path(get_package_share_directory('plant_observer_gazebo'))/'config'
    params = yaml.safe_load((config/'nav2_params.yaml').read_text())
    assert params['controller_server']['ros__parameters']['FollowPath']['use_collision_detection']
    monitor = params['collision_monitor']['ros__parameters']
    assert monitor['FootprintApproach']['enabled']
    assert monitor['FootprintApproach']['action_type'] == 'approach'
