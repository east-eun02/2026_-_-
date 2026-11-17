"""Completion-triggered uploads, using fake motion and local Git remotes only."""
import json
import subprocess
import sys
import time
from types import MethodType
from unittest.mock import Mock

import pytest

from plant_observer_bringup import capture_events as events
from plant_observer_bringup import capture_uploader as cu
from plant_observer_bringup import patrol_manager as pm
from test_capture_uploader import write_lap
from test_previous_patrol import previous_node
from test_system_verification import repos, git


def wait_until(predicate, process, log, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        assert process.poll() is None, log.read_text()
        time.sleep(0.02)
    pytest.fail(f'Uploader did not react within {timeout}s:\n{log.read_text()}')


@pytest.fixture
def start_uploader(repos, tmp_path):
    captures, repo, _ = repos
    processes = []

    def start(interval=3600):
        log = tmp_path / 'uploader.log'
        with log.open('w') as output:
            process = subprocess.Popen(
                [sys.executable, '-m', 'plant_observer_bringup.capture_uploader',
                 '--capture-dir', str(captures), '--repo', str(repo),
                 '--interval', str(interval)], stdout=output, stderr=subprocess.STDOUT)
        processes.append(process)
        # The initial empty index is published before the daemon starts waiting.
        wait_until(lambda: 'push 완료' in log.read_text(), process, log)
        return process, log

    yield start
    for process in processes:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def test_just_finished_lap_is_immediately_eligible(tmp_path):
    write_lap(tmp_path, '2026-10-31_010000', finished=True, age=0)
    write_lap(tmp_path, '2026-10-31_010001', finished=False, age=0)
    assert cu.finished_laps(str(tmp_path)) == ['2026-10-31_010000']


@pytest.mark.parametrize('mode', ['previous', 'baseline', 'fixed_reference'])
def test_each_patrol_completion_wakes_idle_uploader(repos, start_uploader, mode):
    captures, _, remote = repos
    process, log = start_uploader()  # A one-hour retry timer cannot satisfy this test.
    node, _, _ = previous_node(captures)
    node.capture_baseline = mode == 'baseline'
    if mode == 'fixed_reference':
        node.compare_to = 'baseline'
        node.compare = False
    node.wait_ready = Mock()
    node.laps = 1
    node.run_lap = MethodType(pm.PatrolManager.run_lap, node)

    for _ in range(2):
        pm.PatrolManager.run(node)
        folder = sorted(captures.glob('*/result.json'))[-1].parent
        wait_until(lambda: git(remote, 'cat-file', '-e',
                               f'main:captures/{folder.name}/result.json', check=False).returncode == 0,
                   process, log)
        index = json.loads(git(remote, 'show', 'main:captures/index.json').stdout)
        assert index['laps'][0]['lap'] == folder.name
        for photo in folder.glob('*.png'):
            assert git(remote, 'show', f'main:captures/{folder.name}/{photo.name}').stdout == 'fake image'
        if mode == 'baseline':
            assert index['baselines']['demo'] == folder.name
    assert log.read_text().count('순찰 완료 알림: 즉시 업로드 시작') >= 2


def test_missed_notification_is_recovered_by_retry_timer(repos, start_uploader):
    captures, _, remote = repos
    process, log = start_uploader(interval=0.2)
    folder = write_lap(captures, '2026-10-31_010000', finished=True, age=0)
    wait_until(lambda: git(remote, 'cat-file', '-e',
                           f'main:captures/{folder.name}/P00.png', check=False).returncode == 0,
               process, log)


def test_stopped_patrol_does_not_notify_uploader(tmp_path, monkeypatch):
    node, _, _ = previous_node(tmp_path)
    node.wait_ready = Mock()
    node.laps = 1
    node.run_lap = Mock(side_effect=KeyboardInterrupt)
    notify = Mock()
    monkeypatch.setattr(pm, 'notify_completion', notify)
    with pytest.raises(KeyboardInterrupt):
        pm.PatrolManager.run(node)
    notify.assert_not_called()


def test_unavailable_uploader_does_not_fail_completed_patrol(tmp_path):
    node, _, _ = previous_node(tmp_path)
    node.wait_ready = Mock()
    node.laps = 1
    node.run_lap = MethodType(pm.PatrolManager.run_lap, node)
    pm.PatrolManager.run(node)
    result = json.loads(next(tmp_path.glob('*/result.json')).read_text())
    assert result['finished']


def test_completion_is_delivered_only_to_same_capture_directory(tmp_path):
    source = tmp_path / 'captures'
    source.mkdir()
    alias = tmp_path / 'alias'
    alias.symlink_to(source)
    with events.completion_listener(str(source)) as listener:
        assert not events.notify_completion(str(tmp_path / 'other'))
        assert events.notify_completion(str(alias))
        assert events.wait_for_completion(listener, 0.1)
        assert not events.wait_for_completion(listener, 0.01)
