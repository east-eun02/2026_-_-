"""Control panel: systemctl calls are faked, so no unit is started and no robot moves."""
import json
import subprocess
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from plant_observer_bringup import robot_control as rc
from plant_observer_bringup import review_server as rs


class FakeSystemd:
    """Remembers calls and answers `show` with the states in self.units."""

    def __init__(self, robot='inactive', patrol='inactive'):
        self.units = {rc.ROBOT_UNIT: robot, rc.PATROL_UNIT: patrol}
        self.calls = []

    def __call__(self, *args, timeout=10.):
        self.calls.append(args)
        if args[0] == 'show':
            out = f'LoadState=loaded\nActiveState={self.units[args[1]]}\nResult=success\nActiveEnterTimestamp=\n'
            return subprocess.CompletedProcess(args, 0, out, '')
        return subprocess.CompletedProcess(args, 0, '', '')

    def actions(self):
        return [c for c in self.calls if c[0] != 'show']


@pytest.fixture
def fake(monkeypatch):
    systemd = FakeSystemd()
    monkeypatch.setattr(rc, 'systemctl', systemd)
    return systemd


@pytest.fixture
def controller(tmp_path):
    return rc.Controller('demo_real3', max_laps=5, runtime_dir=str(tmp_path), ros=False)


def test_patrol_refused_while_robot_off(fake, controller, tmp_path):
    ok, text = controller.run('patrol_start', 2)
    assert not ok and '로봇' in text
    assert fake.actions() == []
    assert not (tmp_path / rc.PATROL_ENV).exists()


def test_patrol_start_writes_arguments_and_clamps_laps(fake, controller, tmp_path):
    fake.units[rc.ROBOT_UNIT] = 'active'
    ok, _ = controller.run('patrol_start', 99)
    assert ok
    env = (tmp_path / rc.PATROL_ENV).read_text()
    assert 'PATROL_ROUTE=demo_real3' in env and 'PATROL_LAPS=5' in env and 'PATROL_MODE=patrol' in env
    assert ('start', '--no-block', rc.PATROL_UNIT) in fake.actions()


def test_home_uses_home_mode_and_refuses_during_patrol(fake, controller, tmp_path):
    fake.units[rc.ROBOT_UNIT] = 'active'
    assert controller.run('home')[0]
    assert 'PATROL_MODE=home' in (tmp_path / rc.PATROL_ENV).read_text()
    fake.units[rc.PATROL_UNIT] = 'active'
    ok, text = controller.run('home')
    assert not ok and '순찰 중' in text
    assert controller.status()['patrol']['mode'] == 'home'


def test_robot_stop_does_not_wait_and_patrol_stop_needs_patrol(fake, controller):
    assert not controller.run('patrol_stop')[0]
    fake.units[rc.ROBOT_UNIT] = 'active'
    assert controller.run('robot_stop')[0]
    assert fake.actions() == [('stop', '--no-block', rc.ROBOT_UNIT)]


def test_robot_start_refused_when_running(fake, controller):
    fake.units[rc.ROBOT_UNIT] = 'activating'
    assert not controller.run('robot_start')[0]
    fake.units[rc.ROBOT_UNIT] = 'failed'
    assert controller.run('robot_start')[0]
    assert ('reset-failed', rc.ROBOT_UNIT) in fake.actions()


@pytest.fixture
def server(fake, controller, tmp_path):
    handler = rs.make_handler(rs.Store(str(tmp_path)), controller, pin='1234')
    httpd = ThreadingHTTPServer(('127.0.0.1', 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{httpd.server_address[1]}'
    httpd.shutdown()


def post(url, body, headers):
    request = urllib.request.Request(url + '/api/control', json.dumps(body).encode(), headers, method='POST')
    try:
        with urllib.request.urlopen(request) as res:
            return res.status, json.load(res)
    except urllib.error.HTTPError as err:
        return err.code, json.load(err)


def test_control_requires_pin_and_json(server, fake):
    assert post(server, {'action': 'robot_start'}, {'Content-Type': 'application/json'})[0] == 403
    assert post(server, {'action': 'robot_start'},
                {'Content-Type': 'text/plain', 'X-Panel-Pin': '1234'})[0] == 400
    assert fake.actions() == []
    status, body = post(server, {'action': 'robot_start'},
                        {'Content-Type': 'application/json', 'X-Panel-Pin': '1234'})
    assert status == 200 and body['ok']
    assert ('start', '--no-block', rc.ROBOT_UNIT) in fake.actions()


def test_control_status_reports_units(server, fake):
    with urllib.request.urlopen(server + '/api/control') as res:
        body = json.load(res)
    assert body['enabled'] and body['pin_required']
    assert body['robot']['state'] == 'inactive' and body['patrol']['mode'] == 'patrol'
