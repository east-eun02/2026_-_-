"""관찰 기록 화면의 로봇 제어: systemd 사용자 서비스로 로봇(plant-onboard)과 순찰(plant-patrol)을 켜고 끈다.

순찰 인자(루트·회차 수·복귀만)는 $XDG_RUNTIME_DIR/plant-patrol.env에 쓰고 plant-patrol을 시작한다.
plant-patrol은 plant-onboard에 묶여 있어서(Requisite·PartOf·After) 로봇이 꺼져 있으면 시작되지 않고,
로봇을 끄면 순찰이 먼저 멈춘 뒤(이동 취소) 로봇이 꺼진다.
바퀴·라이다·카메라·Nav2·위치 상태는 rclpy가 있으면 작은 노드를 띄워 토픽으로 본다.
"""
import os
import subprocess
import threading
import time

ROBOT_UNIT = 'plant-onboard.service'
PATROL_UNIT = 'plant-patrol.service'
PATROL_ENV = 'plant-patrol.env'
ACTIONS = ('robot_start', 'robot_stop', 'patrol_start', 'patrol_stop', 'home', 'poweroff')
RUNNING = ('active', 'activating', 'deactivating', 'reloading')


def systemctl(*args, timeout=10.):
    try:
        return subprocess.run(['systemctl', '--user', *args], capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return subprocess.CompletedProcess(args, 1, '', str(exc))


def unit_state(unit):
    out = systemctl('show', unit, '-p', 'LoadState', '-p', 'ActiveState', '-p', 'Result',
                    '-p', 'ActiveEnterTimestamp').stdout
    props = dict(line.split('=', 1) for line in out.splitlines() if '=' in line)
    return {'installed': props.get('LoadState') == 'loaded', 'state': props.get('ActiveState', 'unknown'),
            'result': props.get('Result'), 'since': props.get('ActiveEnterTimestamp') or None}


class RosMonitor:
    """로봇 상태를 ROS 토픽으로 본다. 메시지가 FRESH초 안에 들어왔으면 살아 있는 것으로 본다.

    ROS 환경 없이 관찰 기록 화면만 띄워도 되도록 rclpy는 스레드 안에서 불러온다.
    """
    FRESH = 2.0

    def __init__(self):
        self.available = False
        self.error = None
        self.seen = {}
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            import rclpy
            from lifecycle_msgs.srv import GetState
            from nav_msgs.msg import Odometry
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from rclpy.signals import SignalHandlerOptions
            from rclpy.time import Time
            from sensor_msgs.msg import CameraInfo, LaserScan
            from tf2_ros import Buffer, TransformListener

            # Ctrl-C는 관찰 기록 화면(KeyboardInterrupt)이 받는다
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node('plant_review_status')
            mark = lambda key: lambda _msg: self.seen.__setitem__(key, time.monotonic())
            node.create_subscription(Odometry, '/odom', mark('wheels'), qos_profile_sensor_data)
            node.create_subscription(LaserScan, '/scan_filtered', mark('lidar'), qos_profile_sensor_data)
            # 영상 대신 작은 camera_info로 카메라가 살아 있는지 본다
            node.create_subscription(CameraInfo, '/camera/camera_info', mark('camera'), qos_profile_sensor_data)
            tf_buffer = Buffer()
            TransformListener(tf_buffer, node)
            client = node.create_client(GetState, 'bt_navigator/get_state')

            def on_state(future):
                if future.result() is not None and future.result().current_state.id == 3:   # active
                    self.seen['nav2'] = time.monotonic()

            def check():
                if client.service_is_ready():
                    client.call_async(GetState.Request()).add_done_callback(on_state)
                if tf_buffer.can_transform('map', 'base_footprint', Time()):
                    self.seen['localized'] = time.monotonic()

            node.create_timer(1.0, check)
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            self.available = True
            executor.spin()
        except Exception as exc:   # ROS가 없거나 초기화 실패: 화면은 그대로 쓰고 상태만 '확인 불가'
            self.available = False
            self.error = repr(exc)

    def snapshot(self):
        now = time.monotonic()
        # Nav2·위치는 1초마다 확인하므로 조금 더 여유를 둔다
        limits = {'wheels': self.FRESH, 'lidar': self.FRESH, 'camera': self.FRESH,
                  'nav2': self.FRESH + 2., 'localized': self.FRESH + 2.}
        return {'available': self.available,
                **{key: now - self.seen.get(key, -1e9) < limit for key, limit in limits.items()}}


class Controller:

    def __init__(self, route, max_laps=5, runtime_dir=None, ros=True):
        self.route = route
        self.max_laps = max_laps
        self.env_path = os.path.join(runtime_dir or os.environ.get('XDG_RUNTIME_DIR') or '/tmp', PATROL_ENV)
        self.lock = threading.Lock()
        self.busy = None   # 노트북 끄기처럼 오래 걸리는 동작 중이면 그 설명
        self.notice = None   # 뒤에서 돌다 실패한 동작의 안내 (화면에 한 번 보여 줌)
        self.ros = RosMonitor() if ros else None
        self._poweroff_checked = (0., None)

    def patrol_request(self):
        """마지막으로 시작한 순찰 인자 (상태 표시용)."""
        values = {}
        try:
            with open(self.env_path) as f:
                for line in f:
                    key, _, value = line.strip().partition('=')
                    values[key] = value
        except OSError:
            pass
        return {'mode': values.get('PATROL_MODE', 'patrol'), 'laps': int(values.get('PATROL_LAPS', 1) or 1)}

    def can_poweroff(self):
        """로그인 없이 돈 서비스에서도 전원을 끌 수 있는지 (logind 권한). 1분마다 다시 확인."""
        checked, value = self._poweroff_checked
        if time.monotonic() - checked > 60.:
            try:
                out = subprocess.run(['busctl', 'call', '--system', 'org.freedesktop.login1',
                                      '/org/freedesktop/login1', 'org.freedesktop.login1.Manager', 'CanPowerOff'],
                                     capture_output=True, text=True, timeout=5).stdout
                value = out.split('"')[1] if '"' in out else None
            except (OSError, subprocess.TimeoutExpired):
                value = None
            self._poweroff_checked = (time.monotonic(), value)
        return value

    def status(self):
        robot, patrol = unit_state(ROBOT_UNIT), unit_state(PATROL_UNIT)
        patrol.update(self.patrol_request())
        ros = self.ros.snapshot() if self.ros and robot['state'] == 'active' else None
        notice, self.notice = self.notice, None
        return {'route': self.route, 'max_laps': self.max_laps, 'robot': robot, 'patrol': patrol,
                'ros': ros, 'busy': self.busy, 'notice': notice, 'can_poweroff': self.can_poweroff()}

    def run(self, action, laps=1):
        """(성공 여부, 사람이 읽는 말). 긴 동작은 기다리지 않고 바로 돌려준다(--no-block)."""
        with self.lock:
            if self.busy:
                return False, f'{self.busy}입니다'
            robot, patrol = unit_state(ROBOT_UNIT), unit_state(PATROL_UNIT)
            if not robot['installed'] or not patrol['installed']:
                return False, '로봇 서비스가 설치되지 않았습니다 (install_onboard_service.sh)'
            if action == 'robot_start':
                if robot['state'] in RUNNING:
                    return False, '로봇이 이미 켜져 있습니다'
                systemctl('reset-failed', ROBOT_UNIT)
                return self._job(systemctl('start', '--no-block', ROBOT_UNIT),
                                 '로봇을 켜는 중입니다. 준비까지 1~2분 걸립니다')
            if action == 'robot_stop':
                # 순찰은 PartOf로 먼저 멈춘다
                return self._job(systemctl('stop', '--no-block', ROBOT_UNIT),
                                 '로봇을 끄는 중입니다. 순찰 중이면 먼저 멈춥니다')
            if action in ('patrol_start', 'home'):
                if robot['state'] != 'active':
                    return False, '로봇을 먼저 켜세요'
                if patrol['state'] in RUNNING:
                    return False, '이미 순찰 중입니다. 정지한 뒤 다시 누르세요'
                laps = max(1, min(int(laps), self.max_laps))
                mode = 'home' if action == 'home' else 'patrol'
                with open(self.env_path + '.tmp', 'w') as f:
                    f.write(f'PATROL_ROUTE={self.route}\nPATROL_LAPS={laps}\nPATROL_MODE={mode}\n')
                os.replace(self.env_path + '.tmp', self.env_path)
                systemctl('reset-failed', PATROL_UNIT)
                return self._job(systemctl('start', '--no-block', PATROL_UNIT),
                                 '출발점으로 돌아갑니다' if mode == 'home' else f'순찰을 시작합니다 ({laps}회)')
            if action == 'patrol_stop':
                if patrol['state'] not in RUNNING:
                    return False, '순찰 중이 아닙니다'
                return self._job(systemctl('stop', '--no-block', PATROL_UNIT),
                                 '정지합니다. 진행 중인 이동을 취소하고 그 자리에 멈춥니다')
            if action == 'poweroff':
                self.busy = '노트북을 끄는 중'
                threading.Thread(target=self._poweroff, daemon=True).start()
                return True, '로봇을 끈 뒤 노트북 전원을 끕니다'
            return False, '알 수 없는 동작입니다'

    @staticmethod
    def _job(result, message):
        if result.returncode != 0:
            return False, '실패: ' + (result.stderr.strip() or f'systemctl 종료 코드 {result.returncode}')
        return True, message

    def _poweroff(self):
        # 순서대로 종료(Nav2 → 모터 토크 끔)가 끝날 때까지 기다린 뒤 전원을 끈다
        systemctl('stop', ROBOT_UNIT, timeout=120.)
        try:
            result = subprocess.run(['systemctl', 'poweroff'], capture_output=True, text=True, timeout=30)
            error = result.stderr.strip() if result.returncode != 0 else None
        except (OSError, subprocess.TimeoutExpired) as exc:
            error = str(exc)
        if error:
            self.busy = None
            self.notice = f'로봇은 껐지만 노트북 전원을 끄지 못했습니다: {error}'
