"""순찰 노드: 관측 지점을 차례로 돌며 화분 사진을 찍고 출발점으로 돌아온다.

기획안 그림 2의 patrol_manager 첫 버전. 지점 목록은 plant_observer_gazebo/config/patrol_<route>.yaml.
지점마다 Nav2로 이동 → settle_time 동안 멈춰 있기 → 카메라 사진 저장 → 도착 위치 오차 기록.
이동에 실패한 지점은 nav_failed(확인 필요)로 남기고 다음 지점으로 간다.

결과는 <capture_dir>/<회차 시작 시각>/ 아래에 <지점 ID>.png 와 result.json.
찍자마자 기준 사진과 비교(change_compare)해서 변화 종류·개수를 지점마다 compare에 남기고,
비교 그림은 <지점 ID>_compare.jpg. 회차 끝에 변화가 생긴 지점을 모아 로그로 알려준다.
화분별 첫 촬영에는 정면·왼쪽·오른쪽을 저장한다. 이후에는 정면만 직전 완료 순찰의 정면과 비교한다.
정면에서 변화가 감지되면 왼쪽·오른쪽을 추가 촬영하고 다음 화분으로 이동한다.
옆면은 관리자 확인용 자료이며 비교·자동 확정에 사용하지 않는다. final은 정면 판정만 담는다.
정상 비교가 불가능한 정면만 사전학습 모델(YOLO) 영역으로 보조한다(기본 켜짐, 끄려면 model_fallback:=false).
모델 결과는 변화 없음이어도 관리자 확인 필요로 남기며, 원래 실패 사유와 모델 정보를 보존한다.
열매 생성·손상·탈락은 사과 검출 모델(apple_weights, 기본 models/apple_yolo26n_aug_v1.pt)로 판정한다
(끄려면 apple_model:=false). 모델이 없거나 실패하면 색 규칙으로 판정하고 compare.apple_model에 남긴다.
회차 시작 때 같은 루트의 기록과 관리자 확인 결과로 방문 순서를 정한다:
관리자 조치·재촬영 요청 → 미확인 변화(최근 감지 순) → 최근 관측 실패 → 나머지.
직전 사진과 같다는 이유만으로 미확인 변화를 지우지 않는다. 확인 완료·오탐 표시로 해제한다.
재촬영은 최초 촬영 위치 기준 실제 이동량과 화분을 보는 각도 변화가 충분해야 성공으로 인정한다.
기본 previous 모드는 직전 완료 순찰의 정면과 비교한다. 정렬 실패 등은 판정 불가로 기록한다.
고정 정상 기준 비교는 compare_to:=baseline으로 선택한다.
capture_baseline:=true로 정상 상태에서 정면·왼쪽·오른쪽 기준을 한 번 등록한다.
세 방향 촬영이 모두 성공한 회차만 기준으로 등록하며, 실패하면 기존 기준을 유지한다.

Nav2(bringup_launch.py)와 AMCL이 떠 있어야 한다. 로봇은 루트의 home에 놓고 시작한다.
AMCL 위치(nav2_params.yaml의 initial_pose, 보통 지도 원점)가 home에서 reset_pose_distance 넘게
떨어져 있으면 home을 /initialpose로 보낸다. 지도 원점이 출발점이 아닌 지도(demo_real2)도
RViz 2D Pose Estimate 없이 돌릴 수 있다. 직전 회차로 home에 돌아와 있으면 보내지 않는다.

    ros2 run plant_observer_bringup patrol_manager --ros-args -p route:=greenhouse -p use_sim_time:=true
"""
import json
import math
import os
import signal
import time
from datetime import datetime, timedelta

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

from plant_observer_bringup.change_compare import (compare_with_baseline, describe_changes,
                                                    baseline_file, previous_lap, observed_stops)
from plant_observer_bringup.capture_events import notify_completion

# 비교 결과 status를 사람이 읽는 말로 ('changed'는 변화 종류로 따로 적는다)
COMPARE_TEXT = {
    'unchanged': '변화 없음',
    'no_reference': '기준 사진 없음(첫 회차)',
    'align_failed': '비교 불가(정렬 실패)',
    'no_plant': '비교 불가(식물이 안 보임)',
    'board_not_found': '비교 불가(보드 경계 확인 필요)',
    'no_image': '비교 불가(사진 읽기 실패)',
    'view_mismatch': '비교 불가(기준과 촬영 위치·방향 다름)',
}


def describe_change(result):
    """'잎 탈락 2곳'처럼 변화 종류와 개수. 변화가 아니면 상태 설명."""
    text = (describe_changes(result) if result['status'] == 'changed'
            else COMPARE_TEXT.get(result['status'], result['status']))
    if result.get('requires_review'):
        text = f'모델 보조: {text} (관리자 확인 필요)'
    return text


# review_server가 관리자 확인 결과를 저장하는 파일 (<capture_dir>/reviews.json, 키 "<회차>/<지점 ID>")
REVIEWS_FILE = 'reviews.json'


def visit_priority(last, review):
    """직전 회차 지점 기록과 관리자 확인 결과로 (순위, 이유). 순위가 작을수록 먼저 간다."""
    if review in ('조치 필요', '재촬영 필요'):
        return 1, f'관리자 표시: {review}'
    if last is None:
        return 4, None
    result = last.get('final') or last.get('compare') or {}
    verdict = result.get('verdict')
    if verdict == 'mismatch':
        rank, reason = 2, '미확인 재촬영 불일치'
    elif result.get('requires_review'):
        rank, reason = 2, '모델 보조 결과 미확인'
    elif result.get('status') == 'changed':
        rank, reason = 2, f'미확인 변화: {describe_changes(result)}' + (' 재촬영 확인' if verdict == 'confirmed' else '')
    elif last.get('status') != 'ok':
        rank, reason = 3, '최근 관측 ' + {'nav_failed': '이동 실패', 'no_image': '사진 없음'}.get(
            last.get('status'), last.get('status', '?'))
    elif result.get('status') in ('align_failed', 'no_plant', 'board_not_found', 'no_image', 'error', 'view_mismatch',
                                 'nav_failed', 'pose_unavailable', 'insufficient_motion') or verdict == 'recheck_failed':
        rank, reason = 3, '최근 관측 판정 실패'
    else:
        return 4, None
    # 관리자가 이미 살펴본 곳은 원래 루트 순서로 돌아간다
    if review in ('확인 완료', '오탐(변화 아님)'):
        return 4, None
    return rank, reason


def prioritize_stops(stops, history, reviews):
    """history는 오래된 순 (회차 이름, 결과). 동률이면 최신 회차·최신 관측부터.

    과거 변화는 unchanged로 해제하지 않는다. 관리자의 확인 완료/오탐은 해당
    기록과 그 이전 사건을 해제하지만, 그 뒤 새로 감지한 변화는 계속 우선한다.
    과거의 일시적인 촬영/정렬 실패는 최신 관측이 성공했으면 다시 우선하지 않는다.
    """
    ranked = []
    for index, stop in enumerate(stops):
        candidates, seen_latest, resolved = [], False, False
        for lap_index in range(len(history) - 1, -1, -1):
            lap, result = history[lap_index]
            records = result.get('stops', [])
            for observation_index in range(len(records) - 1, -1, -1):
                record = records[observation_index]
                if record['id'] != stop['id']:
                    continue
                review = reviews.get(f"{lap}/{stop['id']}", {}).get('result')
                if review in ('확인 완료', '오탐(변화 아님)'):
                    # 이 기록까지 확인했으므로 더 오래된 사건은 살펴보지 않는다.
                    resolved = True
                    break
                rank, reason = visit_priority(record, review)
                if rank < 3 or (rank == 3 and not seen_latest):
                    candidates.append((rank, -lap_index, -observation_index,
                                       f'{reason} ({lap})'))
                seen_latest = True
            if resolved:
                break
        if candidates:
            rank, recency, observation, reason = min(candidates, key=lambda c: c[:3])
        else:
            rank, recency, observation, reason = 4, 0, 0, None
        ranked.append((rank, recency, observation, index, stop, reason))
    ranked.sort(key=lambda row: row[:4])
    return [(stop, reason) for *_, stop, reason in ranked]


def needs_recheck(capture_status, comparison):
    """정면에서 변화가 감지된 경우에만 관리자용 옆면을 추가한다."""
    return capture_status == 'ok' and comparison.get('status') == 'changed'


VIEW_TEXT = {'front': '정면', 'left': '왼쪽', 'right': '오른쪽'}


def raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def load_route(route):
    """route가 .yaml 경로면 그 파일을, 아니면 plant_observer_gazebo/config/patrol_<route>.yaml을 읽는다."""
    if route.endswith('.yaml'):
        path = os.path.expanduser(route)
    else:
        path = os.path.join(
            get_package_share_directory('plant_observer_gazebo'), 'config', f'patrol_{route}.yaml')
    with open(path) as f:
        data = yaml.safe_load(f)
    return path, data


class PatrolManager(BasicNavigator):

    def __init__(self):
        super().__init__('patrol_manager')
        p = self.declare_parameter
        self.route = p('route', 'greenhouse').value
        self.route_path, route = load_route(self.route)
        self.capture_dir = os.path.expanduser(p('capture_dir', '~/plant_ws/captures').value)
        # 도착 직후 차체 흔들림이 가라앉을 때까지 기다린 뒤 찍는다
        self.settle_time = p('settle_time', 1.0).value
        self.image_timeout = p('image_timeout', 5.0).value
        self.laps = p('laps', 1).value
        # 이동 실패 시 같은 지점을 몇 번 더 시도할지. 일시적인 계획 실패는 대부분 재시도로 넘어간다
        self.nav_retries = p('nav_retries', 1).value
        self.capture_behavior_tree = os.path.join(
            get_package_share_directory('plant_observer_gazebo'), 'config', 'capture_bt.xml')
        self.side_capture_behavior_tree = os.path.join(
            get_package_share_directory('plant_observer_gazebo'), 'config', 'side_capture_bt.xml')
        self.capture_nav_timeout = p('capture_nav_timeout', 60.0).value
        self.nav_timeout = p('nav_timeout', 120.0).value
        self.startup_timeout = p('startup_timeout', 100.0).value
        if not math.isfinite(self.startup_timeout) or self.startup_timeout <= 0.0:
            raise ValueError('시작 제한 시간은 양수여야 합니다')
        if not all(math.isfinite(t) and t > 0.0 for t in (self.capture_nav_timeout, self.nav_timeout)):
            raise ValueError('이동 제한 시간은 양수여야 합니다')
        self.map_frame = p('map_frame', 'map').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.set_initial_pose = p('set_initial_pose', True).value
        # 이 거리 안이면 AMCL이 이미 home 근처를 알고 있다고 보고 위치를 다시 알려주지 않는다
        self.reset_pose_distance = p('reset_pose_distance', 0.3).value
        self.compare = p('compare', True).value
        from plant_observer_bringup.model_fallback import ModelFallback, DEFAULT_PYTHON, DEFAULT_WEIGHTS
        model_enabled = p('model_fallback', True).value
        model_python = p('model_python', DEFAULT_PYTHON).value
        model_weights = p('model_weights', DEFAULT_WEIGHTS).value
        model_timeout = p('model_timeout', 20.0).value
        self.model_fallback = (ModelFallback(model_python, model_weights, model_timeout)
                               if model_enabled and self.compare else None)
        # 열매 판정은 사과 검출 모델로 한다(기본 켜짐). 모델이 실패하면 색 규칙으로 판정한다
        from plant_observer_bringup.apple_detector import AppleDetector, DEFAULT_APPLE_WEIGHTS
        apple_weights = p('apple_weights', DEFAULT_APPLE_WEIGHTS).value
        self.apple_detector = (AppleDetector(model_python, apple_weights, model_timeout)
                               if p('apple_model', True).value and self.compare else None)
        self.capture_baseline = p('capture_baseline', False).value
        if self.capture_baseline and self.laps != 1:
            raise ValueError('기준 등록은 laps:=1로 한 번만 실행하세요')
        self.compare_to = p('compare_to', 'previous').value
        if self.compare_to not in ('previous', 'baseline'):
            raise ValueError('compare_to는 previous 또는 baseline이어야 합니다')
        # 변화가 의심되면 화분 중심을 축으로 이만큼 돌아간 자리에서 다시 찍는다 (관측 지점은 화분 앞 plant_distance)
        self.recheck = p('recheck', True).value
        # 15°는 반경 0.5m에서 13cm라 Nav2의 15cm 도착 허용 오차 안에 들어간다.
        self.recheck_angle = math.radians(p('recheck_angle_deg', route.get('recheck_angle_deg', 35.0)).value)
        self.plant_distance = p('plant_distance', 0.5).value
        # 도착 허용 오차 안쪽에서 멈춘 뒤 회전해도 화분에 너무 가까워지지 않도록 여유를 둔다.
        self.recheck_clearance = p('recheck_clearance', 0.15).value
        self.recheck_min_distance = p('recheck_min_distance', 0.10).value
        self.recheck_min_view_angle = math.radians(p('recheck_min_view_angle_deg', 10.0).value)
        self.odom_frame = p('odom_frame', 'odom').value
        self.pose_max_age = p('pose_max_age', 0.5).value
        if not (0.0 < self.recheck_angle < math.pi / 2
                and self.plant_distance > 0.0 and self.recheck_clearance >= 0.0
                and self.recheck_min_distance > 0.0
                and 0.0 < self.recheck_min_view_angle < self.recheck_angle
                and self.pose_max_age > 0.0):
            raise ValueError('재촬영 각도·최소 이동량·TF 최대 나이 설정을 확인하세요')
        # 직전 회차 판정·관리자 확인 결과로 방문 순서를 바꾼다
        self.prioritize = p('prioritize', True).value
        # 실물 U20CAM이 옆으로 달려 영상이 반시계 90° 돌아가 있다. 저장할 때 시계 방향으로 이만큼 돌린다
        # (0, 90, 180, 270). 주지 않으면(-1) 루트 파일의 image_rotation을 쓴다: 실물 루트 90, Gazebo 루트 0
        image_rotation = p('image_rotation', -1).value

        self.home, self.stops = route['home'], route['stops']
        if image_rotation < 0:
            image_rotation = route.get('image_rotation', 0)
        self.image_rotation = image_rotation % 360
        self.bridge = CvBridge()
        self.last_image = None
        # 시뮬레이션 브리지(reliable)와 usb_cam 어느 쪽과도 연결되도록 best effort로 받는다
        self.create_subscription(
            Image, p('image_topic', '/camera/image_raw').value, self.on_image, qos_profile_sensor_data)
        # 도착 위치는 TF(map→base_footprint)로 읽는다. amcl_pose는 로봇이 멈춰 있으면 갱신되지 않는다
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_image(self, msg):
        self.last_image = msg

    def wait_ready(self):
        # localizer='amcl'로 기다리면 BasicNavigator가 (0, 0, 0)을 /initialpose로 보낼 수 있다.
        # 위치는 AMCL에 맡기고, 여기서는 bt_navigator 활성화와 map→base_footprint TF만 기다린다
        deadline = time.monotonic() + self.startup_timeout
        client = self.create_client(GetState, 'bt_navigator/get_state')
        try:
            while time.monotonic() < deadline:
                if not client.wait_for_service(timeout_sec=.2):
                    continue
                future = client.call_async(GetState.Request())
                response_deadline = min(deadline, time.monotonic() + 2.)
                while not future.done() and time.monotonic() < response_deadline:
                    rclpy.spin_once(self, timeout_sec=.1)
                if future.done() and future.result().current_state.id == 3:
                    break
                future.cancel()
                rclpy.spin_once(self, timeout_sec=.1)
            else:
                raise TimeoutError('Nav2 활성화 제한 시간 초과: nav2_session 로그를 확인하세요')
        finally:
            self.destroy_client(client)
        self.check_capture_goal_checkers()
        last_log = 0.0
        while self.robot_pose() is None:
            if time.monotonic() >= deadline:
                raise TimeoutError('로봇 위치 TF 제한 시간 초과: 지도·초기 위치·센서를 확인하세요')
            if time.monotonic() - last_log > 5.0:
                self.get_logger().info(
                    '로봇 위치(map→base_footprint)를 기다리는 중. 안 잡히면 RViz 2D Pose Estimate로 알려주세요')
                last_log = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.set_initial_pose:
            self.localize_at_home()

    def check_capture_goal_checkers(self):
        """이전 설정으로 실행 중인 Nav2에 새 옆면 목표를 보내지 않는다."""
        client = self.create_client(GetParameters, 'controller_server/get_parameters')
        deadline = time.monotonic() + 6.
        try:
            while time.monotonic() < deadline:
                if not client.wait_for_service(timeout_sec=.2):
                    continue
                request = GetParameters.Request(names=['goal_checker_plugins'])
                future = client.call_async(request)
                response_deadline = min(deadline, time.monotonic() + 2.)
                while not future.done() and time.monotonic() < response_deadline:
                    rclpy.spin_once(self, timeout_sec=.1)
                if not future.done():
                    future.cancel()
                    continue
                values = future.result().values
                plugins = set(values[0].string_array_value) if values else set()
                if not {'capture_goal_checker', 'side_capture_goal_checker'} <= plugins:
                    raise RuntimeError('Nav2에 새 촬영 도착 설정이 없습니다. 터미널 2의 nav2_session을 '
                                       'Ctrl+C로 종료하고 다시 실행한 뒤 순찰을 시작하세요.')
                return
            raise TimeoutError('Nav2 촬영 설정 확인 시간 초과: controller_server 상태를 확인하세요')
        finally:
            self.destroy_client(client)

    def localize_at_home(self):
        """AMCL 위치가 home과 멀면 로봇이 home에 있다고 알려주고, 위치가 옮겨질 때까지 기다린다."""
        x, y, yaw = self.robot_pose()
        home = self.home
        if math.hypot(x - home['x'], y - home['y']) <= self.reset_pose_distance:
            return
        self.get_logger().info(
            f"AMCL 위치 ({x:.2f}, {y:.2f})가 home ({home['x']:.2f}, {home['y']:.2f})과 달라 "
            'home을 초기 위치로 보냅니다. 로봇이 home에 놓여 있어야 합니다')
        subscriber_deadline = time.monotonic() + 5.
        while self.initial_pose_pub.get_subscription_count() == 0:
            if time.monotonic() >= subscriber_deadline:
                raise TimeoutError('AMCL 초기 위치 구독자가 없습니다')
            rclpy.spin_once(self, timeout_sec=0.1)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.pose.pose = self.make_pose(home['x'], home['y'], home['yaw']).pose
        # 표준편차 0.1m / 0.1rad: 손으로 놓은 오차만큼 퍼뜨려서 AMCL이 스캔으로 다듬게 한다
        msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[35] = 0.01
        self.initial_pose_pub.publish(msg)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            pose = self.robot_pose()
            if pose and math.hypot(pose[0] - home['x'], pose[1] - home['y']) <= self.reset_pose_distance:
                self.get_logger().info(f'AMCL 위치 ({pose[0]:.2f}, {pose[1]:.2f})로 옮겨짐')
                return
        self.get_logger().warn('AMCL 위치가 home으로 옮겨지지 않았습니다. RViz 2D Pose Estimate로 확인하세요')

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_from_quaternion(t.transform.rotation)

    def fresh_pose(self, frame, wait=1.0):
        """재촬영 검증에는 오래된 TF를 쓰지 않는다. odom으로 실제 이동도 교차 확인한다.

        TF는 노드를 돌려야 들어온다. 사진 비교(약 1초) 직후에는 버퍼가 그만큼 묵어 있으므로
        wait초 동안 돌리며 새 TF를 기다린다 (실물 2026-10-30_193421: 비교 직후 바로 읽어 재촬영 실패).
        """
        deadline = time.monotonic() + wait
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
            except TransformException:
                t = None
            if t is not None:
                age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
                pose = (t.transform.translation.x, t.transform.translation.y,
                        yaw_from_quaternion(t.transform.rotation))
                if abs(age) <= self.pose_max_age and all(math.isfinite(v) for v in pose):
                    return pose
            if time.monotonic() >= deadline:
                return None

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def go(self, target):
        precise = 'id' in target or 'view_id' in target
        deadline = time.monotonic() + (self.capture_nav_timeout if precise else self.nav_timeout)
        for attempt in range(1 + self.nav_retries):
            if attempt:
                self.get_logger().warn(f'이동 실패, 다시 시도 ({attempt}/{self.nav_retries})')
            tree = (self.side_capture_behavior_tree if target.get('view_id') in ('right', 'left')
                    else self.capture_behavior_tree if precise else '')
            if not self.goToPose(self.make_pose(target['x'], target['y'], target['yaw']), behavior_tree=tree):
                continue
            while not self.isTaskComplete():
                if time.monotonic() >= deadline:
                    self.get_logger().warn('이동 완료 응답 제한 시간 초과: 이동을 취소하고 실패로 기록합니다')
                    if not self.cancel_navigation():
                        raise RuntimeError('Nav2 취소 응답도 없어 순찰을 중단합니다. 저장된 중간 결과를 확인하세요')
                    return TaskResult.FAILED
            if self.getResult() == TaskResult.SUCCEEDED:
                return TaskResult.SUCCEEDED
        return TaskResult.FAILED

    def cancel_navigation(self):
        """취소 응답도 무한정 기다리지 않는다. 미확인 상태에서 다음 목표를 보내지 않는다."""
        if self.goal_handle is None:
            return True
        try:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            return future.done() and future.result().return_code in (0, 3)
        except Exception as exc:
            self.get_logger().error(f'이동 취소 확인 실패: {exc!r}')
            return False

    def spin_for(self, seconds):
        """노드 시계(시뮬레이션이면 sim time) 기준으로 seconds 동안 콜백을 돌린다."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        deadline = time.monotonic() + max(5., seconds * 5.)
        while self.get_clock().now() < end:
            if time.monotonic() >= deadline:
                raise TimeoutError('ROS 시계가 진행하지 않습니다. 시뮬레이터 /clock을 확인하세요')
            rclpy.spin_once(self, timeout_sec=0.05)

    def capture(self, path):
        """멈춘 뒤에 찍힌 사진만 저장한다. image_timeout 안에 새 사진이 없으면 False."""
        self.spin_for(self.settle_time)
        after = self.get_clock().now()
        deadline = time.monotonic() + self.image_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            msg = self.last_image
            if msg is not None and Time.from_msg(msg.header.stamp) >= after:
                image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                rotate = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
                          270: cv2.ROTATE_90_COUNTERCLOCKWISE}.get(self.image_rotation)
                if rotate is not None:
                    image = cv2.rotate(image, rotate)
                try:
                    saved = cv2.imwrite(path, image)
                except (cv2.error, OSError) as exc:
                    self.get_logger().error(f'사진 저장 실패: {path}: {exc}')
                    return False
                if not saved:
                    self.get_logger().error(f'사진 저장 실패: {path}')
                return saved
        return False

    def run_lap(self, lap):
        if self.capture_baseline:
            return self.run_baseline_lap(lap)
        started = datetime.now()
        folder = os.path.join(self.capture_dir, started.strftime('%Y-%m-%d_%H%M%S'))
        while os.path.exists(folder):
            started += timedelta(seconds=1)
            folder = os.path.join(self.capture_dir, started.strftime('%Y-%m-%d_%H%M%S'))
        os.makedirs(folder)
        reference = previous_lap(self.capture_dir, self.route, folder)
        if self.compare_to == 'baseline':
            marker = baseline_file(self.capture_dir, self.route)
            if os.path.isfile(marker):
                with open(marker) as f:
                    reference = os.path.basename(os.path.normpath(f.read().strip()))
        observed = observed_stops(self.capture_dir, self.route, folder)
        order = self.plan_order(folder)
        summary = {'route': self.route, 'route_file': self.route_path, 'image_rotation': self.image_rotation,
                   'compare_to': self.compare_to, 'reference_lap': reference,
                   'capture_policy': 'front_compare_sides_for_admin', 'lap': lap,
                   'started': started.isoformat(timespec='seconds'), 'home': 'pending',
                   'visit_order': [{'id': stop['id'], 'reason': reason} for stop, reason in order],
                   'stops': []}

        def save():
            with open(os.path.join(folder, 'result.json.tmp'), 'w') as f:
                json.dump(summary, f, ensure_ascii=False, indent=2)
            os.replace(os.path.join(folder, 'result.json.tmp'), os.path.join(folder, 'result.json'))

        label = '직전 완료 회차' if self.compare_to == 'previous' else '고정 기준 회차'
        self.get_logger().info(f'{lap}회차 시작: {label} {reference or "없음(첫 촬영)"}의 정면과 비교 → {folder}')
        self.get_logger().info('방문 순서: ' + ' → '.join(
            f"{stop['id']}({reason})" if reason else stop['id'] for stop, reason in order))
        save()
        for stop, _ in order:
            sid = stop['id']
            entry = {'id': sid, 'status': 'pending', 'views': {}, 'initial_observation': sid not in observed}
            summary['stops'].append(entry)
            save()
            self.get_logger().info(f'{sid}: 정면 촬영')
            front = self.capture_view(folder, stop, {**stop, 'view_id': 'front'})
            entry['views']['front'] = front
            entry.update({k: front[k] for k in ('status', 'image', 'pose', 'target') if k in front})
            entry['observed_at'] = datetime.now().isoformat(timespec='microseconds')
            if front.get('pose'):
                pose = front['pose']
                entry['error_xy_m'] = round(math.hypot(pose['x'] - stop['x'], pose['y'] - stop['y']), 3)
                entry['error_yaw_deg'] = round(math.degrees(angle_diff(pose['yaw'], stop['yaw'])), 1)
            save()
            if front['status'] != 'ok':
                self.get_logger().warn(f'{sid}: 정면 촬영 실패({front["status"]}), 확인 필요')
                continue
            if self.compare:
                comparison = (self.compare_stop(folder, sid, front['image'], view_id='front',
                                                reference_lap=reference, current_pose=front['pose'])
                              if reference else {'status': 'no_reference', 'view_id': 'front'})
                front['compare'] = entry['compare'] = comparison
                # 판정은 정면 사진으로만 결정한다. 관리자용 옆면의 성공/실패와 무관하다.
                entry['final'] = {k: comparison[k] for k in (
                    'status', 'changes', 'change', 'count', 'analysis_source', 'requires_review', 'primary_status')
                    if k in comparison}
                entry['final']['source'] = 'front'
                self.log_result(f'{sid}: {describe_change(comparison)}', comparison['status'] == 'changed')
            save()
            changed = needs_recheck(front['status'], entry.get('compare', {}))
            if entry['initial_observation'] or (changed and self.recheck):
                key = 'overview' if entry['initial_observation'] else 'recheck'
                entry[key] = {'purpose': 'admin_review', 'status': 'pending', 'views': {}}
                save()

                def save_side(view_id, view):
                    entry['views'][view_id] = view
                    entry[key]['views'][view_id] = view
                    save()

                self.get_logger().info(f'{sid}: {"첫 촬영" if entry["initial_observation"] else "정면 변화 감지"}'
                                       ' — 관리자 확인용 왼쪽·오른쪽 촬영')
                entry[key] = self.recheck_stop(folder, stop, on_view=save_side)
            elif changed:
                entry['recheck'] = {'purpose': 'admin_review', 'status': 'disabled'}
            save()
        self.get_logger().info('출발점으로 복귀')
        summary['home'] = 'ok' if self.go(self.home) == TaskResult.SUCCEEDED else 'nav_failed'
        summary['finished'] = datetime.now().isoformat(timespec='seconds')
        save()
        self.get_logger().info(f'{lap}회차 끝: 출발점 복귀 {summary["home"]}')
        return summary

    def compare_stop(self, folder, stop_id, image, out_name=None, view_id='front',
                     reference_lap=None, current_pose=None):
        """방금 찍은 사진을 기준 사진과 비교해 결과를 로그로 남기고 돌려준다. 비교가 죽어도 순찰은 계속한다."""
        try:
            result = compare_with_baseline(
                self.capture_dir, self.route, folder, stop_id, os.path.join(folder, image), out_name=out_name,
                use_baseline=self.compare_to == 'baseline', view_id=view_id,
                reference_lap=reference_lap, current_pose=current_pose,
                model_fallback=getattr(self, 'model_fallback', None),
                apple_detector=getattr(self, 'apple_detector', None))
        except Exception as e:  # noqa: BLE001 — 사진 비교 오류로 로봇이 멈추면 안 된다
            self.get_logger().error(f'{stop_id}: 비교 중 오류 {e!r}')
            return {'status': 'error', 'error': repr(e)}
        detail = f" (기준 회차 {result['reference_lap']})" if 'reference_lap' in result else ''
        self.log_result(f'{stop_id}: {describe_change(result)}{detail}', result['status'] == 'changed')
        return result

    def log_result(self, message, changed):
        """변화는 warn, 아니면 info. rclpy는 한 줄에서 심각도를 바꿔 부르면 ValueError를 내서 호출 줄을 나눈다."""
        if changed:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def recheck_poses(self, stop):
        """중앙 촬영 다음은 오른쪽→왼쪽. 위치는 루트 정면을 기준으로 고정한다."""
        d, yaw = self.plant_distance, stop['yaw']
        cx, cy = stop['x'] + d * math.cos(yaw), stop['y'] + d * math.sin(yaw)
        radius = d + self.recheck_clearance
        poses = []
        for view_id, sign in (('right', 1), ('left', -1)):
            turned = yaw + sign * self.recheck_angle
            target = {'x': cx - radius * math.cos(turned),
                      'y': cy - radius * math.sin(turned), 'yaw': turned}
            # 좁은 장소는 루트 YAML의 views.left/right로 고정 위치를 지정할 수 있다.
            target.update(stop.get('views', {}).get(view_id, {}))
            if not all(math.isfinite(target[k]) for k in ('x', 'y', 'yaw')):
                raise ValueError(f'{stop["id"]} {view_id}: 잘못된 촬영 위치')
            poses.append({**target, 'view_id': view_id})
        return poses

    def capture_view(self, folder, stop, target, origin=None, origin_odom=None, suffix=''):
        """한 방향으로 이동·촬영. 실제 자세와 이동 검증 결과를 사진과 함께 기록한다."""
        view_id = target['view_id']
        record = {'view_id': view_id, 'target': {k: target[k] for k in ('x', 'y', 'yaw')}}
        if view_id != 'front':
            record['purpose'] = 'admin_review'
        navigation_started = time.monotonic()
        navigation_result = self.go(target)
        record['navigation_elapsed_s'] = round(time.monotonic() - navigation_started, 2)
        if navigation_result != TaskResult.SUCCEEDED:
            return {**record, 'status': 'nav_failed'}
        if origin is not None:
            enough, measured = self.recheck_motion(stop, origin, origin_odom)
            record.update(measured)
            if not enough:
                return record
        image = f"{stop['id']}{'' if view_id == 'front' else '_' + view_id}{suffix}.png"
        if not self.capture(os.path.join(folder, image)):
            return {**record, 'status': 'no_image'}
        pose, odom = self.fresh_pose(self.map_frame), self.fresh_pose(self.odom_frame)
        if pose is None or odom is None:
            return {**record, 'status': 'pose_unavailable'}
        record.update(status='ok', image=image, pose=dict(zip(('x', 'y', 'yaw'), pose)),
                      odom_pose=dict(zip(('x', 'y', 'yaw'), odom)))
        if origin is not None:
            _, measured = self.recheck_motion(stop, origin, origin_odom)
            record.update(measured)
        record['error_xy_m'] = round(math.hypot(pose[0] - target['x'], pose[1] - target['y']), 3)
        record['error_yaw_deg'] = round(math.degrees(angle_diff(pose[2], target['yaw'])), 1)
        self.get_logger().info(f"{stop['id']} {VIEW_TEXT[view_id]}: 이동 {record['navigation_elapsed_s']}초, "
                               f"위치 오차 {record['error_xy_m']}m / 방향 오차 {record['error_yaw_deg']}°")
        return record

    def run_baseline_lap(self, lap):
        """정상 상태에서 명시적으로 실행하는 3방향 등록. 실패 시 기존 기준은 유지한다."""
        started = datetime.now()
        folder = os.path.join(self.capture_dir, started.strftime('%Y-%m-%d_%H%M%S'))
        while os.path.exists(folder):
            started += timedelta(seconds=1)
            folder = os.path.join(self.capture_dir, started.strftime('%Y-%m-%d_%H%M%S'))
        os.makedirs(folder)
        summary = {'route': self.route, 'route_file': self.route_path, 'lap': lap, 'mode': 'baseline',
                   'image_rotation': self.image_rotation, 'started': started.isoformat(timespec='seconds'),
                   'home': 'pending', 'baseline_complete': False, 'stops': []}

        def save():
            temporary = os.path.join(folder, 'result.json.tmp')
            with open(temporary, 'w') as f:
                json.dump(summary, f, ensure_ascii=False, indent=2)
            os.replace(temporary, os.path.join(folder, 'result.json'))

        save()
        self.get_logger().info('3방향 기준 등록: 정상 상태의 식물을 정면·왼쪽·오른쪽에서 촬영합니다')
        for stop in self.stops:
            front = self.capture_view(folder, stop, {**stop, 'view_id': 'front'})
            entry = {'id': stop['id'], **{k: front[k] for k in ('status', 'image', 'pose', 'target') if k in front},
                     'views': {'front': front}, 'compare': {'status': 'no_reference'}}
            summary['stops'].append(entry)
            save()
            if front['status'] != 'ok':
                continue
            origin = tuple(front['pose'][k] for k in ('x', 'y', 'yaw'))
            origin_odom = tuple(front['odom_pose'][k] for k in ('x', 'y', 'yaw'))
            for target in self.recheck_poses(stop):
                self.get_logger().info(f"{stop['id']}: {target['view_id']} 기준 촬영")
                entry['views'][target['view_id']] = self.capture_view(folder, stop, target, origin, origin_odom)
                save()
        summary['home'] = 'ok' if self.go(self.home) == TaskResult.SUCCEEDED else 'nav_failed'
        summary['finished'] = datetime.now().isoformat(timespec='seconds')
        summary['baseline_complete'] = summary['home'] == 'ok' and bool(self.stops) and all(
            entry['views'].get(view, {}).get('status') == 'ok'
            for entry in summary['stops'] for view in ('front', 'left', 'right'))
        save()
        if summary['baseline_complete']:
            path = baseline_file(self.capture_dir, self.route)
            with open(path + '.tmp', 'w') as f:
                f.write(os.path.basename(folder) + '\n')
            os.replace(path + '.tmp', path)
            self.get_logger().info(f'3방향 기준 등록 완료: {folder}')
        else:
            self.get_logger().warn(f'기준 등록 미완료. 기존 기준을 유지합니다: {folder}')
        return summary

    def recheck_motion(self, stop, origin, origin_odom):
        pose, odom = self.fresh_pose(self.map_frame), self.fresh_pose(self.odom_frame)
        if pose is None or odom is None:
            return False, {'status': 'pose_unavailable'}
        cx = stop['x'] + self.plant_distance * math.cos(stop['yaw'])
        cy = stop['y'] + self.plant_distance * math.sin(stop['yaw'])
        distance = math.hypot(odom[0] - origin_odom[0], odom[1] - origin_odom[1])
        angle = abs(angle_diff(math.atan2(pose[1] - cy, pose[0] - cx),
                               math.atan2(origin[1] - cy, origin[0] - cx)))
        enough = distance >= self.recheck_min_distance and angle >= self.recheck_min_view_angle
        return enough, {'status': 'ok' if enough else 'insufficient_motion',
                        'pose': dict(zip(('x', 'y', 'yaw'), pose)),
                        'odom_pose': dict(zip(('x', 'y', 'yaw'), odom)),
                        'movement_m': round(distance, 3),
                        'viewpoint_change_deg': round(math.degrees(angle), 1)}

    def recheck_stop(self, folder, stop, on_view=None):
        """관리자 확인용 왼쪽·오른쪽 사진. 옆면 비교와 자동 변화 확정은 하지 않는다."""
        sid = stop['id']
        origin, origin_odom = self.fresh_pose(self.map_frame), self.fresh_pose(self.odom_frame)
        common = {'purpose': 'admin_review', 'attempts': [], 'views': {}}
        if origin is None or origin_odom is None:
            self.get_logger().warn(f'{sid}: 추가 촬영 위치 TF를 확인할 수 없습니다')
            return {**common, 'status': 'pose_unavailable'}
        if math.hypot(origin[0] - stop['x'] - self.plant_distance * math.cos(stop['yaw']),
                      origin[1] - stop['y'] - self.plant_distance * math.sin(stop['yaw'])) < 1e-6:
            return {**common, 'status': 'invalid_geometry'}
        common.update(origin_pose=dict(zip(('x', 'y', 'yaw'), origin)),
                      origin_odom_pose=dict(zip(('x', 'y', 'yaw'), origin_odom)))
        captured = []
        for target in self.recheck_poses(stop):
            view_id = target['view_id']
            self.get_logger().info(f'{sid}: 관리자 확인용 {VIEW_TEXT[view_id]} 촬영')
            view = self.capture_view(folder, stop, target, origin, origin_odom)
            view['purpose'] = 'admin_review'
            common['attempts'].append(dict(view))
            common['views'][view_id] = view
            if on_view:
                on_view(view_id, view)
            if view['status'] == 'ok':
                captured.append(view)
            else:
                self.get_logger().warn(f'{sid}: {VIEW_TEXT[view_id]} 추가 촬영 실패({view["status"]})')
        status = ('ok' if len(captured) == 2 else 'incomplete' if captured
                  else common['attempts'][-1]['status'] if common['attempts'] else 'invalid_geometry')
        # 대표 이미지 필드는 이전 업로드 스키마와 호환한다. compare/verdict는 생성하지 않는다.
        return {**common, **(captured[0] if captured else {}), 'status': status}

    def history_laps(self, folder):
        """현재 회차보다 오래된 같은 루트의 결과. 손상된 회차는 경고 후 건너뛴다."""
        current = os.path.basename(folder)
        history = []
        for lap in sorted(os.listdir(self.capture_dir)):
            path = os.path.join(self.capture_dir, lap, 'result.json')
            if lap >= current or not os.path.isfile(path):
                continue
            try:
                with open(path) as f:
                    result = json.load(f)
                if not isinstance(result, dict) or not isinstance(result.get('stops'), list):
                    raise ValueError('잘못된 회차 구조')
                if any(not isinstance(s, dict) or 'id' not in s for s in result['stops']):
                    raise ValueError('잘못된 지점 구조')
            except (OSError, ValueError) as exc:
                self.get_logger().warn(f'방문 순서에서 손상된 회차 제외: {lap}: {exc}')
                continue
            if result.get('route') == self.route and result.get('mode') != 'baseline':
                history.append((lap, result))
        return history

    def plan_order(self, folder):
        """관리자 요청, 미확인 변화의 최신 감지 순, 최근 실패, 기본 순서."""
        if not self.prioritize:
            return [(stop, None) for stop in self.stops]
        history = self.history_laps(folder)
        reviews = {}
        path = os.path.join(self.capture_dir, REVIEWS_FILE)
        if os.path.isfile(path):
            try:
                with open(path) as f:
                    reviews = json.load(f)
                if not isinstance(reviews, dict) or any(not isinstance(v, dict) for v in reviews.values()):
                    raise ValueError('잘못된 관리자 확인 구조')
            except (OSError, ValueError) as exc:
                self.get_logger().warn(f'관리자 확인 기록을 읽지 못해 감지 이력으로 순서 결정: {exc}')
                reviews = {}
        return prioritize_stops(self.stops, history, reviews)

    def run(self):
        self.wait_ready()
        self.get_logger().info(f'루트 {self.route_path}, {self.laps}회 순찰, 사진 회전 {self.image_rotation}°')
        if getattr(self, 'model_fallback', None) is not None:
            self.get_logger().info('정면 비교 실패 시 모델 보조 활성화 — 모든 보조 결과는 관리자 확인 필요')
        detector = getattr(self, 'apple_detector', None)
        if detector is not None:
            if detector.available():
                self.get_logger().info(f'열매 판정: 사과 모델 {detector.weights}')
            else:
                self.get_logger().warn(f'사과 모델 또는 실행 환경 없음({detector.weights}) — 열매는 색 규칙으로 판정')
        for lap in range(1, self.laps + 1):
            summary = self.run_lap(lap)
            # All modes return only after final results and any baseline marker are saved.
            if summary.get('finished'):
                if notify_completion(self.capture_dir):
                    self.get_logger().info(f'{lap}회차 저장 완료: 자동 업로더에 즉시 전송 요청')
                else:
                    self.get_logger().warn(
                        '즉시 업로드 알림을 보내지 못했습니다. 저장된 기록은 자동 업로더가 재확인합니다')


def main():
    # rclpy 기본 SIGINT 처리는 context를 먼저 닫아 버려서 Nav2 목표를 취소할 수 없다.
    # 파이썬 기본 동작(KeyboardInterrupt)으로 받아 목표를 취소해 로봇을 세운 뒤 끝낸다
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGHUP, raise_keyboard_interrupt)
    node = None
    try:
        node = PatrolManager()
        node.run()
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('중단: 진행 중인 이동을 취소합니다')
            node.cancel_navigation()
    except Exception:
        if node is not None:
            node.cancel_navigation()
        raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            if getattr(node, 'model_fallback', None) is not None:
                node.model_fallback.close()
            if getattr(node, 'apple_detector', None) is not None:
                node.apple_detector.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
