"""관찰 기록 화면의 '실시간 보기': 지도 위 로봇·라이다·경로와 카메라 영상을 휴대폰 브라우저로 보여 준다.

RViz를 띄우지 않고도 노트북 덮개를 닫은 채 같은 Wi-Fi(핫스팟)에서 볼 수 있게 한다.
여기에는 ROS 메시지를 브라우저용 값으로 바꾸는 계산만 둔다(ROS 없이 시험 가능).
토픽 구독은 robot_control.RosMonitor가 한다.
"""
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

# 라이다 점은 이만큼만 보낸다 (휴대폰 화면에서는 이 정도면 충분하고 0.5초마다 받아도 가볍다)
MAX_SCAN_POINTS = 360
MAX_PATH_POINTS = 200


def yaw_from_quaternion(q):
    return math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z))


def pose2d(transform):
    """geometry_msgs/Transform → (x, y, yaw)."""
    return transform.translation.x, transform.translation.y, yaw_from_quaternion(transform.rotation)


def map_png(width, height, data):
    """OccupancyGrid 값(-1 모름, 0 빈칸 ~ 100 벽)을 RViz처럼 회색·흰색·검정 PNG로.

    격자 0행은 지도 아래쪽(y 최소)이라 위아래를 뒤집어 화면 위쪽이 +y가 되게 한다.
    """
    grid = np.asarray(data, np.int16).reshape(height, width)
    image = np.where(grid < 0, 205, 254 - np.clip(grid, 0, 100) * 254 // 100).astype(np.uint8)
    ok, encoded = cv2.imencode('.png', image[::-1])
    return encoded.tobytes() if ok else None


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, sensor_pose):
    """LaserScan 거리값을 지도 좌표 [[x, y], ...]로. sensor_pose는 지도에서 라이다의 (x, y, yaw)."""
    ranges = np.asarray(ranges, np.float32)
    angles = angle_min + angle_increment * np.arange(len(ranges), dtype=np.float32)
    valid = np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max)
    ranges, angles = ranges[valid], angles[valid]
    if len(ranges) > MAX_SCAN_POINTS:
        keep = np.linspace(0, len(ranges) - 1, MAX_SCAN_POINTS).astype(int)
        ranges, angles = ranges[keep], angles[keep]
    x, y, yaw = sensor_pose
    xs = x + ranges * np.cos(angles + yaw)
    ys = y + ranges * np.sin(angles + yaw)
    return np.round(np.stack([xs, ys], axis=1), 3).tolist()


def path_points(xy):
    """경로 좌표 목록을 화면에 그릴 만큼만 고르게 줄인다 (끝점은 남긴다)."""
    if len(xy) > MAX_PATH_POINTS:
        keep = sorted(set(np.linspace(0, len(xy) - 1, MAX_PATH_POINTS).astype(int).tolist()))
        xy = [xy[i] for i in keep]
    return [[round(x, 3), round(y, 3)] for x, y in xy]


def load_route(route):
    """순찰 루트 파일에서 출발점·관측 지점·사진 회전만 읽는다. 없거나 깨졌으면 None."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('plant_observer_gazebo')) / 'config' / f'patrol_{route}.yaml'
        config = yaml.safe_load(path.read_text())
        return {'name': route, 'home': config['home'], 'image_rotation': config.get('image_rotation', 0),
                'stops': [{k: stop[k] for k in ('id', 'x', 'y', 'yaw')} for stop in config['stops']]}
    except Exception:
        return None
