"""사과 검출 모델(YOLO, normal_apple / abnormal_apple)을 ROS 밖 모델 실행 환경에서 돌린다.

정면 비교마다 기준 사진과 이번 사진의 사과를 찾고, change_compare가 정렬 결과로 두 사진의 사과를
짝지어 열매 생성·열매 손상·열매 탈락을 정한다. 실패하면 기존 색 기반 열매 판정을 그대로 쓴다.
"""
import os
from pathlib import Path
import time

from plant_observer_bringup.model_fallback import DEFAULT_PYTHON, WorkerProcess

DEFAULT_APPLE_WEIGHTS = str(Path.home() / 'plant_ws/models/apple_yolo26n_aug_v1.pt')
# 클래스별 최소 확신도. 실물 사진(2026-10-31~11-04)에서 갈변 잎이 normal_apple 0.45로,
# 상한 사과가 abnormal_apple 0.33·0.35로 나왔다. 상한 사과는 놓치지 않게 낮게, 정상 사과는 오탐을 막게 높게 둔다
MIN_CONFIDENCE = {'normal_apple': .5, 'abnormal_apple': .3}
# 한 사과에 normal·abnormal 박스가 겹쳐 나온다(같은 사과 normal 0.67 / abnormal 0.65). 확신도 높은 쪽만 남긴다
DUPLICATE_IOU = .5


def iou(a, b):
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.
    inter = w * h
    return inter / ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter)


def select_apples(raw):
    """클래스별 확신도를 넘는 박스만 남기고, 겹친 박스는 클래스와 상관없이 확신도 높은 하나만 남긴다."""
    passed = sorted((d for d in raw if d['confidence'] >= MIN_CONFIDENCE.get(d['class'], 1.1)),
                    key=lambda d: d['confidence'], reverse=True)
    kept = []
    for detection in passed:
        if all(iou(detection['bbox'], other['bbox']) <= DUPLICATE_IOU for other in kept):
            kept.append(detection)
    return kept


class AppleDetector(WorkerProcess):
    """One private apple worker per patrol process. Failure falls back to color-based fruit rules."""

    WORKER = 'apple_worker.py'

    def __init__(self, python=DEFAULT_PYTHON, weights=DEFAULT_APPLE_WEIGHTS, timeout=20., worker=None):
        super().__init__(python, weights, timeout, worker)

    def available(self):
        return os.path.isfile(self.python) and os.path.isfile(self.weights)

    def detect(self, before, after):
        """(기준 사과, 이번 사과, 기록). 실패하면 (None, None, 실패 기록)이고 예외를 내지 않는다."""
        start = time.monotonic()
        try:
            response = self.request({'images': [os.path.abspath(before), os.path.abspath(after)]},
                                    start + self.timeout)
            raw = response['detections']
            if len(raw) != 2:
                raise ValueError('사과 모델 응답 형식 불일치')
            apples = [select_apples(found) for found in raw]
            trace = {'status': 'used', 'model': response['model'],
                     'inference_ms': response.get('inference_ms'),
                     'total_wall_ms': round((time.monotonic()-start)*1000, 2)}
            return apples[0], apples[1], trace
        except Exception as exc:
            return None, None, {'status': 'failed', 'error': f'{type(exc).__name__}: {exc}',
                                'total_wall_ms': round((time.monotonic()-start)*1000, 2)}
