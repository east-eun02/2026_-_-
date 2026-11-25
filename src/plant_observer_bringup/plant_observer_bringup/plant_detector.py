"""사과·잎 검출 모델(YOLO)을 ROS 밖 모델 실행 환경의 워커 하나에서 함께 돌린다.

정면 비교마다 기준 사진과 이번 사진의 사과·잎을 찾고, change_compare가 정렬 결과로 두 사진의
검출을 짝지어 열매 생성·손상·탈락과 잎 탈락을 정한다. 잎 색 변화는 잎 모델이 갈변 잎을 거의 못
잡아서(2026-11-25 실물 사진) 계속 색 규칙으로 판정한다. 실패하면 해당 판정은 색 규칙을 그대로 쓴다.
"""
import os
from pathlib import Path
import time

from plant_observer_bringup.model_fallback import DEFAULT_PYTHON, WorkerProcess

MODELS = Path.home() / 'plant_ws/models'
DEFAULT_APPLE_WEIGHTS = str(MODELS / 'apple_yolo26n_aug_v1.pt')
DEFAULT_LEAF_WEIGHTS = str(MODELS / 'leaf_yolo26n_aug_v5.pt')
# 팀원 학습 입력 크기
INPUT_SIZE = {'apple': 640, 'leaf': 512}
# 모델 클래스 이름 → 내부 이름
CLASS_NAMES = {
    'apple': {'normal_apple': 'normal_apple', 'abnormal_apple': 'abnormal_apple'},
    'leaf': {'정상 잎': 'normal_leaf', '비정상 잎': 'abnormal_leaf'},
}
# 클래스별 최소 확신도. 실물 사진(2026-10-31~11-04)에서
#  - 갈변 잎이 normal_apple 0.45로, 상한 사과가 abnormal_apple 0.33·0.35로 나왔다
#  - 초록 잎은 0.36~0.79, 화분 흙이 정상 잎 0.36·0.47, 사과가 정상 잎 0.60으로 나왔다(색·사과 박스로 거른다)
#  - 비정상 잎은 0.24짜리 천장 전체 박스 같은 엉터리가 나와 판정에 쓰지 않고 개수만 남긴다
MIN_CONFIDENCE = {'normal_apple': .5, 'abnormal_apple': .3, 'normal_leaf': .35, 'abnormal_leaf': .5}
# 한 물체에 두 클래스 박스가 겹쳐 나온다(같은 사과 normal 0.67 / abnormal 0.65). 확신도 높은 쪽만 남긴다
DUPLICATE_IOU = .5


def iou(a, b):
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.
    inter = w * h
    return inter / ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter)


def select(raw, model='apple'):
    """내부 이름으로 바꾸고, 클래스별 확신도를 넘는 박스 중 겹친 것은 확신도 높은 하나만 남긴다."""
    names = CLASS_NAMES[model]
    passed = sorted(({**d, 'class': names[d['class']]} for d in raw
                     if d['class'] in names and d['confidence'] >= MIN_CONFIDENCE[names[d['class']]]),
                    key=lambda d: d['confidence'], reverse=True)
    kept = []
    for detection in passed:
        if all(iou(detection['bbox'], other['bbox']) <= DUPLICATE_IOU for other in kept):
            kept.append(detection)
    return kept


def select_apples(raw):
    return select(raw, 'apple')


class PlantDetector(WorkerProcess):
    """One private worker per patrol process for the apple and leaf models. Failure falls back to color rules."""

    WORKER = 'detector_worker.py'

    def __init__(self, python=DEFAULT_PYTHON, apple_weights=DEFAULT_APPLE_WEIGHTS,
                 leaf_weights=DEFAULT_LEAF_WEIGHTS, timeout=20., worker=None):
        super().__init__(python, apple_weights or '', timeout, worker)
        self.paths = {name: os.path.expanduser(path)
                      for name, path in (('apple', apple_weights), ('leaf', leaf_weights)) if path}

    def models(self):
        """가중치 파일이 있는 모델 {이름: 경로}."""
        return {name: path for name, path in self.paths.items() if os.path.isfile(path)}

    def available(self):
        return os.path.isfile(self.python) and bool(self.models())

    def _command(self):
        specs = [arg for name, path in self.models().items()
                 for arg in ('--model', f'{name}={path}@{INPUT_SIZE[name]}')]
        return [self.python, self.worker, *specs, '--threads', '2']

    def detect(self, before, after):
        """({'apple': (기준, 이번) 또는 None, 'leaf': ...}, 기록). 실패해도 예외를 내지 않는다."""
        start = time.monotonic()
        found = {name: None for name in self.paths}
        try:
            response = self.request({'images': [os.path.abspath(before), os.path.abspath(after)]},
                                    start + self.timeout)
            for name, raw in response['detections'].items():
                if name not in CLASS_NAMES or len(raw) != 2:
                    raise ValueError('검출 모델 응답 형식 불일치')
                found[name] = tuple(select(images, name) for images in raw)
            missing = sorted(name for name in self.paths if found[name] is None)
            trace = {'status': 'used', 'models': response['models'], 'inference_ms': response.get('inference_ms'),
                     'total_wall_ms': round((time.monotonic()-start)*1000, 2)}
            if missing:
                trace['missing'] = missing
            return found, trace
        except Exception as exc:
            return {name: None for name in self.paths}, {
                'status': 'failed', 'error': f'{type(exc).__name__}: {exc}',
                'total_wall_ms': round((time.monotonic()-start)*1000, 2)}
