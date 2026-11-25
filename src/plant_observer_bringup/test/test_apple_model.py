"""Apple detection model: box selection, apple matching, coordinate mapping, fallback, real model."""
import json
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest

from plant_observer_bringup import change_compare as cc
from plant_observer_bringup.apple_detector import AppleDetector, DEFAULT_APPLE_WEIGHTS, select_apples
from plant_observer_bringup.model_fallback import DEFAULT_PYTHON

FIXTURES = Path(__file__).parent/'fixtures/vision'
ROUTE = Path(__file__).parents[2]/'plant_observer_gazebo/config/patrol_demo_real3.yaml'
CAPTURES = Path('/home/eun/plant_ws/captures_manual_demo_real3')


def apple(cls, conf, box):
    return {'class': cls, 'confidence': conf, 'bbox': list(box)}


# --- select_apples -------------------------------------------------------------------------------

def test_overlapping_normal_and_abnormal_keep_more_confident_one():
    # 실물 2026-11-03_212306 P01: 같은 사과에 normal 0.67 / abnormal 0.65
    kept = select_apples([apple('normal_apple', .67, (0, 0, 100, 100)),
                          apple('abnormal_apple', .65, (2, 2, 101, 99))])
    assert [a['class'] for a in kept] == ['normal_apple']


def test_low_confidence_normal_is_dropped_but_abnormal_kept():
    # 실물 2026-10-31_021854 P00: 갈변 잎이 normal 0.45, 상한 사과는 0.33~0.35로 나온다
    kept = select_apples([apple('normal_apple', .45, (0, 0, 50, 50)),
                          apple('abnormal_apple', .33, (200, 200, 260, 260))])
    assert [a['class'] for a in kept] == ['abnormal_apple']


def test_weak_normal_does_not_hide_overlapping_abnormal():
    kept = select_apples([apple('normal_apple', .45, (0, 0, 100, 100)),
                          apple('abnormal_apple', .40, (0, 0, 100, 100))])
    assert [a['class'] for a in kept] == ['abnormal_apple']


def test_unknown_class_is_ignored():
    assert select_apples([apple('potted plant', .99, (0, 0, 10, 10))]) == []


# --- apple_changes -------------------------------------------------------------------------------

A = (100, 100, 140, 140)
B = (300, 100, 340, 140)


def test_same_apples_slightly_shifted_are_no_change():
    assert cc.apple_changes([apple('normal_apple', .9, A)], [apple('normal_apple', .9, (104, 97, 144, 137))]) == []


@pytest.mark.parametrize('before,after,expected', [
    ([], [apple('normal_apple', .9, A)], ['fruit_added']),
    ([], [apple('abnormal_apple', .9, A)], ['fruit_damaged']),
    ([apple('normal_apple', .9, A)], [apple('abnormal_apple', .9, A)], ['fruit_damaged']),
    ([apple('abnormal_apple', .9, A)], [apple('abnormal_apple', .9, A)], []),
    ([apple('abnormal_apple', .9, A)], [apple('normal_apple', .9, A)], []),
    ([apple('normal_apple', .9, A)], [], ['fruit_lost']),
    # 사과를 다른 자리로 옮겨 붙이면 개수는 같아도 한쪽은 탈락, 한쪽은 생성
    ([apple('normal_apple', .9, A)], [apple('normal_apple', .9, B)], ['fruit_added', 'fruit_lost']),
])
def test_apple_changes(before, after, expected):
    assert sorted(kind for kind, _ in cc.apple_changes(before, after)) == expected


# --- coordinate mapping --------------------------------------------------------------------------

@pytest.mark.parametrize('angle,code', [r for r in cc.ROTATIONS if r[1] is not None])
def test_rotated_points_match_cv2_rotate(angle, code):
    image = np.zeros((30, 50), np.uint8)
    image[7, 41] = 255
    ys, xs = np.nonzero(cv2.rotate(image, code))
    assert cc.rotated_points(np.float32([[41, 7]]), code, (50, 30)).tolist() == [[xs[0], ys[0]]]


PLANT_APPLE = (600, 200, 640, 240)   # P01_baseline 식물(x 517~768, y 132~387) 안쪽
OTHER_APPLE = (680, 300, 720, 340)
RED, BROWN = (40, 40, 210), (40, 70, 120)   # BGR


def painted(path, *apples):
    """P01_baseline에 (박스, 색) 사과를 원으로 그려 저장한다."""
    image = cv2.imread(str(FIXTURES/'P01_baseline.png'))
    for (x1, y1, x2, y2), color in apples:
        cv2.circle(image, ((x1+x2)//2, (y1+y2)//2), (x2-x1)//2, color, -1)
    cv2.imwrite(str(path), image)
    return path


def compare(before, after, apples_before, apples_after):
    options = SimpleNamespace(**cc.DEFAULTS, apples_before=apples_before, apples_after=apples_after)
    return cc.compare_stop(str(before), str(after), options)[0]


def test_same_photo_same_apples_is_unchanged_and_counts_are_recorded(tmp_path):
    image = painted(tmp_path/'a.png', (PLANT_APPLE, RED))
    apples = [apple('normal_apple', .9, PLANT_APPLE)]
    result = compare(image, image, apples, apples)
    assert result['status'] == 'unchanged'
    assert result['fruit_source'] == 'apple_model'
    assert result['apples']['reference'] == result['apples']['current'] == {'normal_apple': 1, 'abnormal_apple': 0}


def test_model_apple_changes_are_reported_with_reference_coordinates(tmp_path):
    before = painted(tmp_path/'b.png', (PLANT_APPLE, RED))
    after = painted(tmp_path/'a.png', (PLANT_APPLE, BROWN), (OTHER_APPLE, RED))
    result = compare(before, after, [apple('normal_apple', .9, PLANT_APPLE)],
                     [apple('abnormal_apple', .9, PLANT_APPLE), apple('normal_apple', .9, OTHER_APPLE)])
    assert result['changes'] == {'fruit_damaged': 1, 'fruit_added': 1}
    # 색 규칙이 같은 자리에서 찾은 덩어리는 사과 모델 판정에 맡기고 따로 세지 않는다
    assert all(o.get('source') == 'apple_model' for o in result['objects'])
    added = next(o for o in result['objects'] if o['type'] == 'fruit_added')
    x, y, w, h = added['bbox']
    assert abs(x-680) <= 3 and abs(y-300) <= 3 and abs(w-41) <= 3 and abs(h-41) <= 3


def test_normal_apple_without_red_is_not_an_apple():
    # 실물에서 갈변 잎이 normal_apple 0.52~0.64로 나왔다. 여기서는 초록 잎 자리
    image = FIXTURES/'P01_baseline.png'
    result = compare(image, image, [], [apple('normal_apple', .9, PLANT_APPLE)])
    assert result['status'] == 'unchanged' and result['apples']['current']['normal_apple'] == 0


def test_apple_missed_in_one_photo_is_not_a_change(tmp_path):
    image = painted(tmp_path/'a.png', (PLANT_APPLE, BROWN))
    result = compare(image, image, [apple('abnormal_apple', .5, PLANT_APPLE)], [])
    assert result['status'] == 'unchanged'
    assert result['apples']['changes_checked'] == [{'type': 'fruit_lost', 'changed_fraction': 0.0, 'counted': False}]


def test_apples_outside_plant_area_are_not_counted(tmp_path):
    image = painted(tmp_path/'a.png', ((10, 600, 60, 650), RED))
    result = compare(FIXTURES/'P01_baseline.png', image, [], [apple('normal_apple', .9, (10, 600, 60, 650))])
    assert result['apples']['current']['normal_apple'] == 0 and 'fruit_added' not in result.get('changes', {})


def test_rotated_current_photo_maps_apples_back_to_reference(tmp_path):
    before = painted(tmp_path/'b.png', (PLANT_APPLE, RED))
    turned = tmp_path/'turned.png'
    cv2.imwrite(str(turned), cv2.rotate(cv2.imread(str(before)), cv2.ROTATE_90_CLOCKWISE))
    x1, y1, x2, y2 = PLANT_APPLE
    corners = cc.rotated_points(np.float32([[x1, y1], [x2, y2]]), cv2.ROTATE_90_CLOCKWISE, (1280, 720))
    box = (*corners.min(0).tolist(), *corners.max(0).tolist())
    same = compare(before, turned, [apple('normal_apple', .9, PLANT_APPLE)], [apple('normal_apple', .9, box)])
    assert same['rotation_deg'] == 90 and same['status'] == 'unchanged', same
    plain = tmp_path/'plain.png'
    cv2.imwrite(str(plain), cv2.rotate(cv2.imread(str(FIXTURES/'P01_baseline.png')), cv2.ROTATE_90_CLOCKWISE))
    gone = compare(before, plain, [apple('normal_apple', .9, PLANT_APPLE)], [])
    assert gone['rotation_deg'] == 90 and gone['changes'] == {'fruit_lost': 1}, gone


def test_without_model_apples_color_rules_are_unchanged():
    options = SimpleNamespace(**cc.DEFAULTS)
    result = cc.compare_stop(str(FIXTURES/'P01_baseline.png'), str(FIXTURES/'P01_two_orange_leaves.png'), options)[0]
    assert result['changes'] == {'leaf_color': 2} and 'fruit_source' not in result


def test_orange_leaves_outside_apple_boxes_stay_leaf_changes():
    result = compare(FIXTURES/'P01_baseline.png', FIXTURES/'P01_two_orange_leaves.png', [], [])
    assert result['changes'] == {'leaf_color': 2} and result['fruit_source'] == 'apple_model'


# --- compare_with_baseline / detector failure -----------------------------------------------------

def records(tmp_path, after_image=FIXTURES/'P01_baseline.png'):
    before, after = tmp_path/'2026-01-01_000000', tmp_path/'2026-01-01_000100'
    for folder, image in ((before, FIXTURES/'P01_baseline.png'), (after, after_image)):
        folder.mkdir()
        shutil.copy2(image, folder/'P00.png')
        (folder/'result.json').write_text(json.dumps({'route': 'gazebo_demo', 'finished': '2026-01-01T00:00:00',
                                                      'stops': [{'id': 'P00', 'status': 'ok', 'image': 'P00.png'}]}))
    return after


def test_detector_failure_falls_back_to_color_rules_and_is_recorded(tmp_path, monkeypatch):
    current = records(tmp_path)
    monkeypatch.setattr(cc, 'load_board_profile', Mock(return_value=None))
    detector = Mock()
    detector.detect.return_value = (None, None, {'status': 'failed', 'error': 'TimeoutError: x'})
    result = cc.compare_with_baseline(str(tmp_path), 'gazebo_demo', str(current), 'P00', str(current/'P00.png'),
                                      apple_detector=detector)
    assert result['status'] == 'unchanged' and 'fruit_source' not in result
    assert result['apple_model']['status'] == 'failed'


def test_detector_result_is_used_through_public_api(tmp_path, monkeypatch):
    current = records(tmp_path, painted(tmp_path/'a.png', (PLANT_APPLE, RED)))
    monkeypatch.setattr(cc, 'load_board_profile', Mock(return_value=None))
    detector = Mock()
    detector.detect.return_value = ([], [apple('normal_apple', .9, PLANT_APPLE)], {'status': 'used'})
    result = cc.compare_with_baseline(str(tmp_path), 'gazebo_demo', str(current), 'P00', str(current/'P00.png'),
                                      apple_detector=detector)
    assert result['changes'] == {'fruit_added': 1} and result['apple_model'] == {'status': 'used'}
    assert (current/result['image']).is_file()


def test_missing_runtime_never_raises(tmp_path):
    detector = AppleDetector(python=str(tmp_path/'no-python'), weights=str(tmp_path/'no-model'))
    assert not detector.available()
    before, after, trace = detector.detect('before', 'after')
    assert before is after is None and trace['status'] == 'failed' and detector.process is None


def test_hung_worker_times_out(tmp_path):
    worker = tmp_path/'hung.py'
    worker.write_text('import sys,time\nsys.stdin.readline()\ntime.sleep(30)\n')
    weights = tmp_path/'weights'
    weights.write_bytes(b'test')
    detector = AppleDetector(python=sys.executable, weights=str(weights), worker=str(worker), timeout=.3)
    _, _, trace = detector.detect('before', 'after')
    assert 'TimeoutError' in trace['error'] and detector.process is None


# --- real model on real camera photos -------------------------------------------------------------

@pytest.fixture(scope='module')
def real_detector():
    if not (Path(DEFAULT_PYTHON).is_file() and Path(DEFAULT_APPLE_WEIGHTS).is_file() and CAPTURES.exists()):
        pytest.skip('Local model runtime, apple weights and real camera photos required')
    detector = AppleDetector(timeout=60.)
    yield detector
    detector.close()


def test_real_model_finds_rotten_and_fresh_apples(real_detector):
    # 2026-11-03_212306 P01: 위 상한 사과 1, 아래 멀쩡한 사과 1 (멀쩡한 쪽에 normal/abnormal 박스가 겹쳐 나옴)
    # 2026-11-04_205859 P01: 멀쩡한 사과 1
    image = str(CAPTURES/'2026-11-03_212306/P01.png')
    found, same, trace = real_detector.detect(image, str(CAPTURES/'2026-11-04_205859/P01.png'))
    assert trace['status'] == 'used' and trace['model']['name'] == 'apple_yolo26n_aug_v1.pt'
    assert sorted(a['class'] for a in found) == ['abnormal_apple', 'normal_apple']
    assert [a['class'] for a in same] == ['normal_apple']
