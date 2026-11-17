"""직전 촬영(또는 지정한 기준 회차)과 이번 회차의 사진을 지점별로 비교해 식물 변화의 종류와 개수를 찾는다.

patrol_manager가 남긴 <capture_dir>/<회차>/<지점 ID>.png 와 result.json을 읽는다.
지점마다:
  보드 경계가 등록된 실물 루트는 두 사진에서 보드를 찾고, 안쪽만 정렬·색 보정·비교한다.
  등록 사진은 위치 확인용이다. 변화 비교는 계속 직전 순찰 사진을 사용한다.
  보드를 찾지 못하면 board_not_found로 중단하며 전체 화면으로 대체하지 않는다.
  0. 두 사진의 색을 자동 보정한다(normalize_colors): 흰 보드로 화이트밸런스, 초록 잎 채도로 진하기.
     시연 장소·조명이 바뀌어도 아래 색 기준값을 바꿀 필요가 없다.
  1. 이번 사진을 기준 사진에 맞춰 정렬한다. 특징점(ORB) + 호모그래피가 먼저, 무늬가 적어
     실패하면 ECC로 맞춘다. 폼보드 사진처럼 평면이면 호모그래피로 정확히 겹친다.
  2. 자동 노출 차이를 배경 밝기로 보정한 뒤, 화분 영역(plant_roi) 안에서 Lab 색 차이(ΔE)가 큰 픽셀을 찾는다.
     정렬이 몇 픽셀 어긋나도 괜찮도록 주변 ±shift 픽셀 중 가장 비슷한 값과 비교한다.
  3. 바뀐 픽셀마다 무엇이 생겼는지 먼저 정하고(pixel_kinds), 같은 종류끼리만 덩어리로 묶어 센다.
     서로 닿게 붙인 사과와 갈변 잎도 이렇게 해야 따로 세어진다.
       초록 잎·빈 보드 → 사과 재질   진한 빨강이 많으면 열매 생성, 갈색 부분이 섞이면 열매 손상,
                                    진한 빨강이 거의 없으면 짙은 갈변 잎 (fruit_kind)
       → 그 밖의 빨강~갈색·노랑·주황  잎 색 변화
       초록 → 배경                  잎 탈락
       기준의 빨간 열매 → 빨강 아님    열매 손상
     색상(H)은 조명에 따라 사과와 갈변 잎이 겹쳐서 색의 진하기(Lab 채도)로 가른다.
     정렬 오차로 생긴 가는 띠·밀려 보인 줄기는 버린다. 한 지점에 여러 종류가 함께 나올 수 있다
     (changes = {종류: 개수}, 면적 큰 순). 이번 사진이 90°·180° 돌아가 있어도 맞춘다.

결과는 <이번 회차>/compare_<기준 회차>/ 아래에 <지점 ID>.jpg(기준 | 이번 정렬 | 변화 표시)와 compare.json.

    ros2 run plant_observer_bringup change_compare                            # 최근 회차 vs 바로 전 회차
    ros2 run plant_observer_bringup change_compare <이번 회차>                  # 그 회차 vs 바로 전 회차
    ros2 run plant_observer_bringup change_compare <기준 회차> <이번 회차>       # 직접 지정
    ros2 run plant_observer_bringup change_compare --set-baseline <정상 회차>   # 고정 기준 회차 지정
    ros2 run plant_observer_bringup change_compare --use-baseline             # 고정 기준 회차와 비교
patrol_manager는 순찰 중에 compare_with_baseline()으로 지점마다 같은 규칙으로 바로 비교한다
(기본은 직전 촬영과 비교, compare_to:=baseline이면 고정 기준 회차).
"""
import argparse
import json
import math
import os
import sys
from types import SimpleNamespace

import cv2
import numpy as np

from plant_observer_bringup.board_mask import BoardNotFound, isolate, load_board_profile, locate_board

# 사진 크기가 달라도(Gazebo 1280x720, U20CAM 640x480) 같은 기준으로 비교하도록 이 너비로 줄여서 계산한다
WORK_WIDTH = 640
MIN_INLIERS = 25
# 판정 기본값. 명령줄 옵션과 patrol_manager의 순찰 중 비교가 함께 쓴다.
# min_object 0.0015 → 0.0005: Gazebo에서 지름이 사진 폭의 약 3.4%인 열매를 놓쳐서 낮췄다(변화 없는
# Gazebo 사진 9쌍에서 오탐 없음). 0.0003이면 약 2.3%까지 잡지만 실물 카메라 잡음으로 오탐이 늘 수 있다
DEFAULTS = {'delta_e': 20.0, 'min_object': 0.0005, 'other_threshold': 0.01, 'shift': 3}
# 사과·상한 사과·갈변 잎 구분 기준 (fruit_kind 참고)
FRUIT_MIN_CHROMA = 32
FRUIT_MAX_HUE_ANGLE = 35
ROT_MAX_CHROMA = 30
ROT_MAX_VALUE = 140
# 덩어리 안쪽 원 반지름(640px 기준). MIN_THICKNESS보다 작으면 정렬 오차로 생긴 가는 띠로 버리고,
# MIN_THICKNESS_SURE보다 작으면 밀려 보인 것인지 확인한다. 실물 사진에서 잎·사과는 대부분 13 이상,
# 줄기·잎 가장자리 띠는 3.0~6.0, 보드 끝의 작은 잎은 6.0~7.4였다
MIN_THICKNESS = 4.5
MIN_THICKNESS_SURE = 8.0
# 이번 사진의 물체 픽셀: 채도 35 이상이거나 어두운 것. 오려 붙인 사진의 흰 테두리(채도 낮음)에서 끊겨
# 서로 닿게 붙인 사과·갈변 잎이 따로 나뉜다. 허옇게 찍힌 날에도 잎·열매가 빠지지 않게 50에서 낮췄다
OBJECT_MIN_SAT = 35
OBJECT_MAX_DARK = 110
# 색 자동 보정 (normalize_colors). 시연 장소·조명이 바뀌어도 색 기준값을 손대지 않도록
# 사진마다 흰 보드로 화이트밸런스를, 초록 잎 채도 중앙값이 LEAF_TARGET_SAT가 되게 진하기를 맞춘다.
# 38은 위 색 기준값들을 맞춘 실물 사진(2026-10-30 오후, 잎 채도 중앙값 30~44)의 조건이다
LEAF_TARGET_SAT = 38
SAT_SCALE_RANGE = (0.6, 2.0)
WB_GAIN_RANGE = (0.7, 1.5)
# 흰 보드·벽을 이 밝기로 맞춘다. 실물 사진(2026-10-29~30)의 흰 보드 밝기에 가깝게 잡아 기존 색 기준값을 그대로 쓴다
WHITE_TARGET = 225.0
# 카메라를 옆으로 달았거나 사진을 돌려 저장한 경우에도 맞추도록 이 순서로 돌려 가며 정렬한다
ROTATIONS = ((0, None), (90, cv2.ROTATE_90_CLOCKWISE), (270, cv2.ROTATE_90_COUNTERCLOCKWISE),
             (180, cv2.ROTATE_180))

# 픽셀 색 분류 (OpenCV HSV: H 0~180, S·V 0~255). 실제 인쇄물·U20CAM 사진으로 경계를 다듬어야 한다
BG, GREEN, YELLOW, RED, BROWN, OTHER, ORANGE = range(7)
CHANGE_TYPES = {
    'leaf_loss': '잎 탈락',
    'fruit_added': '열매 생성',
    'leaf_color': '잎 색 변화',
    'fruit_damaged': '열매 손상',
    'other': '기타 변화',
}
OVERLAY_COLORS = {   # BGR
    'leaf_loss': (255, 0, 255),
    'fruit_added': (255, 128, 0),
    'leaf_color': (0, 220, 255),
    'fruit_damaged': (0, 0, 255),
    'other': (255, 255, 255),
}


def color_classes(bgr):
    hsv = cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
    h, s, v = (hsv[..., i].astype(np.int16) for i in range(3))
    classes = np.full(h.shape, OTHER, np.uint8)
    # 뒤에 쓴 조건이 앞의 것을 덮는다
    classes[(s < 45) & (v >= 60)] = BG                                        # 흰색·회색 보드
    classes[(((h <= 30) | (h >= 165)) & (s >= 40) & (v < 140)) | (v < 50)] = BROWN   # 갈색·거무스름
    classes[(h >= 18) & (h <= 34) & (s >= 80) & (v >= 100)] = YELLOW          # 노란 잎
    # 단풍잎(주황빛 빨강)과 사과(자홍빛 빨강)는 H 1을 경계로 가른다. 실물 U20CAM 사진(2026-10-30)에서
    # 단풍잎 덩어리 H 중앙값 +3~+8, 사과 -3~-4(빨강 0 기준)였다. 예전 경계(H 8 이하 = 빨강)는
    # 단풍잎을 열매 생성으로 오판했다
    classes[(h >= 2) & (h <= 17) & (s >= 60) & (v >= 100)] = ORANGE           # 단풍잎
    # 초록 잎. 채도 기준 50은 2026-10-30 오후처럼 사진이 허옇게 찍히면(잎 채도 중앙값 28~44) 잎을 거의 못 찾아
    # 화분 영역·정렬이 실패했다. 25에서도 흰 보드·벽·바닥은 초록으로 잡히지 않았다
    classes[(h >= 35) & (h <= 95) & (s >= 25) & (v >= 40)] = GREEN
    classes[((h <= 1) | (h >= 165)) & (s >= 90) & (v >= 70)] = RED            # 빨간 열매
    return classes


def object_mask(classes):
    """잎·열매로 볼 픽셀(초록·노랑·주황·빨강·갈색)."""
    mask = np.isin(classes, (GREEN, YELLOW, ORANGE, RED, BROWN)).astype(np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(bool)


def plant_roi(classes, grow=0.25):
    """식물 색(초록·노랑·주황·빨강) 픽셀이 가장 많이 모인 곳을 둘러싼 사각형 마스크.

    갈색·검정은 넣지 않는다. 벽 너머 의자·사람 같은 어두운 배경이 잡히면 화분 밖까지 넓어진다.
    비교와 정렬을 이 안으로 한정해서 배경 변화를 식물 변화로 오판하지 않게 한다.
    """
    mask = np.isin(classes, (GREEN, YELLOW, ORANGE, RED)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    roi = np.zeros(classes.shape, np.uint8)
    # 잎 사이 틈을 메워 한 화분을 한 덩어리로 묶는다
    count, _, stats, _ = cv2.connectedComponentsWithStats(cv2.dilate(mask, np.ones((25, 25), np.uint8)))
    if count <= 1:
        return roi
    x, y, bw, bh, _ = stats[1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])]
    dx, dy = int(bw * grow), int(bh * grow)
    roi[max(0, y - dy):y + bh + dy, max(0, x - dx):x + bw + dx] = 255
    return roi


def plausible(homography):
    """크기가 반 이하·두 배 이상으로 바뀌거나 심하게 비트는 정렬은 실패로 본다."""
    if homography is None or not np.isfinite(homography).all():
        return False
    det = np.linalg.det(homography[:2, :2])
    # Determinant alone accepts a severe stretch on one axis and compression
    # on the other. Such a warp made a displaced plant look like a missing leaf.
    scales = np.linalg.svd(homography[:2, :2], compute_uv=False)
    return (0.5 < det < 2.0 and scales[-1] > 0.5 and scales[0] < 2.0
            and scales[0] / scales[-1] < 1.8
            and abs(homography[2, 0]) < 2e-3 and abs(homography[2, 1]) < 2e-3)


def align_features(before, after, roi_before, roi_after):
    """이번 사진 → 기준 사진 호모그래피. 실패하면 (None, 정상 매칭 수)."""
    orb = cv2.ORB_create(3000)
    g1 = cv2.cvtColor(before, cv2.COLOR_BGR2GRAY)
    g2 = cv2.cvtColor(after, cv2.COLOR_BGR2GRAY)
    k1, d1 = orb.detectAndCompute(g1, roi_before)
    k2, d2 = orb.detectAndCompute(g2, roi_after)
    if d1 is None or d2 is None or len(k1) < MIN_INLIERS or len(k2) < MIN_INLIERS:
        return None, 0
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d2, d1, k=2)
    good = [p[0] for p in pairs if len(p) == 2 and p[0].distance < 0.75 * p[1].distance]
    if len(good) < MIN_INLIERS:
        return None, len(good)
    src = np.float32([k2[m.queryIdx].pt for m in good])
    dst = np.float32([k1[m.trainIdx].pt for m in good])
    homography, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 4.0)
    count = int(inliers.sum()) if inliers is not None else 0
    if count < MIN_INLIERS or not plausible(homography):
        return None, count
    return homography, count


def align_ecc(before, after, mask_before, mask_after, roi_before):
    """무늬가 적은 장면(Gazebo)용. 잎·열매 무게중심·면적으로 초기값을 잡고 ECC로 다듬는다."""
    g1 = cv2.GaussianBlur(cv2.cvtColor(before, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.float32)
    g2 = cv2.GaussianBlur(cv2.cvtColor(after, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(np.float32)
    # ECC의 warp는 기준 사진 좌표 → 이번 사진 좌표
    warp = np.eye(3, dtype=np.float32)
    if mask_before.any() and mask_after.any():
        m1 = cv2.moments(mask_before.astype(np.uint8), True)
        m2 = cv2.moments(mask_after.astype(np.uint8), True)
        scale = float(np.sqrt(m2['m00'] / m1['m00']))
        c1 = np.array([m1['m10'] / m1['m00'], m1['m01'] / m1['m00']])
        c2 = np.array([m2['m10'] / m2['m00'], m2['m01'] / m2['m00']])
        if 0.5 < scale < 2.0:
            warp[0, 0] = warp[1, 1] = scale
            warp[:2, 2] = c2 - scale * c1
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 200, 1e-5)
    try:
        _, warp = cv2.findTransformECC(g1, g2, warp, cv2.MOTION_HOMOGRAPHY, criteria, roi_before, 5)
    except cv2.error:
        return None
    homography = np.linalg.inv(warp)
    homography /= homography[2, 2]
    return homography if plausible(homography) else None


def match_brightness(before_lab, after_lab, background):
    """자동 노출·화이트밸런스 차이를 배경(잎·열매 밖) 픽셀의 중앙값 차이로 맞춘다."""
    out = after_lab.astype(np.float32)
    if background.sum() < 500:
        return out
    for c in range(3):
        out[..., c] += np.median(before_lab[..., c][background].astype(np.float32)
                                 - out[..., c][background])
    return out


def color_difference(before_lab, after_lab, shift):
    """ΔE(CIE76). 이번 사진을 ±shift 픽셀 옮겨 가며 가장 작은 차이를 쓴다(정렬 오차 흡수)."""
    before_lab = before_lab.astype(np.float32)
    # OpenCV 8비트 Lab은 L이 0~255로 늘어나 있다. ΔE 단위(L 0~100)로 맞춘다
    scale = np.array([100.0 / 255.0, 1.0, 1.0], np.float32)
    best = None
    for dy in range(-shift, shift + 1):
        for dx in range(-shift, shift + 1):
            moved = np.roll(after_lab, (dy, dx), axis=(0, 1))
            de = np.sqrt((((before_lab - moved) * scale) ** 2).sum(axis=2))
            best = de if best is None else np.minimum(best, de)
    return best


def normalize_colors(bgr, fallback=None, mask=None):
    """(화이트밸런스만 맞춘 사진, 진하기까지 맞춘 사진, 보정값 (채널 이득, 채도 배율)).

    흰 보드·벽으로 화이트밸런스를, 초록 잎 채도 중앙값으로 진하기를 맞춘다. 진하기 보정은 초록 잎 찾기·
    화분 영역·정렬에만 쓴다. 사과·갈변 잎을 가르는 색 진하기는 사진마다 배율이 달라지면 오히려 흔들려서
    (실물 4회차에서 사과↔갈변 잎이 뒤바뀌었다) 화이트밸런스만 맞춘 사진으로 판정한다.
    흰색이나 잎이 너무 적게 보이면 fallback(기준 사진의 보정값)을 쓴다.
    """
    img = bgr.astype(np.float32)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    allowed = np.ones(bgr.shape[:2], bool) if mask is None else mask.astype(bool)
    # 흰 보드·벽: 밝은 쪽 30% 중 채도가 낮은 픽셀. 고정 채도 기준(30 미만)은 푸른·노란 조명에서
    # 흰 보드 자체가 채도 30을 넘어 못 찾았다
    v = hsv[..., 2]
    bright = allowed & (v >= np.percentile(v[allowed], 70)) & (v <= 250) if allowed.any() else allowed
    white = bright & (hsv[..., 1] <= np.percentile(hsv[..., 1][bright], 40)) if bright.any() else bright
    if white.sum() >= 500:
        # 흰색이 WHITE_TARGET 회색이 되도록 채널마다 곱한다: 화이트밸런스와 노출(밝기)을 함께 맞춘다.
        # 상한 사과의 '어두운 부분'(밝기 140 미만) 같은 밝기 기준이 장소 밝기에 흔들리지 않게 한다
        means = img[white].mean(axis=0)
        gains = np.clip(WHITE_TARGET / np.maximum(means, 1.0), *WB_GAIN_RANGE)
    else:
        gains = fallback[0] if fallback else np.ones(3, np.float32)
    balanced = np.clip(img * gains, 0, 255).astype(np.uint8)
    hsv = cv2.cvtColor(balanced, cv2.COLOR_BGR2HSV).astype(np.float32)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    leaf = allowed & (h >= 35) & (h <= 95) & (s >= 15) & (v >= 50) & (v <= 230)
    if leaf.sum() >= 300:
        sat_scale = float(np.clip(LEAF_TARGET_SAT / np.median(s[leaf]), *SAT_SCALE_RANGE))
    else:
        sat_scale = fallback[1] if fallback else 1.0
    hsv[..., 1] = np.clip(s * sat_scale, 0, 255)
    return balanced, cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR), (gains, sat_scale)


def object_pixels(hsv):
    """이번 사진에서 물체(잎·열매·줄기)로 볼 픽셀. 흰 보드와 오려 붙인 사진의 흰 테두리는 빠진다."""
    return (hsv[..., 1] >= OBJECT_MIN_SAT) | (hsv[..., 2] < OBJECT_MAX_DARK)


def object_kind(blob, classes_before, classes_after, hsv, chroma, hue_angle):
    """새로 생긴(색이 바뀐) 물체 조각 하나의 변화 종류."""
    if (classes_before[blob] == RED).mean() >= 0.4:
        # 기준 회차에 있던 빨간 열매가 바뀜
        return 'fruit_damaged'
    if warm_pixels(hsv)[blob].mean() >= 0.5:
        return fruit_kind(blob, chroma, hue_angle, hsv[..., 2])
    if np.isin(classes_after[blob], (YELLOW, ORANGE)).mean() >= 0.5:
        return 'leaf_color'
    return 'other'


def thickness(mask):
    """덩어리 안에 들어가는 가장 큰 원의 반지름(픽셀)."""
    return float(cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5).max())


def shifted_copy(blob, before_lab, after_lab, radius=12, max_delta=12.0):
    """덩어리 대부분이 기준 사진의 radius 픽셀 안쪽 같은 색과 맞으면 True (정렬 오차로 밀려 보인 것).

    실물 사진에서 보드 끝 화분 가장자리는 정렬 오차가 8~12픽셀이었다(8로는 못 걸렀다).
    """
    ys, xs = np.nonzero(blob)
    y0, y1 = max(0, ys.min() - radius), min(blob.shape[0], ys.max() + radius + 1)
    x0, x1 = max(0, xs.min() - radius), min(blob.shape[1], xs.max() + radius + 1)
    region = blob[y0:y1, x0:x1]
    after_crop = after_lab[y0:y1, x0:x1].astype(np.float32)
    before_pad = cv2.copyMakeBorder(before_lab[y0:y1, x0:x1].astype(np.float32),
                                    radius, radius, radius, radius, cv2.BORDER_REPLICATE)
    scale = np.array([100.0 / 255.0, 1.0, 1.0], np.float32)
    best = np.full(region.shape, np.inf, np.float32)
    for dy in range(0, 2 * radius + 1, 2):
        for dx in range(0, 2 * radius + 1, 2):
            moved = before_pad[dy:dy + region.shape[0], dx:dx + region.shape[1]]
            best = np.minimum(best, np.sqrt((((after_crop - moved) * scale) ** 2).sum(axis=2)))
    return (best[region] < max_delta).mean() >= 0.6


# 예전(옆면 재촬영으로 판정을 확정하던 때) 기록의 final.verdict. 관찰 기록 화면이 옛 회차를 보여줄 때 쓴다
VERDICT_TEXT = {
    'confirmed': '재촬영 확인',
    'mismatch': '재촬영 불일치',
    'recheck_used': '재촬영 사진으로 판정',
    'recheck_failed': '재촬영 실패',
}


def describe_changes(entry):
    """'잎 색 변화 4곳, 열매 손상 2곳'. 예전 결과(changes 없음)는 change·count로."""
    changes = entry.get('changes') or ({entry['change']: entry.get('count', 0)} if entry.get('change') else {})
    return ', '.join(f'{CHANGE_TYPES.get(kind, kind)} {n}곳' for kind, n in changes.items())


def warm_pixels(hsv):
    """빨강~주황~갈색 계열 픽셀 (HSV, int)."""
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return ((h <= 30) | (h >= 150)) & (s >= 40) & (v >= 40)


def fruit_kind(blob, chroma, hue_angle, value):
    """사과 재질 덩어리가 사과·상한 사과·짙은 갈변 잎 중 무엇인지.

    실물 U20CAM 사진(2026-10-30, 4회차)에서 진한 빨강(Lab 채도 32 이상, 색상각 35° 미만) 픽셀 비율이
    사과 0.66~0.73, 갈변 잎 0.00~0.02로 확실히 갈렸다. 색상(H)은 조명에 따라 서로 겹쳤다.
    상한 부분(채도 30 미만·밝기 140 미만)은 멀쩡한 사과에서 0~3%였다.
    """
    n = blob.sum()
    red = (blob & (chroma >= FRUIT_MIN_CHROMA) & (hue_angle < FRUIT_MAX_HUE_ANGLE)).sum() / n
    rot = (blob & (chroma < ROT_MAX_CHROMA) & (value < ROT_MAX_VALUE)).sum() / n
    if red >= 0.1 and rot >= 0.2:
        return 'fruit_damaged'
    if red >= 0.35:
        return 'fruit_added'
    return 'leaf_color'


def draw_label(image, text, org, color):
    cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


def compare_stop(before_path, after_path, args):
    before_full = cv2.imread(before_path)
    after_full = cv2.imread(after_path)
    if before_full is None or after_full is None:
        return {'status': 'no_image'}, None
    board = getattr(args, 'board', None)
    scope = {}
    if board is not None:
        scope = {'comparison_scope': 'board', 'board': {}}
        for name, source in (('reference', before_full), ('current', after_full)):
            try:
                mask, info = locate_board(source, board)
            except BoardNotFound as exc:
                return {**scope, 'status': 'board_not_found', 'failed_image': name, 'reason': str(exc)}, None
            scope['board'][name] = info
            if name == 'reference':
                board_before_full = mask
            else:
                board_after_full = mask
        # Exterior pixels must disappear before any resize/filter/statistic.
        before_full = isolate(before_full, board_before_full)
        after_full = isolate(after_full, board_after_full)
    else:
        if getattr(args, 'model_scope_before', None) is not None:
            board_before_full = args.model_scope_before
            board_after_full = args.model_scope_after
            before_full = isolate(before_full, board_before_full)
            after_full = isolate(after_full, board_after_full)
            scope = {'comparison_scope': 'model_bbox'}
        else:
            board_before_full = np.full(before_full.shape[:2], 255, np.uint8)
            board_after_full = np.full(after_full.shape[:2], 255, np.uint8)
    scale = WORK_WIDTH / before_full.shape[1]
    before = cv2.resize(before_full, (WORK_WIDTH, round(before_full.shape[0] * scale)),
                        interpolation=cv2.INTER_AREA)
    board_before = cv2.resize(board_before_full, before.shape[1::-1], interpolation=cv2.INTER_NEAREST)
    # 조명이 달라도 같은 색 기준으로 판정하도록 두 사진을 먼저 보정한다 (normalize_colors 참고).
    # before·after·warped는 화이트밸런스만, *_vivid는 진하기까지 맞춘 사진
    before, before_vivid, correction_before = normalize_colors(before, mask=board_before)
    after_full, after_vivid_full, correction_after = normalize_colors(
        after_full, fallback=correction_before, mask=board_after_full)
    classes_before = color_classes(before_vivid)
    classes_before[board_before == 0] = BG
    mask_before = object_mask(classes_before)
    # 정렬은 화분 영역 특징점만 쓴다. 벽 너머 배경은 로봇 위치에 따라 어긋나 보인다
    roi_before = plant_roi(classes_before) & board_before
    if not roi_before.any():
        return {**scope, 'status': 'no_plant'}, None

    def prepared(image, code, size=None):
        turned = image if code is None else cv2.rotate(image, code)
        size = size or (WORK_WIDTH, round(turned.shape[0] * WORK_WIDTH / turned.shape[1]))
        return cv2.resize(turned, size, interpolation=cv2.INTER_NEAREST if image.ndim == 2 else cv2.INTER_AREA)

    homography, inliers, rotation = None, 0, 0
    for rotation, code in ROTATIONS:
        after_vivid = prepared(after_vivid_full, code)
        board_after = prepared(board_after_full, code)
        classes_candidate = color_classes(after_vivid)
        classes_candidate[board_after == 0] = BG
        homography, count = align_features(before_vivid, after_vivid, roi_before,
                                           plant_roi(classes_candidate) & board_after)
        inliers = max(inliers, count)
        if homography is not None:
            after = prepared(after_full, code)
            break
    method = 'features'
    if homography is None:
        rotation = 0
        after_vivid = prepared(after_vivid_full, None, before.shape[1::-1])
        after = prepared(after_full, None, before.shape[1::-1])
        board_after = prepared(board_after_full, None, before.shape[1::-1])
        if after_full.shape[0] * before.shape[1] == after_full.shape[1] * before.shape[0]:
            homography = align_ecc(before_vivid, after_vivid, mask_before,
                                   object_mask(color_classes(after_vivid)) & (board_after > 0), roi_before)
            method = 'ecc'
    if homography is None:
        return {**scope, 'status': 'align_failed', 'feature_inliers': inliers}, None

    h, w = before.shape[:2]
    warped = cv2.warpPerspective(after, homography, (w, h))
    valid = (cv2.warpPerspective(board_after, homography, (w, h)) == 255) & (board_before > 0)
    if board is not None and valid.sum() < 0.85 * (board_before > 0).sum():
        return {**scope, 'status': 'align_failed', 'reason': '정렬한 두 보드의 겹치는 영역 부족'}, None
    # Delta-E samples ±shift pixels after a 5x5 blur. Keep every sample inside
    # both board masks; warping and smoothing must not mix exterior pixels in.
    margin = max(2, args.shift + 3) if board is not None else 2
    valid = cv2.erode(valid.astype(np.uint8), np.ones((2 * margin + 1, 2 * margin + 1), np.uint8)).astype(bool)
    classes_after = color_classes(cv2.warpPerspective(after_vivid, homography, (w, h)))
    hsv = cv2.cvtColor(cv2.GaussianBlur(warped, (5, 5), 0), cv2.COLOR_BGR2HSV).astype(np.int16)
    hsv_before = cv2.cvtColor(cv2.GaussianBlur(before, (5, 5), 0), cv2.COLOR_BGR2HSV).astype(np.int16)
    # 연한 분홍빛 갈변 잎(실물 S 70~84)은 어느 식물 색에도 안 들어가 비교 영역에서 빠졌다.
    # 빨강~갈색 계열 픽셀도 비교 영역에 넣는다
    mask_warped = (object_mask(classes_after) | warm_pixels(hsv)) & valid

    region = cv2.dilate((mask_before | mask_warped).astype(np.uint8),
                        np.ones((7, 7), np.uint8)).astype(bool) & valid
    # 화분 밖(벽 너머 사람·의자 등)의 변화는 보지 않는다
    region &= roi_before.astype(bool)
    if getattr(args, 'foreground_before', None) is not None:
        model_before = cv2.resize(args.foreground_before, (w, h), interpolation=cv2.INTER_NEAREST)
        rotation_code = next(code for angle, code in ROTATIONS if angle == rotation)
        model_after = prepared(args.foreground_after, rotation_code, after.shape[1::-1])
        model_warped = cv2.warpPerspective(model_after, homography, (w, h), flags=cv2.INTER_NEAREST)
        # Union preserves a removed leaf visible only in the reference image.
        model_union = ((model_before > 0) | (model_warped > 0)).astype(np.uint8)
        region &= cv2.dilate(model_union, np.ones((9, 9), np.uint8)).astype(bool)
    if region.sum() < 200:
        return {**scope, 'status': 'no_plant', 'align': method}, None

    before_lab = cv2.cvtColor(cv2.GaussianBlur(before, (5, 5), 0), cv2.COLOR_BGR2LAB)
    after_lab = cv2.cvtColor(cv2.GaussianBlur(warped, (5, 5), 0), cv2.COLOR_BGR2LAB)
    after_lab = match_brightness(before_lab, after_lab, valid & ~region)
    delta = color_difference(before_lab, after_lab, args.shift)
    a, b = after_lab[..., 1] - 128, after_lab[..., 2] - 128
    chroma, hue_angle = np.hypot(a, b), np.degrees(np.arctan2(b, a))
    warm_before_near = cv2.dilate(warm_pixels(hsv_before).astype(np.uint8),
                                  np.ones((9, 9), np.uint8)).astype(bool)
    ys, xs = np.nonzero(roi_before)
    roi_box = (xs.min(), ys.min(), xs.max(), ys.max())

    # 덩어리를 바뀐 픽셀만으로 자르면 서로 닿게 붙인 사과·갈변 잎이 한 덩어리가 되고, 색으로만 자르면
    # 상한 사과가 여러 조각이 된다. 오려 붙인 사진은 흰 테두리가 있어서, 이번 사진의 물체 픽셀을
    # 테두리에서 끊어 조각내고 그중 바뀐 부분만 한 물체로 센다. 사라진 잎은 물체가 아닌 쪽에서 찾는다
    changed = (delta > args.delta_e) & region
    obj_after = object_pixels(hsv)

    min_area = args.min_object * h * w
    objects = []
    keep = np.zeros(changed.shape, bool)
    overlay = before.copy()

    def components(mask, close):
        mask = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        if close:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        return [labels == i for i in range(1, count) if stats[i][cv2.CC_STAT_AREA] >= min_area]

    def real(blob, warm, new_object=False):
        """정렬 오차로 생긴 가는 띠·밀려 보인 줄기·화분이 아니면 True.

        새로 생긴 물체는 두께와 상관없이 바로 옆 기준 사진에 같은 색이 있었는지 본다. 곡면 보드에 인쇄된
        화분 가장자리가 초승달 모양(두께 8px 이상)으로 밀려 보여 잎 색 변화로 잡혔다(2026-10-30 실물).
        사라진 잎은 옆에 늘 흰 보드가 있어서 이 확인을 하면 안 된다.
        """
        thick = thickness(blob)
        if thick < MIN_THICKNESS:
            return False
        if (thick < MIN_THICKNESS_SURE or new_object) and shifted_copy(blob, before_lab, after_lab):
            return False
        # 기준 사진 바로 옆에 원래 있던 갈색 줄기·화분이 정렬 오차로 밀려 보인 것
        return not (warm and (blob & warm_before_near).sum() >= 0.5 * blob.sum())

    found = []
    new_objects = np.zeros(changed.shape, bool)
    for blob in components(changed & obj_after & roi_before.astype(bool), close=False):
        kind = object_kind(blob, classes_before, classes_after, hsv, chroma, hue_angle)
        if real(blob, kind in ('fruit_added', 'leaf_color'), new_object=True):
            found.append((kind, blob))
            if kind != 'other':
                new_objects |= blob
    # 바꿔 붙인 잎·사과가 원래 초록 잎 자리에서 살짝 비켜 붙으면 드러난 보드가 잎 탈락처럼 보인다.
    # 새 물체 바로 옆의 것은 교체의 일부로 보고 세지 않는다
    near_new = cv2.dilate(new_objects.astype(np.uint8), np.ones((11, 11), np.uint8)).astype(bool)
    for blob in components(changed & ~obj_after & (classes_before == GREEN), close=True):
        if real(blob, False) and not (blob & near_new).any():
            found.append(('leaf_loss', blob))

    for kind, blob in found:
        x, y, bw, bh = cv2.boundingRect(blob.astype(np.uint8))
        area = int(blob.sum())
        # 화분 영역 경계에 닿은 것은 영역 밖 배경(벽 너머 사람·의자)이 이어져 들어온 것으로 본다.
        # 잎·열매는 식물 범위보다 넓게 잡은 영역 안쪽에 있어 경계에 닿지 않는다
        if (x <= roi_box[0] + 1 or y <= roi_box[1] + 1
                or x + bw >= roi_box[2] or y + bh >= roi_box[3]):
            continue
        # 종류를 못 정한 덩어리는 반사·그림자일 수 있어 더 커야 센다
        if kind == 'other' and area < args.other_threshold * h * w:
            continue
        keep |= blob
        color = OVERLAY_COLORS[kind]
        overlay[blob] = (0.4 * overlay[blob] + 0.6 * np.array(color)).astype(np.uint8)
        cv2.rectangle(overlay, (x, y), (x + bw, y + bh), color, 2)
        draw_label(overlay, kind, (x, max(14, y - 4)), color)
        # 원본 사진 픽셀 좌표로 기록
        objects.append({'type': kind, 'bbox': [round(v / scale) for v in (x, y, bw, bh)],
                        'area_px': round(area / scale ** 2)})

    # 한 지점에 여러 종류가 함께 생길 수 있다(예: 잎 색 변화 4 + 열매 손상 2). 종류별 개수를 면적 큰 순으로.
    # 종류가 정해진 변화가 하나라도 있으면 기타 변화는 뺀다
    typed = [obj for obj in objects if obj['type'] != 'other'] or objects
    area_by_type = {}
    for obj in typed:
        area_by_type[obj['type']] = area_by_type.get(obj['type'], 0) + obj['area_px']
    changes = {kind: sum(obj['type'] == kind for obj in typed)
               for kind in sorted(area_by_type, key=area_by_type.get, reverse=True)}
    change = next(iter(changes), None)
    entry = {
        **scope,
        'status': 'changed' if change else 'unchanged',
        'change': change,
        'count': changes.get(change, 0),
        'changes': changes,
        'changed_ratio': round(float(keep.sum() / region.sum()), 4),
        'align': method,
        'feature_inliers': inliers,
        'objects': objects,
    }
    if rotation:
        entry['rotation_deg'] = rotation
    entry['color_correction'] = {
        name: {'wb_gains_bgr': [round(float(g), 3) for g in gains], 'sat_scale': round(sat, 3)}
        for name, (gains, sat) in (('reference', correction_before), ('current', correction_after))}

    panels = [before, warped, overlay]
    if board is not None:
        contours, _ = cv2.findContours(board_before, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for panel in panels:
            cv2.drawContours(panel, contours, -1, (80, 210, 80), 2)
    verdict = ', '.join(f'{kind} x{n}' for kind, n in changes.items()) or 'unchanged'
    for panel, text in zip(panels, ('reference', 'now (aligned)', verdict)):
        cv2.putText(panel, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(panel, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return entry, np.hstack(panels)


def load_lap(folder):
    with open(os.path.join(folder, 'result.json')) as f:
        return json.load(f)


def resolve(folder, capture_dir):
    path = os.path.expanduser(folder)
    path = path if os.path.isdir(path) else os.path.join(capture_dir, folder)
    if not os.path.isfile(os.path.join(path, 'result.json')):
        sys.exit(f'{path}에 result.json이 없습니다')
    return path


def baseline_file(capture_dir, route):
    return os.path.join(capture_dir, f'baseline_{route}.txt')


def previous_lap(capture_dir, route, current_folder):
    """직전 완료 순찰 한 회차를 고른다. 순찰 이력이 없을 때만 성공한 기준 등록을 쓴다.

    누락된 지점·방향을 오래된 사진으로 보충하지 않는다. 중단된 회차는 제외하지만
    복귀만 실패한 완료 회차의 촬영 기록은 사용할 수 있다.
    """
    current = os.path.basename(os.path.normpath(current_folder))
    enrollment = None
    for lap in sorted(os.listdir(capture_dir), reverse=True):
        if lap >= current:
            continue
        try:
            result = load_lap(os.path.join(capture_dir, lap))
        except (OSError, ValueError):
            continue
        if not isinstance(result, dict) or result.get('route') != route:
            continue
        if result.get('mode') == 'baseline':
            if result.get('baseline_complete') and enrollment is None:
                enrollment = lap
        elif result.get('finished'):
            return lap
    return enrollment


def saved_view(stop, view_id, prefer_recheck=False):
    """재촬영이 성공했으면 같은 방향의 가장 나중 사진을 사용한다."""
    if prefer_recheck:
        view = stop.get('recheck', {}).get('views', {}).get(view_id)
        if view and view.get('status') == 'ok' and view.get('image'):
            return view
    return stop.get('views', {}).get(view_id) or (stop if view_id == 'front' else None)


def observed_stops(capture_dir, route, current_folder):
    """처음 촬영할 화분을 구분하기 위한 완료 회차의 정면 촬영 이력."""
    current = os.path.basename(os.path.normpath(current_folder))
    observed = set()
    for lap in os.listdir(capture_dir):
        if lap >= current:
            continue
        try:
            result = load_lap(os.path.join(capture_dir, lap))
        except (OSError, ValueError):
            continue
        if not isinstance(result, dict) or result.get('route') != route:
            continue
        if not (result.get('baseline_complete') if result.get('mode') == 'baseline' else result.get('finished')):
            continue
        for stop in result.get('stops', []):
            front = saved_view(stop, 'front', prefer_recheck=True)
            if (front and front.get('status') == 'ok' and front.get('image')
                    and os.path.isfile(os.path.join(capture_dir, lap, front['image']))):
                observed.add(stop['id'])
    return observed


def pick_before(capture_dir, after_dir, use_baseline=False):
    """순찰 노드와 같은 직전 완료 회차 선택. use_baseline이면 고정 기준을 쓴다."""
    route = load_lap(after_dir)['route']
    path = baseline_file(capture_dir, route)
    if use_baseline and os.path.isfile(path):
        with open(path) as f:
            return resolve(f.read().strip(), capture_dir)
    after_name = os.path.basename(os.path.normpath(after_dir))
    earlier = previous_lap(capture_dir, route, after_dir)
    if earlier is None:
        sys.exit(f'{after_name}와 같은 루트({route})의 이전 회차가 없습니다. --set-baseline으로 기준 회차를 지정하세요')
    return os.path.join(capture_dir, earlier)


def reference_image(capture_dir, route, stop_id, current_folder, use_baseline=False,
                    view_id='front', reference_lap=None):
    """stop_id의 비교 기준 사진 (회차 이름, 경로). 없으면 None.

    같은 루트·같은 촬영 방향만 선택한다. 기본은 직전 완료 순찰 한 회차다.
    예전 자료는 front로만 해석한다. 옆면 기준이 없으면 정면으로 대신하지 않는다.
    """
    current = os.path.basename(os.path.normpath(current_folder))

    def stop_image(lap):
        folder = os.path.join(capture_dir, lap)
        if lap >= current or not os.path.isfile(os.path.join(folder, 'result.json')):
            return None
        try:
            result = load_lap(folder)
        except (OSError, ValueError):
            return None
        if not isinstance(result, dict):
            return None
        if result.get('route') != route:
            return None
        if result.get('mode') == 'baseline' and not result.get('baseline_complete'):
            return None
        for stop in result.get('stops', []):
            if stop.get('id') != stop_id:
                continue
            view = saved_view(stop, view_id, prefer_recheck=not use_baseline)
            if view and view.get('status') == 'ok' and view.get('image'):
                image = os.path.join(folder, view['image'])
                if os.path.isfile(image):
                    return lap, image
        return None

    if view_id not in ('front', 'left', 'right'):
        raise ValueError(f'알 수 없는 촬영 방향: {view_id}')
    if reference_lap is not None:
        return stop_image(os.path.basename(os.path.normpath(reference_lap)))
    path = baseline_file(capture_dir, route)
    if use_baseline and os.path.isfile(path):
        with open(path) as f:
            return stop_image(os.path.basename(os.path.normpath(f.read().strip())))
    lap = previous_lap(capture_dir, route, current_folder)
    return stop_image(lap) if lap else None


def compare_with_baseline(capture_dir, route, lap_folder, stop_id, image_path, params=None, out_name=None,
                          use_baseline=False, view_id='front', reference_lap=None,
                          current_pose=None, max_pose_distance=0.20, max_pose_yaw=0.35,
                          model_fallback=None):
    """방금 찍은 사진을 비교 기준 사진(기본: 직전 촬영)과 비교한다. patrol_manager가 지점마다 부른다.

    비교 그림은 <lap_folder>/<out_name>, 기본 <지점 ID>_compare.jpg. 기준 사진이 없으면 status 'no_reference'.
    """
    reference = reference_image(capture_dir, route, stop_id, lap_folder, use_baseline,
                                view_id, reference_lap)
    if reference is None:
        return {'status': 'no_reference', 'view_id': view_id}
    reference_lap, reference_path = reference
    metadata = {'reference_lap': reference_lap, 'reference_image': os.path.basename(reference_path),
                'view_id': view_id}
    if view_id != 'front':
        record = load_lap(os.path.join(capture_dir, reference_lap))
        baseline_pose = next(((saved_view(s, view_id, prefer_recheck=not use_baseline) or {}).get('pose')
                              for s in record.get('stops', []) if s.get('id') == stop_id), None)
        if current_pose is None or baseline_pose is None:
            return {**metadata, 'status': 'view_mismatch', 'reason': '촬영 위치 기록 없음'}
        distance = math.hypot(current_pose['x'] - baseline_pose['x'],
                              current_pose['y'] - baseline_pose['y'])
        yaw = abs(math.atan2(math.sin(current_pose['yaw'] - baseline_pose['yaw']),
                             math.cos(current_pose['yaw'] - baseline_pose['yaw'])))
        metadata.update(reference_pose=baseline_pose, pose_distance_m=round(distance, 3),
                        pose_yaw_difference_deg=round(math.degrees(yaw), 1))
        if not (math.isfinite(distance) and math.isfinite(yaw)
                and distance <= max_pose_distance and yaw <= max_pose_yaw):
            return {**metadata, 'status': 'view_mismatch', 'reason': '기준 사진과 촬영 위치·방향 차이가 큼'}
    try:
        board = load_board_profile(route, stop_id)
    except BoardNotFound as exc:
        return {**metadata, 'comparison_scope': 'board', 'status': 'board_not_found', 'reason': str(exc)}
    options = SimpleNamespace(**{**DEFAULTS, **(params or {}), 'board': board})
    entry, picture = compare_stop(reference_path, image_path, options)
    # Never replace a valid primary verdict, a missing reference/photo, invalid
    # board configuration, or a side-view pose mismatch with model inference.
    if model_fallback is not None and view_id == 'front':
        from plant_observer_bringup.model_fallback import ELIGIBLE_STATUSES
        if entry['status'] in ELIGIBLE_STATUSES:
            candidate, candidate_picture, trace = model_fallback.compare(
                reference_path, image_path, options, compare_stop)
            if candidate is not None and candidate['status'] in ('changed', 'unchanged'):
                entry = {**candidate, 'analysis_source': 'model_fallback', 'requires_review': True,
                         'primary_status': entry['status'], 'primary_result': entry,
                         'model_fallback': trace}
                picture = candidate_picture
            else:
                entry = {**entry, 'model_fallback': trace}
    entry = {**metadata, **entry}
    if picture is not None:
        entry['image'] = out_name or f'{stop_id}_compare.jpg'
        cv2.imwrite(os.path.join(lap_folder, entry['image']), picture)
    return entry


def latest_lap(capture_dir):
    laps = sorted(d for d in os.listdir(capture_dir)
                  if os.path.isfile(os.path.join(capture_dir, d, 'result.json')))
    if not laps:
        sys.exit(f'{capture_dir}에 result.json이 있는 회차가 없습니다')
    return os.path.join(capture_dir, laps[-1])


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('laps', nargs='*', help='[기준 회차] [이번 회차] 폴더 (이름 또는 경로)')
    parser.add_argument('--capture-dir', default='~/plant_ws/captures')
    parser.add_argument('--set-baseline', metavar='LAP',
                        help='이 회차를 그 루트의 기준(정상 화분) 회차로 지정 (--use-baseline일 때 쓴다)')
    parser.add_argument('--use-baseline', action='store_true',
                        help='직전 회차 대신 --set-baseline으로 지정한 기준 회차와 비교')
    parser.add_argument('--delta-e', type=float, default=DEFAULTS['delta_e'],
                        help='이보다 색 차이(ΔE)가 크면 바뀐 픽셀 (기본 %(default)s)')
    parser.add_argument('--min-object', type=float, default=DEFAULTS['min_object'],
                        help='잎·열매 하나로 볼 최소 덩어리 크기, 사진 넓이 대비 (기본 %(default)s = 640x480에서 약 150픽셀)')
    parser.add_argument('--other-threshold', type=float, default=DEFAULTS['other_threshold'],
                        help='종류를 못 정한 덩어리는 이 크기 이상만 기타 변화로 셈 (기본 %(default)s)')
    parser.add_argument('--shift', type=int, default=DEFAULTS['shift'],
                        help='정렬 오차 허용 픽셀 (640px 기준, 기본 %(default)s)')
    args = parser.parse_args()
    capture_dir = os.path.expanduser(args.capture_dir)

    if args.set_baseline:
        lap = resolve(args.set_baseline, capture_dir)
        route = load_lap(lap)['route']
        with open(baseline_file(capture_dir, route), 'w') as f:
            f.write(os.path.basename(os.path.normpath(lap)) + '\n')
        print(f'루트 {route}의 기준 회차: {lap}')
        return

    if len(args.laps) > 2:
        parser.error('회차 폴더는 최대 두 개입니다')
    if len(args.laps) == 2:
        before_dir, after_dir = (resolve(lap, capture_dir) for lap in args.laps)
    else:
        after_dir = resolve(args.laps[0], capture_dir) if args.laps else latest_lap(capture_dir)
        before_dir = pick_before(capture_dir, after_dir, args.use_baseline)
    if os.path.samefile(before_dir, after_dir):
        sys.exit(f'기준 회차와 이번 회차가 같습니다: {after_dir}')

    before_lap, after_lap = load_lap(before_dir), load_lap(after_dir)
    before_stops = {s['id']: s for s in before_lap['stops']}
    out_dir = os.path.join(after_dir, f'compare_{os.path.basename(os.path.normpath(before_dir))}')
    os.makedirs(out_dir, exist_ok=True)
    print(f'기준 {before_dir}\n이번 {after_dir}')

    results = []
    for stop in after_lap['stops']:
        sid = stop['id']
        prev = before_stops.get(sid)
        if prev is None or prev.get('status') != 'ok' or stop.get('status') != 'ok':
            entry = {'status': 'skipped', 'reason': '두 회차 중 사진이 없는 쪽이 있음'}
        else:
            try:
                args.board = load_board_profile(after_lap['route'], sid)
                entry, picture = compare_stop(os.path.join(before_dir, prev['image']),
                                              os.path.join(after_dir, stop['image']), args)
            except BoardNotFound as exc:
                entry, picture = {'status': 'board_not_found', 'reason': str(exc)}, None
            if picture is not None:
                entry['image'] = f'{sid}.jpg'
                cv2.imwrite(os.path.join(out_dir, entry['image']), picture)
        entry = {'id': sid, **entry}
        results.append(entry)
        if entry['status'] == 'changed':
            print(f"  {sid}: {describe_changes(entry)} (정렬 {entry['align']})")
        elif entry['status'] == 'unchanged':
            print(f"  {sid}: 변화 없음 (정렬 {entry['align']})")
        else:
            print(f"  {sid}: {entry['status']}")

    summary = {
        'baseline': before_dir,
        'current': after_dir,
        'route': after_lap['route'],
        'params': {k: getattr(args, k) for k in ('delta_e', 'min_object', 'other_threshold', 'shift')},
        'stops': results,
    }
    with open(os.path.join(out_dir, 'compare.json'), 'w') as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(f'결과: {out_dir}')


if __name__ == '__main__':
    main()
