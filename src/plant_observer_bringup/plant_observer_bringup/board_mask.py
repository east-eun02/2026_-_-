"""Registered board boundaries for front photos; never a change-detection baseline.

Only features inside the registered board locate it in each new photo. A missing,
cropped or unreliable board is a comparison failure, never a full-frame fallback.
"""
from pathlib import Path

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


class BoardNotFound(ValueError):
    pass


def load_board_profile(route, stop_id):
    path = Path(route).expanduser()
    if not path.is_file():
        path = Path(get_package_share_directory('plant_observer_gazebo')) / 'config' / f'patrol_{route}.yaml'
    try:
        config = yaml.safe_load(path.read_text())
        if not isinstance(config, dict) or config.get('comparison_scope', 'plant') not in ('plant', 'board'):
            raise BoardNotFound('비교 영역 설정 오류')
        if config.get('comparison_scope', 'plant') == 'plant':
            return None
        profile = dict(config['comparison_boards'][stop_id])
        profile['image'] = str(path.parent / profile['image'])
        return profile
    except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
        raise BoardNotFound(f'보드 경계 설정을 읽을 수 없음: {path} / {stop_id}') from exc


def polygon_mask(shape, polygon):
    mask = np.zeros(shape[:2], np.uint8)
    cv2.fillPoly(mask, [np.rint(polygon).astype(np.int32)], 255)
    return mask


def isolate(image, mask):
    """Remove exterior pixels before blur, resampling, normalization or alignment."""
    result = image.copy()
    result[mask == 0] = 225
    return result


def locate_board(image, profile):
    """Return a conservative interior mask and diagnostic coordinates in image pixels."""
    try:
        anchor = cv2.imread(str(profile['image']))
        polygon = np.asarray(profile['polygon'], np.float32)
    except (KeyError, TypeError, ValueError) as exc:
        raise BoardNotFound('보드 경계 설정 오류') from exc
    if anchor is None:
        raise BoardNotFound('보드 위치 등록 사진 없음')
    if (polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3
            or not np.isfinite(polygon).all()
            or not cv2.isContourConvex(polygon)):
        raise BoardNotFound('보드 경계는 볼록한 다각형이어야 함')
    h, w = anchor.shape[:2]
    if (polygon.min() < 0 or (polygon[:, 0] >= w).any() or (polygon[:, 1] >= h).any()):
        raise BoardNotFound('등록 보드 경계가 사진 밖에 있음')
    scale = w / image.shape[1]
    current = cv2.resize(image, (w, round(image.shape[0] * scale)), interpolation=cv2.INTER_AREA)
    anchor_mask = polygon_mask(anchor.shape, polygon)
    # Four pyramid levels use a descriptor support radius < 28px. The 31px
    # margin keeps the descriptors entirely inside the registered board.
    interior = cv2.erode(anchor_mask, np.ones((63, 63), np.uint8))
    orb = cv2.ORB_create(nfeatures=8000, nlevels=4)
    key_a, desc_a = orb.detectAndCompute(cv2.cvtColor(isolate(anchor, anchor_mask), cv2.COLOR_BGR2GRAY), interior)
    if desc_a is None or len(key_a) < 25:
        raise BoardNotFound('등록 보드 안쪽 특징점 부족')

    def fit(target, search):
        key_b, desc_b = orb.detectAndCompute(cv2.cvtColor(target, cv2.COLOR_BGR2GRAY), search)
        if desc_b is None or len(key_b) < 25:
            raise BoardNotFound('현재 사진에서 보드를 찾지 못함')
        pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(desc_a, desc_b, k=2)
        matches = [p[0] for p in pairs if len(p) == 2 and p[0].distance < 0.7 * p[1].distance]
        # Each target point can support the board geometry only once.
        matches = list({m.trainIdx: m for m in sorted(matches, key=lambda m: -m.distance)}.values())
        if len(matches) < 25:
            raise BoardNotFound('보드 특징점 대응 부족')
        source = np.float32([key_a[m.queryIdx].pt for m in matches])
        dest = np.float32([key_b[m.trainIdx].pt for m in matches])
        transform, good = cv2.findHomography(source, dest, cv2.RANSAC, 3.0)
        count = int(good.sum()) if good is not None else 0
        if transform is None or not np.isfinite(transform).all() or count < 25 or count / len(matches) < 0.45:
            raise BoardNotFound('보드 위치 추정 신뢰도 부족')
        support = source[good.ravel().astype(bool)]
        coverage = cv2.contourArea(cv2.convexHull(support)) / cv2.contourArea(polygon)
        if coverage < 0.18:
            raise BoardNotFound('보드의 작은 일부만 일치함')
        corners = cv2.perspectiveTransform(polygon[None], transform)[0]
        ratio = cv2.contourArea(corners, oriented=True) / cv2.contourArea(polygon, oriented=True)
        if (not np.isfinite(corners).all() or not cv2.isContourConvex(corners)
                or not 0.55 < ratio < 1.8 or corners.min() < 2
                or (corners[:, 0] >= current.shape[1] - 2).any()
                or (corners[:, 1] >= current.shape[0] - 2).any()):
            raise BoardNotFound('보드가 잘렸거나 위치·크기 변화가 너무 큼')
        edges = np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)
        original_edges = np.linalg.norm(np.roll(polygon, -1, axis=0) - polygon, axis=1)
        edge_scales = edges / original_edges
        if edge_scales.min() < 0.55 or edge_scales.max() > 1.6 or edge_scales.max() / edge_scales.min() > 1.6:
            raise BoardNotFound('보드 경계 변형이 너무 큼')
        return corners, count, coverage

    corners, _, _ = fit(current, None)
    # Refine exclusively on the located board, excluding exterior texture from
    # feature allocation and descriptor patches in the final estimate.
    projected = polygon_mask(current.shape, corners)
    search = cv2.erode(projected, np.ones((63, 63), np.uint8))
    refined, count, coverage = fit(isolate(current, projected), search)
    if np.max(np.linalg.norm(refined - corners, axis=1)) > 8:
        raise BoardNotFound('보드 위치 추정이 안정적이지 않음')
    projected = polygon_mask(current.shape, refined)
    # Keep 6px inside the registered boundary, absorbing localization/blur error.
    projected = cv2.erode(projected, np.ones((13, 13), np.uint8))
    mask = cv2.resize(projected, image.shape[1::-1], interpolation=cv2.INTER_NEAREST)
    return mask, {
        'polygon': np.round(refined / scale, 1).tolist(),
        'inset_px': round(6 / scale, 1),
        'feature_inliers': count,
        'feature_coverage': round(coverage, 3),
    }
