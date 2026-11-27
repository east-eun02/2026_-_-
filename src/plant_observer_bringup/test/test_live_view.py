"""Live view conversions: map image orientation, lidar points in map frame, path thinning."""
import math

import cv2
import numpy as np

from plant_observer_bringup import live_view as lv


def test_map_png_puts_positive_y_at_top_and_keeps_rviz_shades():
    # 2 x 3 grid, row 0 is the bottom of the map: wall at bottom-left, unknown at top-right
    data = [100, 0, 0,
            0, 0, -1]
    image = cv2.imdecode(np.frombuffer(lv.map_png(3, 2, data), np.uint8), cv2.IMREAD_GRAYSCALE)
    assert image.shape == (2, 3)
    assert image[1, 0] == 0          # wall, drawn on the bottom row
    assert image[0, 2] == 205        # unknown, top row
    assert image[0, 0] == 254        # free


def test_scan_points_use_sensor_pose_and_drop_invalid_ranges():
    points = lv.scan_points([1.0, float('inf'), 0.1, 2.0], angle_min=0., angle_increment=math.pi / 2,
                            range_min=0.2, range_max=8., sensor_pose=(1., 2., math.pi))
    # laser yaw π (mounted backwards): a beam at 0 rad points to -x in the map
    assert np.allclose(points, [[0., 2.], [1., 4.]], atol=1e-3)


def test_scan_points_are_thinned():
    points = lv.scan_points([1.] * 2000, -math.pi, 2 * math.pi / 2000, 0.1, 8., (0., 0., 0.))
    assert len(points) == lv.MAX_SCAN_POINTS


def test_path_points_keep_both_ends():
    xy = [(i * 0.01, 0.) for i in range(1000)]
    thinned = lv.path_points(xy)
    assert len(thinned) <= lv.MAX_PATH_POINTS
    assert thinned[0] == [0., 0.] and thinned[-1] == [9.99, 0.]


def test_load_route_reads_stops_and_rotation():
    route = lv.load_route('demo_real3_artificial')
    assert route['image_rotation'] == 90 and [s['id'] for s in route['stops']] == ['P00', 'P01']
    assert lv.load_route('no_such_route') is None
