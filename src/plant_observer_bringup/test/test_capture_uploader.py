"""순찰 중인 회차를 올리지 않고, 예전에 덜 올라간 회차는 다시 올린다."""
import json
import os
import time

from plant_observer_bringup import capture_uploader as cu


def write_lap(root, name, finished, age):
    folder = root / name
    folder.mkdir()
    result = {'route': 'demo', 'stops': []}
    if finished:
        result['finished'] = '2026-10-30T20:28:00'
    path = folder / 'result.json'
    path.write_text(json.dumps(result))
    (folder / 'P00.png').write_bytes(b'image')
    stamp = time.time() - age
    os.utime(path, (stamp, stamp))
    return folder


def test_running_lap_is_not_uploaded(tmp_path):
    write_lap(tmp_path, '2026-10-30_202412', finished=False, age=30)
    write_lap(tmp_path, '2026-10-30_203302', finished=True, age=30)
    write_lap(tmp_path, '2026-10-30_190000', finished=False, age=cu.ABANDONED_SECONDS + 1)
    assert cu.finished_laps(str(tmp_path)) == ['2026-10-30_190000', '2026-10-30_203302']


def test_partial_upload_is_detected(tmp_path):
    source = write_lap(tmp_path, '2026-10-30_202412', finished=True, age=30)
    target = tmp_path / 'repo' / '2026-10-30_202412'
    target.mkdir(parents=True)
    (target / 'result.json').write_text(json.dumps({'route': 'demo', 'stops': []}))
    assert not cu.same_file(str(source / 'result.json'), str(target / 'result.json'))
    cu.copy_lap(str(tmp_path), str(tmp_path / 'repo'), '2026-10-30_202412')
    assert cu.same_file(str(source / 'result.json'), str(target / 'result.json'))
    assert (target / 'P00.png').exists()
