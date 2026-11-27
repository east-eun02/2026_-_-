"""관찰 기록 화면(기획안 그림 10): 순찰 결과를 브라우저에서 보고 관리자가 확인 결과를 남긴다.

patrol_manager가 남긴 <capture_dir>/<회차>/result.json·사진·<지점 ID>_compare.jpg를 그대로 읽는다.
관리자가 저장한 확인 결과·메모는 <capture_dir>/reviews.json 한 파일에 모은다(순찰 기록은 건드리지 않는다).

맨 위 '로봇 제어'에서 로봇 켜기·끄기, 순찰 시작·정지, 출발점 복귀, 노트북 끄기를 한다(robot_control).
평소에는 plant-review 서비스로 항상 켜 두고, 로봇은 이 화면에서 켠다(service/install_onboard_service.sh).

    ros2 run plant_observer_bringup review_server                 # http://<노트북 IP>:8080
    ros2 run plant_observer_bringup review_server --port 8000 --host 127.0.0.1   # 이 노트북에서만
    ros2 run plant_observer_bringup review_server --no-control    # 기록만 보기 (버튼 숨김)

같은 Wi-Fi의 휴대폰·PC 브라우저로 연다. ~/.config/plant_observer/panel_pin 파일에 숫자를 적어 두면
로봇 제어 버튼에 그 PIN을 묻는다. 없으면 같은 네트워크의 누구나 로봇을 움직일 수 있다.
순찰 기록은 ROS 없이 읽는다. 순찰 중이든 아니든 따로 켜고 끌 수 있다.
"""
import argparse
import csv
import hmac
import io
import json
import os
import re
import socket
import sys
import threading
import warnings
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from plant_observer_bringup.change_compare import CHANGE_TYPES, VERDICT_TEXT, describe_changes
from plant_observer_bringup.robot_control import ACTIONS, Controller

REVIEWS_FILE = 'reviews.json'
REVIEW_RESULTS = ['미확인', '확인 완료', '조치 필요', '오탐(변화 아님)', '재촬영 필요']
# 열매 생성은 자라는 기록, 나머지는 사람이 살펴봐야 할 변화 (기획안: 성장 기록과 확인 항목을 구분)
GROWTH_TYPES = {'fruit_added'}
COMPARE_TEXT = {
    'unchanged': '변화 없음',
    'no_reference': '기준 사진 없음',
    'align_failed': '비교 불가(정렬 실패)',
    'no_plant': '비교 불가(식물이 안 보임)',
    'board_not_found': '비교 불가(보드 경계 확인 필요)',
    'no_image': '비교 불가(사진 읽기 실패)',
    'error': '비교 오류',
    'view_mismatch': '비교 불가(촬영 위치·방향 다름)',
}
STOP_TEXT = {'nav_failed': '이동 실패', 'no_image': '사진 없음'}
LAP_NAME = re.compile(r'^\d{4}-\d{2}-\d{2}_\d{6}$')
IMAGE_PATH = re.compile(r'^/captures/(\d{4}-\d{2}-\d{2}_\d{6})/([\w.-]+\.(?:png|jpg))$')


def classify(stop):
    """지점 하나를 화면에 보일 (분류, 설명)으로. 분류: check(확인 필요), growth(성장 기록), ok, none."""
    final = stop.get('final')
    recovered = final and final.get('status') in ('changed', 'unchanged')
    if stop.get('status') != 'ok' and not recovered:
        return 'check', STOP_TEXT.get(stop.get('status'), stop.get('status', '?'))
    compare = stop.get('compare')
    if compare is None and not final:
        return 'none', '비교 안 함'
    compare = compare or {}
    result = final or compare
    if result.get('requires_review') or compare.get('requires_review'):
        text = (describe_changes(result) if result.get('status') == 'changed'
                else COMPARE_TEXT.get(result.get('status'), result.get('status', '판정 불가')))
        return 'check', f'모델 보조: {text} · 관리자 확인 필요'
    # 현재 정책은 정면 판정만 사용한다. 예전 기록의 재촬영 확정 표시도 읽을 수 있다.
    if final:
        verdict = VERDICT_TEXT.get(final.get('verdict'), final.get('verdict'))
        if final.get('verdict') == 'mismatch':
            return 'check', f"{describe_changes(final)} · {verdict}"
        if final.get('status') == 'changed':
            growth = set(final.get('changes', {})) <= GROWTH_TYPES
            return ('growth' if growth else 'check'), describe_changes(final) + (f' · {verdict}' if verdict else '')
        compare = {**compare, 'status': final.get('status')}
    status = compare.get('status')
    if status == 'changed':
        kinds = set(compare.get('changes') or [compare.get('change')])
        return ('growth' if kinds <= GROWTH_TYPES else 'check'), describe_changes(compare)
    if status == 'unchanged':
        return 'ok', '변화 없음'
    if status == 'no_reference':
        return 'none', '기준 사진 없음'
    return 'check', COMPARE_TEXT.get(status, status)


class Store:
    """회차 폴더와 reviews.json을 읽고 쓴다. 요청마다 다시 읽어 순찰 중에 생긴 회차도 바로 보인다."""

    def __init__(self, capture_dir):
        self.capture_dir = capture_dir
        self.lock = threading.Lock()

    def reviews(self):
        path = os.path.join(self.capture_dir, REVIEWS_FILE)
        if not os.path.isfile(path):
            return {}
        with open(path) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            raise ValueError('invalid reviews schema')
        return data

    def save_review(self, key, result, memo):
        with self.lock:
            reviews = self.reviews()
            reviews[key] = {'result': result, 'memo': memo,
                            'saved': datetime.now().isoformat(timespec='seconds')}
            path = os.path.join(self.capture_dir, REVIEWS_FILE)
            # 쓰다가 끊겨도 기존 기록이 깨지지 않게 임시 파일에 쓰고 바꿔 끼운다
            with open(path + '.tmp', 'w') as f:
                json.dump(reviews, f, ensure_ascii=False, indent=2)
            os.replace(path + '.tmp', path)
            return reviews[key]

    def baselines(self):
        result = {}
        for name in os.listdir(self.capture_dir):
            match = re.match(r'^baseline_(.+)\.txt$', name)
            if match:
                with open(os.path.join(self.capture_dir, name)) as f:
                    result[match.group(1)] = f.read().strip()
        return result

    def observations(self):
        """회차 × 지점 목록, 최신 회차 먼저."""
        try:
            reviews = self.reviews()
            if not isinstance(reviews, dict):
                raise ValueError('invalid reviews schema')
        except (OSError, ValueError) as exc:
            warnings.warn(f'reviews.json 읽기 실패 (원본 보존): {exc}')
            reviews = {}
        laps = sorted((d for d in os.listdir(self.capture_dir)
                       if LAP_NAME.match(d) and os.path.isfile(os.path.join(self.capture_dir, d, 'result.json'))),
                      reverse=True)
        items = []
        for lap in laps:
            try:
                with open(os.path.join(self.capture_dir, lap, 'result.json')) as f:
                    result = json.load(f)
                if not isinstance(result, dict) or not isinstance(result.get('stops', []), list):
                    raise ValueError('invalid lap schema')
            except (OSError, ValueError) as exc:
                warnings.warn(f'손상된 회차 제외: {lap}: {exc}')
                continue
            for stop in result.get('stops', []):
                if (not isinstance(stop, dict) or not isinstance(stop.get('id'), str)
                        or any(stop.get(key) is not None and not isinstance(stop[key], dict)
                               for key in ('compare', 'final', 'views', 'recheck'))):
                    warnings.warn(f'손상된 지점 제외: {lap}')
                    continue
                kind, text = classify(stop)
                if result.get('mode') == 'baseline':
                    kind = 'none' if result.get('baseline_complete') else 'check'
                    text = '3방향 기준 등록 완료' if result.get('baseline_complete') else '3방향 기준 등록 미완료'
                compare = stop.get('compare') or {}
                reference_lap = compare.get('reference_lap')
                key = f"{lap}/{stop['id']}"
                review = reviews.get(key)
                items.append({
                    'key': key,
                    'lap': lap,
                    'route': result.get('route'),
                    'compare_to': result.get('compare_to'),
                    'capture_policy': result.get('capture_policy'),
                    'started': result.get('started'),
                    'lap_status': ('진행 중 또는 중단됨' if not result.get('finished') else
                                   '복귀 완료' if result.get('home') == 'ok' else '복귀 실패'),
                    'stop': stop['id'],
                    'kind': kind,
                    'text': text,
                    'compare_status': compare.get('status'),
                    'change': compare.get('change'),
                    'count': compare.get('count'),
                    'objects': [{'type': o['type'], 'name': CHANGE_TYPES.get(o['type'], o['type']),
                                 'area_px': o.get('area_px')} for o in compare.get('objects', [])],
                    'align': compare.get('align'),
                    'analysis_source': compare.get('analysis_source', 'primary'),
                    'primary_status': compare.get('primary_status'),
                    'model_fallback': compare.get('model_fallback'),
                    'error_xy_m': stop.get('error_xy_m'),
                    'error_yaw_deg': stop.get('error_yaw_deg'),
                    'reference_lap': reference_lap,
                    'image': f"/captures/{lap}/{stop['image']}" if stop.get('image') else None,
                    'side_images': [{'view': view, 'image': f"/captures/{lap}/{record['image']}",
                                     'status': record.get('status'), 'purpose': record.get('purpose')}
                                    for view, record in stop.get('views', {}).items()
                                    if view in ('left', 'right') and record.get('image')],
                    'reference_image': (f"/captures/{reference_lap}/{compare.get('reference_image') or stop['id'] + '.png'}"
                                        if reference_lap else None),
                    'compare_image': f"/captures/{lap}/{compare['image']}" if compare.get('image') else None,
                    'verdict': (stop.get('final') or {}).get('verdict'),
                    'side_capture_status': (stop.get('recheck') or stop.get('overview') or {}).get('status'),
                    'recheck_compare_image': (
                        f"/captures/{lap}/{stop['recheck']['compare']['image']}"
                        if (stop.get('recheck') or {}).get('compare', {}).get('image') else None),
                    'review': review or {'result': '미확인', 'memo': '', 'saved': None},
                })
        return items

    def latest_lap(self):
        """가장 최근 회차 요약 (제어 패널의 순찰 상태 표시용)."""
        laps = sorted((d for d in os.listdir(self.capture_dir) if LAP_NAME.match(d)), reverse=True)
        for lap in laps:
            try:
                with open(os.path.join(self.capture_dir, lap, 'result.json')) as f:
                    result = json.load(f)
                stops = result.get('stops', [])
                return {'lap': lap, 'route': result.get('route'), 'finished': result.get('finished'),
                        'home': result.get('home'), 'stops': len(stops),
                        'failed': sum(1 for s in stops if isinstance(s, dict) and s.get('status') != 'ok')}
            except (OSError, ValueError, AttributeError):
                continue
        return None

    def image_path(self, lap, name):
        path = os.path.realpath(os.path.join(self.capture_dir, lap, name))
        root = os.path.realpath(self.capture_dir)
        return path if path.startswith(root + os.sep) and os.path.isfile(path) else None


def to_csv(items):
    out = io.StringIO()
    writer = csv.writer(out)
    writer.writerow(['회차', '루트', '지점', '분류', '판정', '기준 회차', '위치 오차(m)', '방향 오차(°)',
                     '확인 결과', '메모', '확인 시각'])
    kind_text = {'check': '확인 필요', 'growth': '성장 기록', 'ok': '변화 없음', 'none': '-'}
    for item in items:
        review = item['review']
        writer.writerow([item['lap'], item['route'], item['stop'], kind_text[item['kind']], item['text'],
                         item['reference_lap'] or '', item['error_xy_m'], item['error_yaw_deg'],
                         review['result'], review['memo'], review['saved'] or ''])
    # 엑셀에서 한글이 깨지지 않도록 BOM을 붙인다
    return '﻿' + out.getvalue()


def make_handler(store, controller=None, pin=None):

    class Handler(BaseHTTPRequestHandler):

        def log_message(self, fmt, *args):
            pass   # 사진 요청마다 찍히는 접속 로그는 끈다

        def send(self, status, body, content_type, extra=None):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(data)

        def send_json(self, obj, status=HTTPStatus.OK):
            self.send(status, json.dumps(obj, ensure_ascii=False), 'application/json; charset=utf-8')

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == '/':
                self.send(HTTPStatus.OK, PAGE, 'text/html; charset=utf-8')
            elif url.path == '/api/observations':
                self.send_json({'items': store.observations(), 'baselines': store.baselines(),
                                'results': REVIEW_RESULTS})
            elif url.path == '/api/control':
                if controller is None:
                    self.send_json({'enabled': False})
                else:
                    self.send_json({'enabled': True, 'pin_required': bool(pin), 'last_lap': store.latest_lap(),
                                    **controller.status()})
            elif url.path == '/api/export.csv':
                items = store.observations()
                route = parse_qs(url.query).get('route', [''])[0]
                if route:
                    items = [i for i in items if i['route'] == route]
                name = f"plant_review_{datetime.now():%Y%m%d_%H%M}.csv"
                self.send(HTTPStatus.OK, to_csv(items), 'text/csv; charset=utf-8',
                          {'Content-Disposition': f'attachment; filename="{name}"'})
            elif (match := IMAGE_PATH.match(url.path)) and (path := store.image_path(*match.groups())):
                with open(path, 'rb') as f:
                    data = f.read()
                self.send(HTTPStatus.OK, data, 'image/png' if path.endswith('.png') else 'image/jpeg')
            else:
                self.send(HTTPStatus.NOT_FOUND, 'not found', 'text/plain; charset=utf-8')

        def do_POST(self):
            if urlparse(self.path).path == '/api/control':
                self.control()
                return
            if urlparse(self.path).path != '/api/review':
                self.send(HTTPStatus.NOT_FOUND, 'not found', 'text/plain; charset=utf-8')
                return
            try:
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))))
                key, result, memo = body['key'], body['result'], str(body.get('memo', ''))[:2000]
            except (ValueError, KeyError, TypeError):
                self.send_json({'error': '잘못된 요청'}, HTTPStatus.BAD_REQUEST)
                return
            lap, _, stop = key.partition('/')
            if (result not in REVIEW_RESULTS or not LAP_NAME.match(lap)
                    or not os.path.isfile(os.path.join(store.capture_dir, lap, 'result.json'))
                    or not re.match(r'^[\w-]+$', stop)):
                self.send_json({'error': '잘못된 요청'}, HTTPStatus.BAD_REQUEST)
                return
            try:
                saved = store.save_review(key, result, memo)
            except (OSError, ValueError):
                self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR,
                                'Cannot save review; existing reviews.json was preserved')
                return
            self.send_json(saved)

        def control(self):
            # JSON만 받는다: 다른 사이트의 폼이 이 노트북으로 몰래 명령을 보내지 못하게 (브라우저 사전 확인)
            if controller is None or not self.headers.get('Content-Type', '').startswith('application/json'):
                self.send_json({'message': '잘못된 요청'}, HTTPStatus.BAD_REQUEST)
                return
            if pin and not hmac.compare_digest(self.headers.get('X-Panel-Pin', ''), pin):
                self.send_json({'message': 'PIN이 맞지 않습니다'}, HTTPStatus.FORBIDDEN)
                return
            try:
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))))
                action, laps = body['action'], int(body.get('laps', 1))
            except (ValueError, KeyError, TypeError):
                self.send_json({'message': '잘못된 요청'}, HTTPStatus.BAD_REQUEST)
                return
            if action not in ACTIONS:
                self.send_json({'message': '알 수 없는 동작입니다'}, HTTPStatus.BAD_REQUEST)
                return
            ok, message = controller.run(action, laps)
            self.send_json({'ok': ok, 'message': message}, HTTPStatus.OK if ok else HTTPStatus.CONFLICT)

    return Handler


def lan_address():
    """다른 기기에서 접속할 때 쓸 이 노트북의 IP (인터넷에 실제로 보내지는 않는다)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('10.255.255.255', 1))
            return s.getsockname()[0]
    except OSError:
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--capture-dir', default='~/plant_ws/captures')
    parser.add_argument('--host', default='0.0.0.0', help='127.0.0.1이면 이 노트북에서만 열린다 (기본 %(default)s)')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--route', default='demo_real3', help='순찰 시작 버튼이 도는 루트 (기본 %(default)s)')
    parser.add_argument('--max-laps', type=int, default=5, help='한 번에 고를 수 있는 최대 회차 수')
    parser.add_argument('--pin-file', default='~/.config/plant_observer/panel_pin',
                        help='있으면 로봇 제어 버튼에 이 PIN을 묻는다 (기본 %(default)s)')
    parser.add_argument('--no-control', action='store_true', help='로봇 제어 버튼 없이 기록만 보여 준다')
    # ros2 launch의 Node로 띄우면 뒤에 --ros-args ...가 붙는다. ROS 노드가 아니라 그 뒤는 버린다
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = parser.parse_args(argv)
    capture_dir = os.path.expanduser(args.capture_dir)
    # 부팅 직후 자동 시작하면 아직 순찰 기록이 없을 수 있다. 빈 목록으로 띄우고 순찰이 같은 폴더에 쌓는다
    os.makedirs(capture_dir, exist_ok=True)

    pin = None
    pin_path = os.path.expanduser(args.pin_file)
    if os.path.isfile(pin_path):
        with open(pin_path) as f:
            pin = f.read().strip() or None
    controller = None if args.no_control else Controller(args.route, args.max_laps)

    server = ThreadingHTTPServer((args.host, args.port), make_handler(Store(capture_dir), controller, pin))
    # ros2 run으로 띄우면 출력이 버퍼에 쌓여 안 보이므로 바로 내보낸다
    print(f'관찰 기록 화면: http://localhost:{args.port}', flush=True)
    if args.host == '0.0.0.0' and (ip := lan_address()):
        print(f'같은 Wi-Fi의 다른 기기: http://{ip}:{args.port}', flush=True)
    print(f'기록 폴더 {capture_dir} · 끄려면 Ctrl-C', flush=True)
    if controller is not None:
        print(f"로봇 제어: 루트 {args.route} · {'PIN 사용' if pin else 'PIN 없음 (' + pin_path + ' 에 적으면 사용)'}",
              flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


PAGE = r"""<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>식물 관찰 기록</title>
<style>
:root {
  --bg: #f6f7f5; --panel: #ffffff; --text: #1d2420; --muted: #66706a; --line: #dde2dd;
  --accent: #2f7d4f; --check: #b4462f; --check-bg: #fbe9e5; --growth: #8a5a00; --growth-bg: #fdf1d8;
  --ok: #2f7d4f; --ok-bg: #e6f3ea; --none-bg: #eceeec; --sel: #e9f1ec;
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #141816; --panel: #1c211e; --text: #e6ebe7; --muted: #9aa49e; --line: #2e3531;
    --accent: #6cc28e; --check: #f0907c; --check-bg: #3a221d; --growth: #f2c66d; --growth-bg: #392d14;
    --ok: #6cc28e; --ok-bg: #1d3226; --none-bg: #262b28; --sel: #233029;
  }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--text);
  font: 15px/1.5 system-ui, -apple-system, "Noto Sans KR", "Malgun Gothic", sans-serif; }
header { display: flex; flex-wrap: wrap; gap: 8px 16px; align-items: center; padding: 12px 16px;
  border-bottom: 1px solid var(--line); background: var(--panel); position: sticky; top: 0; z-index: 2; }
header h1 { font-size: 18px; margin: 0 auto 0 0; }
.controls { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
select, input, textarea, button { font: inherit; color: var(--text); background: var(--panel);
  border: 1px solid var(--line); border-radius: 6px; padding: 5px 8px; }
button { cursor: pointer; }
button.primary { background: var(--accent); color: #fff; border-color: var(--accent); }
a.button { color: var(--text); text-decoration: none; border: 1px solid var(--line); border-radius: 6px;
  padding: 5px 10px; background: var(--panel); }
main { display: grid; grid-template-columns: 300px 1fr; min-height: calc(100vh - 58px); }
#list { border-right: 1px solid var(--line); overflow-y: auto; max-height: calc(100vh - 58px);
  position: sticky; top: 58px; }
.lap { padding: 10px 14px 4px; font-size: 13px; color: var(--muted); }
.item { display: flex; gap: 8px; align-items: center; padding: 8px 14px; cursor: pointer;
  border-left: 3px solid transparent; }
.item:hover { background: var(--sel); }
.item.active { background: var(--sel); border-left-color: var(--accent); }
.item .stop { font-weight: 600; width: 38px; }
.item .text { flex: 1; font-size: 14px; }
.badge { font-size: 12px; border-radius: 10px; padding: 1px 8px; white-space: nowrap; }
.k-check { color: var(--check); background: var(--check-bg); }
.k-growth { color: var(--growth); background: var(--growth-bg); }
.k-ok { color: var(--ok); background: var(--ok-bg); }
.k-none { color: var(--muted); background: var(--none-bg); }
.r-unreviewed { color: var(--muted); border: 1px dashed var(--line); }
.r-done { color: var(--ok); background: var(--ok-bg); }
#detail { padding: 16px; min-width: 0; }
#detail h2 { margin: 0 0 4px; font-size: 18px; }
.meta { color: var(--muted); font-size: 13px; margin-bottom: 12px; }
.photos { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
figure { margin: 0; background: var(--panel); border: 1px solid var(--line); border-radius: 8px; overflow: hidden; }
figure img { display: block; width: 100%; height: auto; background: var(--none-bg); }
figcaption { padding: 6px 10px; font-size: 13px; color: var(--muted); }
.wide { margin-top: 12px; }
table { width: 100%; border-collapse: collapse; margin-top: 12px; background: var(--panel);
  border: 1px solid var(--line); border-radius: 8px; overflow: hidden; }
th, td { text-align: left; padding: 7px 10px; border-bottom: 1px solid var(--line); font-size: 14px; }
th { color: var(--muted); font-weight: 500; }
.review { margin-top: 12px; padding: 12px; background: var(--panel); border: 1px solid var(--line);
  border-radius: 8px; display: grid; gap: 8px; }
.review textarea { width: 100%; min-height: 60px; resize: vertical; }
.row { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
.saved { color: var(--muted); font-size: 13px; }
.empty { color: var(--muted); padding: 24px; }
#control { padding: 12px 16px; background: var(--panel); border-bottom: 1px solid var(--line);
  display: grid; gap: 10px; }
#control .row strong { min-width: 44px; }
#control button { padding: 7px 14px; }
#control button:disabled { opacity: .45; cursor: default; }
button.danger { background: var(--check); color: #fff; border-color: var(--check); }
.checks { display: flex; flex-wrap: wrap; gap: 4px; }
.notice { font-size: 14px; }
.notice.error { color: var(--check); }
button.quiet { margin-left: auto; color: var(--muted); font-size: 13px; }
@media (max-width: 760px) {
  main { grid-template-columns: 1fr; }
  #list { position: static; max-height: 40vh; border-right: 0; border-bottom: 1px solid var(--line); }
  .photos { grid-template-columns: 1fr; }
  #control button { flex: 1 1 auto; padding: 10px 12px; }
  #control button.quiet { flex: 0 0 auto; padding: 6px 10px; }
}
</style>
</head>
<body>
<header>
  <h1>식물 관찰 기록</h1>
  <div class="controls">
    <select id="route" aria-label="루트"></select>
    <select id="filter" aria-label="보기">
      <option value="all">전체</option>
      <option value="attention">변화·확인 필요만</option>
      <option value="unreviewed">미확인 변화만</option>
    </select>
    <button id="reload" type="button">새로고침</button>
    <a class="button" id="csv" href="/api/export.csv">기록 내려받기</a>
  </div>
</header>
<section id="control" hidden>
  <div class="row">
    <strong>로봇</strong><span id="robotState" class="badge k-none">-</span>
    <span id="checks" class="checks"></span>
  </div>
  <div class="row">
    <button id="robotStart" class="primary" type="button">로봇 켜기</button>
    <button id="robotStop" type="button">로봇 끄기</button>
  </div>
  <div class="row">
    <strong>순찰</strong><span id="patrolState" class="badge k-none">-</span>
    <span id="lastLap" class="saved"></span>
  </div>
  <div class="row">
    <select id="laps" aria-label="회차 수"></select>
    <button id="patrolStart" class="primary" type="button">순찰 시작</button>
    <button id="patrolStop" class="danger" type="button">정지</button>
    <button id="home" type="button">출발점 복귀</button>
  </div>
  <div class="row">
    <span id="controlMsg" class="notice"></span>
    <button id="poweroff" class="quiet" type="button">노트북 끄기</button>
  </div>
</section>
<main>
  <nav id="list"></nav>
  <section id="detail"><p class="empty">왼쪽에서 관찰 기록을 고르세요.</p></section>
</main>
<script>
const KIND = { check: '확인 필요', growth: '성장 기록', ok: '변화 없음', none: '-' };
let data = { items: [], baselines: {}, results: [] };
let selected = null;

const $ = (id) => document.getElementById(id);
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const lapTime = (lap) => lap.replace(/^(\d{4})-(\d{2})-(\d{2})_(\d{2})(\d{2})(\d{2})$/, '$2/$3 $4:$5');
const needsLook = (i) => i.kind === 'check' || i.kind === 'growth';

function remember(key, value) { try { localStorage.setItem(key, value); } catch (e) {} }
function recall(key) { try { return localStorage.getItem(key); } catch (e) { return null; } }

// 처음 열 때와 순찰이 끝났을 때(newest)는 가장 최근 회차의 루트를 보여 준다. 목록이 최신 회차 순이라 routes[0]
async function load(newest) {
  const res = await fetch('/api/observations');
  data = await res.json();
  const routes = [...new Set(data.items.map((i) => i.route))];
  const current = (!newest && $('route').value) || routes[0] || '';
  $('route').innerHTML = routes.map((r) => `<option value="${esc(r)}">${esc(r)}</option>`).join('');
  $('route').value = routes.includes(current) ? current : (routes[0] || '');
  render();
}

function visible() {
  const route = $('route').value, filter = $('filter').value;
  return data.items.filter((i) => i.route === route
    && (filter === 'all' || (filter === 'attention' && needsLook(i))
        || (filter === 'unreviewed' && needsLook(i) && i.review.result === '미확인')));
}

function render() {
  const items = visible();
  $('csv').href = '/api/export.csv?route=' + encodeURIComponent($('route').value);
  const baseline = data.baselines[$('route').value];
  let html = '', lastLap = null;
  for (const i of items) {
    if (i.lap !== lastLap) {
      html += `<div class="lap">${lapTime(i.lap)}${i.lap === baseline ? ' · 기준 회차' : ''}</div>`;
      lastLap = i.lap;
    }
    const reviewed = i.review.result !== '미확인';
    html += `<div class="item${i.key === selected ? ' active' : ''}" data-key="${esc(i.key)}">
      <span class="stop">${esc(i.stop)}</span>
      <span class="text">${esc(i.text)}</span>
      ${needsLook(i) ? `<span class="badge ${reviewed ? 'r-done' : 'r-unreviewed'}">${esc(reviewed ? i.review.result : '미확인')}</span>` : ''}
    </div>`;
  }
  $('list').innerHTML = html || '<p class="empty">조건에 맞는 기록이 없습니다.</p>';
  if (!items.some((i) => i.key === selected)) selected = (items.find(needsLook) || items[0] || {}).key || null;
  renderDetail();
  for (const el of document.querySelectorAll('.item')) el.classList.toggle('active', el.dataset.key === selected);
}

function photo(src, caption) {
  return `<figure>${src ? `<img src="${esc(src)}" alt="${esc(caption)}" loading="lazy">` : '<p class="empty">사진 없음</p>'}
    <figcaption>${esc(caption)}</figcaption></figure>`;
}

function renderDetail() {
  const i = data.items.find((x) => x.key === selected);
  if (!i) { $('detail').innerHTML = '<p class="empty">왼쪽에서 관찰 기록을 고르세요.</p>'; return; }
  const rows = {};
  for (const o of i.objects) rows[o.name] = (rows[o.name] || 0) + 1;
  const table = Object.keys(rows).length
    ? `<table><tr><th>변화 항목</th><th>개수</th><th>기록 구분</th></tr>${Object.entries(rows).map(([name, n]) => {
        const growth = i.objects.find((o) => o.name === name).type === 'fruit_added' && i.analysis_source !== 'model_fallback';
        return `<tr><td>${esc(name)}</td><td>${n}</td><td><span class="badge ${growth ? 'k-growth' : 'k-check'}">${growth ? '성장 기록' : '확인 필요'}</span></td></tr>`;
      }).join('')}</table>`
    : '';
  const err = i.error_xy_m != null ? ` · 도착 오차 ${i.error_xy_m}m / ${i.error_yaw_deg}°` : '';
  const referenceLabel = i.compare_to === 'previous' ? '직전 순찰 사진' : '기준 사진';
  $('detail').innerHTML = `
    <h2>${esc(i.stop)} <span class="badge k-${i.kind}">${KIND[i.kind]}</span> ${esc(i.text)}</h2>
    <div class="meta">이번 ${lapTime(i.lap)}${i.reference_lap ? ` · 기준 ${lapTime(i.reference_lap)}` : ''}${err}${i.align ? ` · 정렬 ${esc(i.align)}` : ''}${i.lap_status ? ` · ${esc(i.lap_status)}` : ''}</div>
    ${i.analysis_source === 'model_fallback' ? '<div class="meta">기존 사진 비교가 불가능하여 모델이 찾은 식물 영역으로 보조 분석했습니다. 변화 없음 결과도 직접 확인해주세요.</div>' : ''}
    ${i.model_fallback && i.model_fallback.status === 'failed' ? '<div class="meta">모델 보조로도 판정하지 못했습니다. 원래 비교 실패 상태를 유지합니다.</div>' : ''}
    <div class="photos">
      ${photo(i.reference_image, referenceLabel + (i.reference_lap ? ` (${lapTime(i.reference_lap)})` : ''))}
      ${photo(i.image, `이번 사진 (${lapTime(i.lap)})`)}
    </div>
    ${i.compare_image ? `<div class="wide">${photo(i.compare_image, '비교: 기준 | 이번(정렬) | 변화 표시')}</div>` : ''}
    ${(i.side_images || []).length ? `<div class="photos">${i.side_images.map(v => photo(v.image, (v.view === 'left' ? '왼쪽 촬영' : '오른쪽 촬영') + (v.purpose === 'admin_review' ? ' · 관리자 확인용' : ''))).join('')}</div>` : ''}
    ${i.capture_policy === 'front_compare_sides_for_admin' && i.side_capture_status && i.side_capture_status !== 'ok' ? `<div class="meta">관리자용 추가 촬영 미완료 (${esc(i.side_capture_status)}) · 판정은 정면 사진 기준</div>` : ''}
    ${i.recheck_compare_image ? `<div class="wide">${photo(i.recheck_compare_image, '추가 재촬영 비교: 이전 | 재촬영(정렬) | 변화 표시')}</div>` : ''}
    ${table}
    <form class="review" id="reviewForm">
      <div class="row">
        <label for="result">확인 결과</label>
        <select id="result">${data.results.map((r) => `<option${r === i.review.result ? ' selected' : ''}>${esc(r)}</option>`).join('')}</select>
        <button class="primary" type="submit">저장</button>
        <span class="saved" id="saved">${i.review.saved ? '최근 저장 ' + esc(i.review.saved.replace('T', ' ')) : ''}</span>
      </div>
      <textarea id="memo" placeholder="관찰 메모">${esc(i.review.memo)}</textarea>
    </form>`;
  $('reviewForm').addEventListener('submit', save);
}

async function save(event) {
  event.preventDefault();
  const body = { key: selected, result: $('result').value, memo: $('memo').value };
  const res = await fetch('/api/review', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  if (!res.ok) { $('saved').textContent = '저장 실패'; return; }
  const review = await res.json();
  data.items.find((x) => x.key === selected).review = review;
  render();
}

$('list').addEventListener('click', (e) => {
  const el = e.target.closest('.item');
  if (el) { selected = el.dataset.key; render(); }
});
$('route').addEventListener('change', () => { selected = null; render(); });
$('filter').addEventListener('change', () => { remember('filter', $('filter').value); render(); });
$('reload').addEventListener('click', () => load());
$('filter').value = recall('filter') || 'all';
load();

// ── 로봇 제어 ──
const UNIT = { active: '켜짐', activating: '켜는 중', deactivating: '끄는 중', inactive: '꺼짐', failed: '오류로 꺼짐' };
const UNIT_KIND = { active: 'k-ok', activating: 'k-growth', deactivating: 'k-growth', failed: 'k-check' };
const CHECKS = [['wheels', '바퀴'], ['lidar', '라이다'], ['camera', '카메라'], ['nav2', 'Nav2'], ['localized', '위치']];
const RUNNING = ['active', 'activating', 'deactivating', 'reloading'];
let control = null, lastPatrol = null;

function message(text, error) {
  $('controlMsg').textContent = text || '';
  $('controlMsg').classList.toggle('error', !!error);
}

function patrolText(c) {
  const p = c.patrol;
  if (p.state === 'active') return p.mode === 'home' ? '출발점 복귀 중' : `순찰 중 (${p.laps}회)`;
  if (p.state === 'activating') return '시작 중';
  if (p.state === 'deactivating') return '멈추는 중';
  if (p.state === 'failed') return '실패 (로그 확인)';
  return '대기';
}

function renderControl() {
  const c = control;
  if (!c.enabled) return;
  $('control').hidden = false;
  const robot = c.robot.state, patrol = c.patrol.state;
  $('robotState').textContent = c.robot.installed ? (UNIT[robot] || robot) : '서비스 미설치';
  $('robotState').className = 'badge ' + (UNIT_KIND[robot] || 'k-none');
  $('checks').innerHTML = robot !== 'active' ? ''
    : !c.ros || !c.ros.available ? '<span class="badge k-none">상태 확인 불가</span>'
    : CHECKS.map(([k, name]) => `<span class="badge ${c.ros[k] ? 'k-ok' : 'k-check'}">${c.ros[k] ? '✓' : '✗'} ${name}</span>`).join('');
  $('patrolState').textContent = patrolText(c);
  $('patrolState').className = 'badge ' + (RUNNING.includes(patrol) ? 'k-growth' : patrol === 'failed' ? 'k-check' : 'k-none');
  const l = c.last_lap;
  $('lastLap').textContent = l ? `최근 ${lapTime(l.lap)} · ${l.stops}곳${l.failed ? ` (실패 ${l.failed})` : ''} · ${
    !l.finished ? '끝나지 않음' : l.home === 'ok' ? '복귀 완료' : '복귀 실패'}` : '';
  if ($('laps').options.length !== c.max_laps) {
    $('laps').innerHTML = Array.from({ length: c.max_laps }, (_, i) => `<option value="${i + 1}">${i + 1}회</option>`).join('');
  }
  const ready = robot === 'active' && c.ros && c.ros.nav2 && c.ros.localized;
  const busy = !!c.busy || !c.robot.installed;
  $('robotStart').disabled = busy || RUNNING.includes(robot);
  $('robotStop').disabled = busy || !['active', 'activating'].includes(robot);
  $('patrolStart').disabled = $('home').disabled = busy || !ready || RUNNING.includes(patrol);
  $('patrolStop').disabled = busy || !['active', 'activating'].includes(patrol);
  $('poweroff').hidden = !['yes', 'challenge'].includes(c.can_poweroff);
  $('poweroff').disabled = busy;
  if (c.busy) message(c.busy + '…');
  else if (c.notice) message(c.notice, true);
  else if (robot === 'active' && !ready && !RUNNING.includes(patrol)) message('로봇 준비 중입니다. Nav2와 위치가 ✓가 되면 순찰할 수 있습니다');
  // 순찰이 끝나면 새 회차가 보이도록 기록을 다시 읽는다
  if (lastPatrol && RUNNING.includes(lastPatrol) && !RUNNING.includes(patrol)) load(true);
  lastPatrol = patrol;
}

async function refreshControl() {
  try {
    const res = await fetch('/api/control');
    if (!res.ok) return;
    control = await res.json();
    renderControl();
  } catch (e) {
    if (control && control.enabled) message('노트북과 연결이 끊겼습니다. 핫스팟 연결을 확인하세요', true);
  }
}

async function act(action, confirmText) {
  if (confirmText && !confirm(confirmText)) return;
  const headers = { 'Content-Type': 'application/json' };
  const pin = recall('pin');
  if (pin) headers['X-Panel-Pin'] = pin;
  let res;
  try {
    res = await fetch('/api/control', { method: 'POST', headers, body: JSON.stringify({ action, laps: +$('laps').value || 1 }) });
  } catch (e) { message('노트북과 연결이 끊겼습니다', true); return; }
  if (res.status === 403) {
    const typed = prompt('로봇 제어 PIN');
    if (!typed) return;
    remember('pin', typed);
    return act(action);
  }
  const body = await res.json().catch(() => ({ message: '응답 오류' }));
  message(body.message, !res.ok);
  refreshControl();
}

$('robotStart').addEventListener('click', () => act('robot_start'));
$('robotStop').addEventListener('click', () => act('robot_stop', '로봇을 끌까요? 순찰 중이면 먼저 멈춥니다.'));
$('patrolStart').addEventListener('click', () => act('patrol_start'));
$('patrolStop').addEventListener('click', () => act('patrol_stop'));
$('home').addEventListener('click', () => act('home'));
$('poweroff').addEventListener('click', () => act('poweroff', '로봇을 끄고 노트북 전원을 끌까요? 다시 켜려면 노트북 전원 버튼을 눌러야 합니다.'));
refreshControl();
setInterval(refreshControl, 3000);
</script>
</body>
</html>
"""


if __name__ == '__main__':
    main()
