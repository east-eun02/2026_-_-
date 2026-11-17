"""순찰 기록을 팀 GitHub 저장소에 자동으로 올린다. 관리자 사이트(팀원 구현)는 그 저장소를 읽는다.

<capture_dir>을 지켜보다가 끝난 회차(result.json이 생긴 폴더)를 저장소의 captures/<회차>/로 복사하고,
사이트가 폴더를 뒤지지 않아도 되게 captures/index.json(회차 목록·지점별 판정)을 다시 만든 뒤 commit·push한다.
patrol_manager가 회차 저장 완료 알림을 보내면 바로 업로드한다.
따로 도는 프로그램이라, 인터넷이 끊기거나 push가 실패해도 순찰은 멈추지 않는다.
알림을 놓쳤거나 못 올린 commit은 30초마다 다시 확인한다.

저장소는 미리 clone해 둔다(쓰기 권한이 있는 배포 키 사용 권장):
    git clone git@github.com:<팀>/<저장소>.git ~/plant_ws/upload_repo

    ros2 run plant_observer_bringup capture_uploader            # 회차 완료 즉시 올림 (Ctrl-C로 끝)
    ros2 run plant_observer_bringup capture_uploader --once     # 한 번만 올리고 끝
"""
import argparse
import copy
import fcntl
import json
import os
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime

from plant_observer_bringup.capture_events import completion_listener, wait_for_completion

LAP_NAME = re.compile(r'^\d{4}-\d{2}-\d{2}_\d{6}$')
# 회차 폴더에서 올릴 파일. change_compare 명령이 만드는 compare_<기준>/ 하위 폴더는 올리지 않는다
UPLOAD_EXTENSIONS = ('.png', '.jpg', '.json')
# finished가 있는 result.json은 모든 사진을 저장한 뒤 원자적으로 교체하므로 바로 올릴 수 있다.
# patrol_manager는 회차 시작 때부터 result.json을 쓰고 끝날 때 finished를 남긴다. 중간에 Ctrl-C로 끊겨
# finished가 없는 회차는 이만큼 바뀌지 않으면 끝난 것으로 보고 올린다
ABANDONED_SECONDS = 600.0

README = """# 순찰 기록 (자동 업로드)

로봇 노트북의 `capture_uploader`가 순찰 회차가 끝날 때마다 이 폴더에 올린다. 사람이 직접 고치지 않는다.

## 구조

```
captures/
  index.json                  # 회차 목록과 지점별 판정 요약 (사이트는 이것부터 읽는다)
  baseline_<루트>.txt          # 루트별 기준 회차(정상 화분) 이름
  <회차>/                      # 회차 = 순찰 시작 시각 YYYY-MM-DD_HHMMSS
    result.json               # 그 회차의 전체 기록
    <지점 ID>.png              # 지점에서 찍은 사진 (예: P00.png)
    <지점 ID>_compare.jpg      # 기준 | 이번(정렬) | 변화 표시 3칸 비교 그림 (비교했을 때만)
```

## index.json

```jsonc
{
  "updated": "2026-10-30T00:23:10",
  "baselines": {"demo_real3": "2026-10-29_192945"},
  "laps": [                                   // 최신 회차가 먼저
    {
      "lap": "2026-10-30_002113", "route": "demo_real3",
      "started": "2026-10-30T00:21:13", "finished": "2026-10-30T00:22:50",
      "stops": [
        {
          "id": "P00",
          "status": "ok",                     // ok | nav_failed(이동 실패) | no_image(사진 없음)
          "image": "2026-10-30_002113/P00.png",   // captures/ 기준 경로, 없으면 null
          "compare": {                        // 비교를 안 했으면 null
            "status": "changed",              // changed | unchanged | no_reference | align_failed | no_plant | no_image | error
            "change": "fruit_added",          // changed일 때 가장 큰 변화 종류, 아니면 null
            "count": 3,                       // 그 종류의 개수
            "reference_lap": "2026-10-29_192945",
            "reference_image": "2026-10-29_192945/P00.png",
            "image": "2026-10-30_002113/P00_compare.jpg"
          }
        }
      ]
    }
  ]
}
```

변화 종류: `leaf_loss` 잎 탈락, `fruit_added` 열매 생성(성장 기록), `leaf_color` 잎 색 변화,
`fruit_damaged` 열매 손상, `other` 기타 변화. 덩어리별 위치(bbox)·면적 등 자세한 값은 각 회차의 `result.json`에 있다.

지점의 `final`이 있으면 최종 판정으로 사용한다. `final.verdict`는 `confirmed`(재촬영 확인),
`mismatch`(불일치), `recheck_used`(재촬영으로 판정), `recheck_failed`(재촬영 실패)이며,
`final.changes`와 `final.agreeing_views`에 종류별 개수와 일치한 촬영 방향이 있다.
`views.front/left/right`에는 방향별 사진·비교·촬영 위치가 담긴다. 사진 경로는 모두 `captures/` 기준이다.
`compare`는 최초 사진의 결과이므로 `final`을 대신하지 않는다. 회차의 `home`, `mode`,
`baseline_complete`도 결과 파일과 동일하게 제공한다.

`analysis_source: model_fallback`은 기존 정면 비교 실패 후 모델 영역으로 보조한 결과다.
`requires_review: true`이면 변화 없음이어도 관리자 확인이 필요하다. `primary_status`와
`primary_result`에 원래 비교 실패를 보존하고, `model_fallback`에 모델·처리 시간·실패 사유를 남긴다.
`final`에도 보조 여부와 확인 필요 표시를 보존한다. 모델 결과를 자동 확정으로 표시하지 않는다.

사진 주소 예 (GitHub Pages가 이 저장소를 올린 경우): `https://<팀>.github.io/<저장소>/captures/2026-10-30_002113/P00.png`
"""


def git(repo, *args, check=True):
    return subprocess.run(['git', '-C', repo, *args], check=check, capture_output=True,
                          text=True, timeout=60, env={**os.environ, 'GIT_TERMINAL_PROMPT': '0'})


class UploadError(RuntimeError):
    """Local data is retained, but publishing has not completed."""


def log(message):
    print(f'[{datetime.now():%H:%M:%S}] {message}', flush=True)


def finished_laps(capture_dir):
    """result.json이 다 쓰인 회차 폴더 이름들, 오래된 것부터."""
    laps = []
    for name in sorted(os.listdir(capture_dir)):
        path = os.path.join(capture_dir, name, 'result.json')
        if not LAP_NAME.match(name) or not os.path.isfile(path):
            continue
        age = time.time() - os.path.getmtime(path)
        try:
            with open(path) as f:
                result = json.load(f)
        except (OSError, ValueError):
            continue
        if not isinstance(result, dict) or not isinstance(result.get('stops', []), list):
            continue
        # 순찰 중인 회차(사진이 아직 없음)를 올리면 다시 올리지 않아 사진이 빠진다 (2026-10-30_202412)
        if not (isinstance(result, dict) and result.get('finished')) and age < ABANDONED_SECONDS:
            continue
        laps.append(name)
    return laps


def same_file(a, b):
    """copy2가 수정 시각을 그대로 옮기므로 크기·수정 시각만 비교한다. 매 주기 사진 전체를 읽지 않는다.
    다르게 보이면 한 번 다시 복사할 뿐이고, 내용이 같으면 git이 commit을 만들지 않는다."""
    try:
        sa, sb = os.stat(a), os.stat(b)
    except FileNotFoundError:
        return False
    return sa.st_size == sb.st_size and sa.st_mtime_ns == sb.st_mtime_ns


def copy_lap(capture_dir, target_dir, lap):
    source = os.path.join(capture_dir, lap)
    target = os.path.join(target_dir, lap)
    os.makedirs(target, exist_ok=True)
    for name in os.listdir(source):
        if name.endswith(UPLOAD_EXTENSIONS) and os.path.isfile(os.path.join(source, name)):
            shutil.copy2(os.path.join(source, name), os.path.join(target, name))


def lap_matches(capture_dir, target_dir, lap):
    """A matching result.json alone does not prove all photos were copied."""
    source = os.path.join(capture_dir, lap)
    return all(same_file(os.path.join(source, name), os.path.join(target_dir, lap, name))
               for name in os.listdir(source)
               if name.endswith(UPLOAD_EXTENSIONS) and os.path.isfile(os.path.join(source, name)))


def indexed_view(view, lap, stop_id):
    """Keep view metadata and resolve image paths relative to captures/."""
    view = copy.deepcopy(view)
    if view.get('image'):
        view['image'] = f"{lap}/{view['image']}"
    compare = view.get('compare')
    if compare:
        if compare.get('image'):
            compare['image'] = f"{lap}/{compare['image']}"
        reference = compare.get('reference_lap')
        if reference:
            compare['reference_image'] = f"{reference}/{compare.get('reference_image') or stop_id + '.png'}"
    if view.get('views'):
        view['views'] = {name: indexed_view(record, lap, stop_id)
                         for name, record in view['views'].items()}
    for name in ('recheck', 'confirmation', 'overview'):
        if view.get(name):
            view[name] = indexed_view(view[name], lap, stop_id)
    if view.get('attempts'):
        view['attempts'] = [indexed_view(record, lap, stop_id) for record in view['attempts']]
    return view


def write_if_changed(path, text):
    if os.path.isfile(path):
        with open(path) as f:
            if f.read() == text:
                return
    with open(path, 'w') as f:
        f.write(text)


def build_index(target_dir):
    """저장소에 올라간 회차로 index.json 내용을 만든다(로컬 captures가 아니라 저장소 기준)."""
    baselines = {}
    for name in sorted(os.listdir(target_dir)):
        match = re.match(r'^baseline_(.+)\.txt$', name)
        if match:
            with open(os.path.join(target_dir, name)) as f:
                baselines[match.group(1)] = f.read().strip()
    laps = []
    for lap in sorted((d for d in os.listdir(target_dir) if LAP_NAME.match(d)), reverse=True):
        path = os.path.join(target_dir, lap, 'result.json')
        if not os.path.isfile(path):
            continue
        try:
            with open(path) as f:
                result = json.load(f)
            if not isinstance(result, dict) or not isinstance(result.get('stops', []), list):
                raise ValueError('invalid lap schema')
        except (OSError, ValueError) as exc:
            log(f'손상된 회차 제외: {lap}: {exc}')
            continue
        stops = []
        for stop in result.get('stops', []):
            if not isinstance(stop, dict) or not isinstance(stop.get('id'), str):
                continue
            compare = stop.get('compare')
            summary = None
            if compare:
                reference = compare.get('reference_lap')
                summary = {
                    'status': compare.get('status'),
                    'change': compare.get('change'),
                    'count': compare.get('count'),
                    'reference_lap': reference,
                    'reference_image': f"{reference}/{compare.get('reference_image') or stop['id'] + '.png'}" if reference else None,
                    'image': f"{lap}/{compare['image']}" if compare.get('image') else None,
                }
            indexed = indexed_view(stop, lap, stop['id'])
            # Preserve the legacy summary keys along with the full comparison,
            # final verdict and all three view records.
            indexed['compare'] = {**(summary or {}), **indexed['compare']} if compare else None
            indexed.setdefault('views', {})
            stops.append(indexed)
        laps.append({'lap': lap, 'route': result.get('route'), 'started': result.get('started'),
                     'finished': result.get('finished'), 'home': result.get('home'),
                     'mode': result.get('mode'), 'baseline_complete': result.get('baseline_complete'),
                     'compare_to': result.get('compare_to'), 'reference_lap': result.get('reference_lap'),
                     'capture_policy': result.get('capture_policy'),
                     'visit_order': result.get('visit_order'),
                     'stops': stops})
    # updated는 내용이 바뀔 때만 갱신해야 빈 commit이 생기지 않으므로 호출하는 쪽에서 붙인다
    return {'baselines': baselines, 'laps': laps}


def update_index(target_dir):
    path = os.path.join(target_dir, 'index.json')
    index = build_index(target_dir)
    if os.path.isfile(path):
        try:
            with open(path) as f:
                old = json.load(f)
        except (OSError, ValueError):
            old = {}
        if not isinstance(old, dict):
            old = {}
        if {k: v for k, v in old.items() if k != 'updated'} == index:
            return
    index = {'updated': datetime.now().isoformat(timespec='seconds'), **index}
    with open(path, 'w') as f:
        json.dump(index, f, ensure_ascii=False, indent=2)


def has_remote(repo):
    return bool(git(repo, 'remote').stdout.strip())


def ahead_count(repo):
    """원격보다 앞선(아직 push 안 한) commit 수. 추적 브랜치가 없으면 None."""
    result = git(repo, 'rev-list', '--count', '@{u}..HEAD', check=False)
    return int(result.stdout.strip()) if result.returncode == 0 else None


def sync(capture_dir, repo, subdir, routes):
    """Serialize uploaders sharing one checkout without altering the user's index."""
    if not subdir or os.path.isabs(subdir) or '..' in subdir.split('/') or subdir in ('.', '.git'):
        raise ValueError('subdir must be a folder inside the repository')
    git_dir = git(repo, 'rev-parse', '--absolute-git-dir').stdout.strip()
    with open(os.path.join(git_dir, 'capture-uploader.lock'), 'w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise UploadError('같은 저장소의 uploader가 이미 실행 중입니다') from exc
        return _sync(capture_dir, repo, subdir, routes)


def _sync(capture_dir, repo, subdir, routes):
    """새 회차를 복사·commit하고 push한다. 올린 회차 이름 목록을 돌려준다."""
    target_dir = os.path.join(repo, subdir)
    os.makedirs(target_dir, exist_ok=True)

    new_laps = []
    for lap in finished_laps(capture_dir):
        # 이미 올린 회차도 result.json이 바뀌었으면(예전에 순찰 중에 올라간 것) 사진까지 다시 올린다
        if lap_matches(capture_dir, target_dir, lap):
            continue
        if routes:
            with open(os.path.join(capture_dir, lap, 'result.json')) as f:
                if json.load(f).get('route') not in routes:
                    continue
        copy_lap(capture_dir, target_dir, lap)
        new_laps.append(lap)
    for name in os.listdir(capture_dir):
        if re.match(r'^baseline_.+\.txt$', name):
            with open(os.path.join(capture_dir, name)) as f:
                write_if_changed(os.path.join(target_dir, name), f.read())
    write_if_changed(os.path.join(target_dir, 'README.md'), README)
    update_index(target_dir)

    git(repo, 'add', '--', subdir)
    if git(repo, 'diff', '--cached', '--quiet', '--', subdir, check=False).returncode != 0:
        message = (f"순찰 기록 {', '.join(new_laps)}" if new_laps else '순찰 기록 목록·기준 회차 갱신')
        git(repo, 'commit', '--only', '-m', message, '--', subdir)
        log(f'commit: {message}')

    if not has_remote(repo):
        raise UploadError('원격 저장소(remote)가 없어 commit만 했습니다')
    ahead = ahead_count(repo)
    if ahead == 0:
        return new_laps
    if ahead is None:
        # 빈 저장소를 막 clone해서 추적 브랜치가 없을 때: 처음 push하며 연결한다
        push = git(repo, 'push', '-u', 'origin', 'HEAD', check=False)
    else:
        pending = git(repo, 'log', '--format=', '--name-only', '@{u}..HEAD').stdout.splitlines()
        if any(path and not path.startswith(subdir.rstrip('/') + '/') for path in pending):
            raise UploadError('업로드 폴더 밖의 미전송 commit이 있어 자동 push를 중단합니다')
        # Push first: unrelated staged work must remain staged and unpublished.
        push = git(repo, 'push', check=False)
        if push.returncode != 0 and ('[rejected]' in push.stderr):
            if git(repo, 'status', '--porcelain').stdout.strip():
                raise UploadError('원격 갱신이 필요하지만 작업 중인 파일이 있어 자동 rebase를 보류합니다')
            pull = git(repo, 'pull', '--rebase', check=False)
            if pull.returncode != 0:
                git(repo, 'rebase', '--abort', check=False)
                raise UploadError(f'pull 실패: {last_line(pull)}')
            push = git(repo, 'push', check=False)
    if push.returncode != 0:
        raise UploadError(f'push 실패: {last_line(push)}')
    else:
        log('push 완료')
    return new_laps


def last_line(result):
    lines = (result.stderr or result.stdout).strip().splitlines()
    return lines[-1] if lines else f'종료 코드 {result.returncode}'


def check_repo(repo):
    if git(repo, 'rev-parse', '--is-inside-work-tree', check=False).returncode != 0:
        sys.exit(f'{repo}가 git 저장소가 아닙니다. 먼저 팀 저장소를 clone하세요:\n'
                 f'  git clone git@github.com:<팀>/<저장소>.git {repo}')
    # 로봇이 만드는 commit의 작성자. 저장소에만 적용하고 전역 설정은 건드리지 않는다
    if not git(repo, 'config', 'user.name', check=False).stdout.strip():
        git(repo, 'config', 'user.name', 'plant-robot')
    if not git(repo, 'config', 'user.email', check=False).stdout.strip():
        git(repo, 'config', 'user.email', 'plant-robot@users.noreply.github.com')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--capture-dir', default='~/plant_ws/captures')
    parser.add_argument('--repo', default='~/plant_ws/upload_repo', help='팀 저장소를 clone한 폴더 (기본 %(default)s)')
    parser.add_argument('--subdir', default='captures', help='저장소 안에서 기록을 둘 폴더 (기본 %(default)s)')
    parser.add_argument('--route', action='append', default=[],
                        help='이 루트의 회차만 올림. 여러 번 줄 수 있다 (기본: 전부)')
    parser.add_argument('--interval', type=float, default=30.0,
                        help='완료 알림 누락·업로드 실패 시 재확인 주기, 초 (기본 %(default)s)')
    parser.add_argument('--once', action='store_true', help='한 번만 올리고 끝낸다')
    # ros2 launch의 Node로 띄우면 뒤에 --ros-args ...가 붙는다. ROS 노드가 아니라 그 뒤는 버린다
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = parser.parse_args(argv)
    if args.interval <= 0:
        parser.error('--interval은 0보다 커야 합니다')
    capture_dir = os.path.expanduser(args.capture_dir)
    repo = os.path.expanduser(args.repo)
    if not os.path.isdir(capture_dir):
        parser.error(f'{capture_dir} 폴더가 없습니다')
    check_repo(repo)

    listener = None
    try:
        if not args.once:
            listener = completion_listener(capture_dir)
        log(f'{capture_dir} → {repo}/{args.subdir}' + (f" (루트 {', '.join(args.route)})" if args.route else ''))
        if listener is not None:
            log(f'순찰 완료 알림 수신 준비; 실패·누락 재확인은 {args.interval:g}초마다')
        while True:
            failed = False
            try:
                sync(capture_dir, repo, args.subdir, set(args.route))
            except (OSError, subprocess.SubprocessError, ValueError, UploadError) as e:
                failed = True
                detail = e.stderr.strip() if isinstance(e, subprocess.CalledProcessError) else e
                log(f'업로드 중 오류, 다음에 다시 시도: {detail}')
            if args.once:
                return 1 if failed else 0
            if wait_for_completion(listener, args.interval):
                log('순찰 완료 알림: 즉시 업로드 시작')
    except KeyboardInterrupt:
        pass
    finally:
        if listener is not None:
            listener.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
