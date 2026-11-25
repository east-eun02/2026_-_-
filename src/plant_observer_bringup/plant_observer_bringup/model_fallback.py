"""Bounded, lazy model inference outside ROS's Python/OpenCV environment."""
import json
import math
import os
from pathlib import Path
import selectors
import subprocess
import tempfile
import time
import uuid

import cv2
import numpy as np

DEFAULT_PYTHON = str(Path.home() / 'plant_ws/.model_runtime/bin/python')
DEFAULT_WEIGHTS = str(Path.home() / 'plant_ws/models/yolo11n-seg.pt')
ELIGIBLE_STATUSES = frozenset(('board_not_found', 'align_failed', 'no_plant'))


def padded_box(mask):
    ys, xs = np.nonzero(mask)
    if not len(xs):
        raise ValueError('모델 영역이 비어 있음')
    dx = max(8, round((xs.max()-xs.min()+1)*.08))
    dy = max(8, round((ys.max()-ys.min()+1)*.08))
    result = np.zeros_like(mask)
    result[max(0, ys.min()-dy):min(mask.shape[0], ys.max()+dy+1),
           max(0, xs.min()-dx):min(mask.shape[1], xs.max()+dx+1)] = 255
    return result


class WorkerProcess:
    """Lazily started JSON-lines worker in the model venv, one per patrol process."""

    WORKER = 'model_worker.py'

    def __init__(self, python=DEFAULT_PYTHON, weights=DEFAULT_WEIGHTS, timeout=20., worker=None):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('모델 제한 시간은 양수여야 함')
        self.python = os.path.expanduser(python)
        self.weights = os.path.expanduser(weights)
        self.timeout = float(timeout)
        self.worker = worker or str(Path(__file__).with_name(self.WORKER))
        self.process = None
        self.stderr = None
        self.buffer = b''

    def close(self):
        process, self.process = self.process, None
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.)
            for pipe in (process.stdin, process.stdout):
                if pipe:
                    pipe.close()
        if self.stderr is not None:
            self.stderr.close()
            self.stderr = None
        self.buffer = b''

    def available(self):
        return os.path.isfile(self.python) and os.path.isfile(self.weights)

    def _command(self):
        return [self.python, self.worker, '--weights', self.weights, '--threads', '2']

    def _start(self):
        if self.process is not None and self.process.poll() is None:
            return
        self.close()
        if not self.available():
            raise FileNotFoundError('모델 실행 환경 또는 가중치 없음')
        env = os.environ.copy()
        env.pop('PYTHONPATH', None)
        env['PYTHONUNBUFFERED'] = '1'
        self.stderr = tempfile.TemporaryFile()
        self.process = subprocess.Popen(
            self._command(),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.stderr,
            env=env, start_new_session=True)

    def _receive(self, request_id, deadline):
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdout, selectors.EVENT_READ)
            while True:
                if b'\n' in self.buffer:
                    line, self.buffer = self.buffer.split(b'\n', 1)
                    response = json.loads(line)
                    if response.get('id') != request_id:
                        raise ValueError('모델 응답 식별자 불일치')
                    return response
                remaining = deadline-time.monotonic()
                if remaining <= 0 or not selector.select(remaining):
                    raise TimeoutError('모델 응답 제한 시간 초과')
                chunk = os.read(self.process.stdout.fileno(), 65536)
                if not chunk:
                    raise RuntimeError('모델 프로세스가 응답 없이 종료됨')
                self.buffer += chunk
                if len(self.buffer) > 1024*1024:
                    raise ValueError('모델 응답 크기 초과')

    def request(self, payload, deadline):
        """Send one request and return its successful response. Errors close the worker."""
        try:
            self._start()
            request_id = uuid.uuid4().hex
            self.process.stdin.write((json.dumps({**payload, 'id': request_id})+'\n').encode())
            self.process.stdin.flush()
            response = self._receive(request_id, deadline)
            if not response.get('ok'):
                raise ValueError(response.get('error', '모델 추론 실패'))
            return response
        except BaseException:
            self.close()
            raise


class ModelFallback(WorkerProcess):
    """Plant-region model used only when the primary comparison fails. Failure leaves the primary result intact."""

    def masks(self, before, after):
        start = time.monotonic()
        try:
            self._start()
            with tempfile.TemporaryDirectory(prefix='plant-model-') as folder:
                output = os.path.join(folder, 'masks.npz')
                request_id = uuid.uuid4().hex
                request = {'id': request_id, 'before': os.path.abspath(before),
                           'after': os.path.abspath(after), 'output': output}
                self.process.stdin.write((json.dumps(request)+'\n').encode())
                self.process.stdin.flush()
                response = self._receive(request_id, start+self.timeout)
                if not response.get('ok'):
                    raise ValueError(response.get('error', '모델 영역 추출 실패'))
                with np.load(output, allow_pickle=False) as arrays:
                    masks = [arrays[key].copy() for key in ('before', 'after')]
                for path, mask in zip((before, after), masks):
                    image = cv2.imread(path)
                    if (image is None or mask.dtype != np.uint8 or mask.shape != image.shape[:2]
                            or not np.isin(mask, (0, 255)).all() or not mask.any()):
                        raise ValueError('모델 마스크 형식 불일치')
                return masks, {k: response[k] for k in ('model', 'views')}, round((time.monotonic()-start)*1000, 2)
        except BaseException:
            self.close()
            raise

    def compare(self, before, after, params, comparator):
        """Return candidate plus trace. All timeout/errors are ordinary comparison failures."""
        start = time.monotonic()
        try:
            masks, trace, elapsed = self.masks(before, after)
            options = dict(vars(params))
            options.update(board=None, model_scope_before=padded_box(masks[0]),
                           model_scope_after=padded_box(masks[1]),
                           foreground_before=masks[0], foreground_after=masks[1])
            from types import SimpleNamespace
            entry, picture = comparator(before, after, SimpleNamespace(**options))
            trace.update(status='used' if entry['status'] in ('changed', 'unchanged') else 'failed',
                         inference_wall_ms=elapsed, total_wall_ms=round((time.monotonic()-start)*1000, 2),
                         candidate_status=entry['status'])
            return entry, picture, trace
        except Exception as exc:
            return None, None, {'status': 'failed', 'error': f'{type(exc).__name__}: {exc}',
                                'total_wall_ms': round((time.monotonic()-start)*1000, 2)}
