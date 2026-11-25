"""Private JSON-lines apple detection worker. Run in a separate CPU PyTorch environment.

Request:  {"id": ..., "images": [path, ...]}
Response: {"id": ..., "ok": true, "model": {...}, "detections": [[{class, confidence, bbox}, ...], ...]}
bbox is [x1, y1, x2, y2] in original image pixels. Raw boxes down to RAW_CONFIDENCE are returned;
apple_detector.select_apples() on the ROS side applies the per-class thresholds.

No ROS imports or network/model auto-downloads. stdout is reserved for IPC.
"""
import argparse
from contextlib import redirect_stdout
import hashlib
import json
import os
from pathlib import Path
import sys
import time

RAW_CONFIDENCE = .25
# 팀원 학습 입력 크기(apple_yolo26n_aug_v1: imgsz 640)
INPUT_SIZE = 640
CLASSES = {'normal_apple', 'abnormal_apple'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', required=True)
    parser.add_argument('--threads', type=int, default=2)
    args = parser.parse_args()
    path = Path(args.weights).expanduser().resolve(strict=True)
    os.environ.setdefault('YOLO_CONFIG_DIR', str(Path.home() / '.cache/plant_observer/yolo'))
    os.environ['OMP_NUM_THREADS'] = str(args.threads)
    os.environ['MKL_NUM_THREADS'] = str(args.threads)
    with redirect_stdout(sys.stderr):
        import cv2
        import torch
        import ultralytics
        from ultralytics import YOLO, settings
        torch.set_num_threads(args.threads)
        cv2.setNumThreads(args.threads)
        settings.update({k: False for k in ('sync', 'hub', 'wandb', 'mlflow', 'comet', 'clearml', 'tensorboard') if k in settings})
        model = YOLO(str(path))
    names = set(model.names.values())
    metadata = {'name': path.name, 'weights_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                'ultralytics': ultralytics.__version__, 'device': 'cpu', 'raw_confidence': RAW_CONFIDENCE,
                'input_size': INPUT_SIZE, 'threads': args.threads}

    def detect(image_path):
        image = cv2.imread(str(Path(image_path).resolve(strict=True)))
        if image is None:
            raise ValueError('사진을 읽을 수 없음')
        with redirect_stdout(sys.stderr):
            prediction = model.predict(image, device='cpu', conf=RAW_CONFIDENCE, imgsz=INPUT_SIZE,
                                       verbose=False, save=False)[0]
        return [{'class': prediction.names[int(box.cls[0])], 'confidence': round(float(box.conf[0]), 4),
                 'bbox': [round(v, 1) for v in box.xyxy[0].tolist()]} for box in prediction.boxes]

    for line in sys.stdin:
        request_id = None
        try:
            request = json.loads(line)
            request_id = request['id']
            if names != CLASSES:
                raise ValueError(f'사과 모델 클래스가 다름: {sorted(names)}')
            start = time.monotonic()
            detections = [detect(image) for image in request['images']]
            response = {'id': request_id, 'ok': True, 'model': metadata, 'detections': detections,
                        'inference_ms': round((time.monotonic()-start)*1000, 2)}
        except Exception as exc:
            response = {'id': request_id, 'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
        print(json.dumps(response, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
