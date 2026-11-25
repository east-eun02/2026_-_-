"""Private JSON-lines detection worker (apple + leaf models). Run in a separate CPU PyTorch environment.

    detector_worker.py --model apple=/path/apple.pt@640 --model leaf=/path/leaf.pt@512

Request:  {"id": ..., "images": [path, ...]}
Response: {"id": ..., "ok": true, "models": {name: {...}},
           "detections": {name: [[{class, confidence, bbox}, ...] per image]}}
bbox is [x1, y1, x2, y2] in original image pixels; class is the model's own class name.
Raw boxes down to RAW_CONFIDENCE are returned; plant_detector on the ROS side applies thresholds.

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


def model_spec(text):
    name, _, rest = text.partition('=')
    path, _, size = rest.rpartition('@')
    if not name or not path or not size.isdigit():
        raise argparse.ArgumentTypeError('형식: 이름=경로@입력크기')
    return name, Path(path).expanduser().resolve(strict=True), int(size)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=model_spec, action='append', required=True)
    parser.add_argument('--threads', type=int, default=2)
    args = parser.parse_args()
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
        models = {name: (YOLO(str(path)), size) for name, path, size in args.model}
    metadata = {name: {'name': path.name, 'weights_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                       'classes': sorted(models[name][0].names.values()), 'input_size': size,
                       'ultralytics': ultralytics.__version__, 'device': 'cpu', 'raw_confidence': RAW_CONFIDENCE,
                       'threads': args.threads}
                for name, path, size in args.model}

    def detect(model, size, image):
        with redirect_stdout(sys.stderr):
            prediction = model.predict(image, device='cpu', conf=RAW_CONFIDENCE, imgsz=size,
                                       verbose=False, save=False)[0]
        return [{'class': prediction.names[int(box.cls[0])], 'confidence': round(float(box.conf[0]), 4),
                 'bbox': [round(v, 1) for v in box.xyxy[0].tolist()]} for box in prediction.boxes]

    for line in sys.stdin:
        request_id = None
        try:
            request = json.loads(line)
            request_id = request['id']
            start = time.monotonic()
            images = []
            for path in request['images']:
                image = cv2.imread(str(Path(path).resolve(strict=True)))
                if image is None:
                    raise ValueError('사진을 읽을 수 없음')
                images.append(image)
            detections = {name: [detect(model, size, image) for image in images]
                          for name, (model, size) in models.items()}
            response = {'id': request_id, 'ok': True, 'models': metadata, 'detections': detections,
                        'inference_ms': round((time.monotonic()-start)*1000, 2)}
        except Exception as exc:
            response = {'id': request_id, 'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
        print(json.dumps(response, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
