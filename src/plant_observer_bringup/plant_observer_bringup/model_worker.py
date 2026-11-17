"""Private JSON-lines inference worker. Run in a separate CPU PyTorch environment.

No ROS imports or network/model auto-downloads. stdout is reserved for IPC.
"""
import argparse
from collections import OrderedDict
from contextlib import redirect_stdout
import hashlib
import json
import os
from pathlib import Path
import sys
import time


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
        import numpy as np
        import torch
        import ultralytics
        from ultralytics import YOLO, settings
        torch.set_num_threads(args.threads)
        cv2.setNumThreads(args.threads)
        settings.update({k: False for k in ('sync', 'hub', 'wandb', 'mlflow', 'comet', 'clearml', 'tensorboard') if k in settings})
        model = YOLO(str(path))
    metadata = {'name': path.name, 'weights_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                'ultralytics': ultralytics.__version__, 'device': 'cpu', 'confidence': .25,
                'input_size': 640, 'threads': args.threads}
    cache = OrderedDict()

    def infer(image_path):
        image_path = Path(image_path).resolve(strict=True)
        key = hashlib.sha256(image_path.read_bytes()).hexdigest()
        if key in cache:
            value = cache.pop(key)
            cache[key] = value
            return value[0], {**value[1], 'cache_hit': True}
        image = cv2.imread(str(image_path))
        if image is None:
            raise ValueError('사진을 읽을 수 없음')
        start = time.monotonic()
        with redirect_stdout(sys.stderr):
            prediction = model.predict(image, device='cpu', conf=.25, iou=.7,
                                       imgsz=640, retina_masks=True, verbose=False, save=False)[0]
        plants = [i for i, box in enumerate(prediction.boxes)
                  if prediction.names[int(box.cls[0])] == 'potted plant']
        if len(plants) != 1 or prediction.masks is None:
            raise ValueError('식물 영역을 하나로 특정할 수 없음')
        index = plants[0]
        x1, y1, x2, y2 = prediction.boxes[index].xyxy[0].tolist()
        selected = [index]
        # A separate apple elsewhere in the room must not enlarge the target ROI.
        for i, box in enumerate(prediction.boxes):
            if prediction.names[int(box.cls[0])] != 'apple':
                continue
            ax1, ay1, ax2, ay2 = box.xyxy[0].tolist()
            if x1 <= (ax1+ax2)/2 <= x2 and y1 <= (ay1+ay2)/2 <= y2:
                selected.append(i)
        mask = np.zeros(image.shape[:2], np.uint8)
        for i in selected:
            part = prediction.masks.data[i].cpu().numpy()
            if part.shape != mask.shape:
                part = cv2.resize(part, image.shape[1::-1], interpolation=cv2.INTER_NEAREST)
            mask[part > .5] = 255
        area = float((mask > 0).mean())
        if not .01 <= area <= .85:
            raise ValueError('식물 영역 크기가 유효 범위를 벗어남')
        record = {'confidence': round(float(prediction.boxes[index].conf[0]), 4),
                  'bbox': [round(v, 2) for v in (x1, y1, x2, y2)],
                  'area_ratio': round(area, 4), 'inference_ms': round((time.monotonic()-start)*1000, 2),
                  'image_sha256': key, 'cache_hit': False}
        cache[key] = (mask, record)
        if len(cache) > 8:
            cache.popitem(last=False)
        return mask, record

    for line in sys.stdin:
        request_id = None
        try:
            request = json.loads(line)
            request_id = request['id']
            before, b_info = infer(request['before'])
            after, a_info = infer(request['after'])
            output = Path(request['output']).resolve()
            np.savez_compressed(output, before=before, after=after)
            response = {'id': request_id, 'ok': True, 'model': metadata,
                        'views': {'reference': b_info, 'current': a_info}}
        except Exception as exc:
            response = {'id': request_id, 'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
        print(json.dumps(response, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
