#!/usr/bin/env python3
"""Read a numbered BGR plate crop with the local Paddle recognition model."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import time

from plate_evaluation import PREPROCESS
from probe_plate_ocr import sha256


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--cache', required=True, type=Path)
    args = parser.parse_args()
    for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
        os.environ[key] = '2'
    os.environ.update(PADDLE_PDX_DISABLE_MODEL_SOURCE_CHECK='True', HF_HUB_OFFLINE='1',
                      PADDLE_PDX_CACHE_HOME=str(args.cache), XDG_CACHE_HOME=str(args.cache))
    # Keep the same address-space cap as the previous batch recognizer.
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    cap = 5 * 1024**3
    if soft == resource.RLIM_INFINITY or soft > cap:
        resource.setrlimit(resource.RLIMIT_AS, (cap, hard))
    import cv2
    import numpy as np
    from paddleocr import TextRecognition
    import rclpy
    from amr_reobservation.srv import ReadPlateCrop

    baseline = json.loads(args.baseline.read_text())
    config = baseline['config']
    for name, expected in baseline['model_hashes'].items():
        if sha256(Path(config['model_dir'])/name) != expected:
            raise ValueError('ocr_model_changed')
    cv2.setNumThreads(2)
    model = TextRecognition(**config)
    rclpy.init(args=[])
    node = rclpy.create_node('plate_crop_ocr')

    def read(req, reply):
        image = req.crop
        reply.sample_id = req.sample_id
        reply.header = image.header
        reply.status = 'invalid_crop'
        if (not req.sample_id or not image.header.frame_id or image.encoding != 'bgr8'
                or not 0 < image.width <= 8192 or not 0 < image.height <= 8192
                or image.step != image.width*3 or len(image.data) != image.step*image.height):
            return reply
        try:
            start = time.monotonic()
            reply.crop_pixels_sha256 = hashlib.sha256(image.data).hexdigest()
            crop = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, 3)
            resized = cv2.resize(crop, (PREPROCESS['width'], PREPROCESS['height']),
                                 interpolation=cv2.INTER_CUBIC)
            prediction, = model.predict(input=resized, batch_size=1)
            reply.text = str(prediction['rec_text'])
            reply.confidence = float(prediction['rec_score'])
            if not math.isfinite(reply.confidence) or not 0 <= reply.confidence <= 1:
                raise ValueError('invalid_ocr_score')
            reply.recognition_ms = (time.monotonic()-start)*1000
            reply.status = 'recognized'
        except Exception as error:
            node.get_logger().error(f'{req.sample_id}: {error}')
            reply.status = 'ocr_error'
        return reply

    node.create_service(ReadPlateCrop, '/plate/read_crop', read)
    node.get_logger().info('Local Korean plate OCR ready')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
