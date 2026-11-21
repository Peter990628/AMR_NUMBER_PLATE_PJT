#!/usr/bin/env python3
"""Submit saved lossless originals to the detector, then request a window decision."""
import argparse
import hashlib
import json
from pathlib import Path
from uuid import uuid4

import cv2
import numpy as np
import rclpy
from rosidl_runtime_py.convert import message_to_ordereddict as as_dict
from rosidl_runtime_py.set_message import set_message_fields
from amr_reobservation.msg import PlateWindowSample
from amr_reobservation.srv import AssessPlateWindow, InspectPlateFrame, ReadPlateCrop

from evaluate_plate_recovery import load_captures, validate_inputs
from plate_evaluation import PREPROCESS, match_status, source_key
from probe_plate_ocr import crop_bounds, normalize_text, sha256
from probe_paddle_plate_ocr import write_json


def call(node, client, request, timeout):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        client.remove_pending_request(future)
        raise TimeoutError('service_timeout:'+client.srv_name)
    return future.result()


def inspect(node, client, row, output):
    data = Path(row['image']).read_bytes()
    if hashlib.sha256(data).hexdigest() != row['image_sha256']:
        raise ValueError('source_image_changed')
    req = InspectPlateFrame.Request()
    req.sample_id = row['sample_id']
    set_message_fields(req.image.header, row['image_header'])
    req.image.format = 'png'
    req.image.data.frombytes(data)
    reply = call(node, client, req, 10.)
    if reply.sample_id != req.sample_id or reply.observation.header != req.image.header:
        raise ValueError('detector_source_mismatch')
    row['observation'] = as_dict(reply.observation)
    status = match_status(row, row['observation'])
    if reply.status == 'not_detected' and status == 'not_detected':
        row['status'] = status
        return
    if reply.status != 'recognized' or status != 'ready_for_ocr':
        raise ValueError('frame_inspection:'+reply.status+'/'+status)
    width, height = row['image_size_wh']
    bounds, clipped = crop_bounds(row['observation'], width, height)
    x1, y1, x2, y2 = bounds
    original = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    crop = original[y1:y2, x1:x2]
    # Verify that the pixels acknowledged by the Python OCR node match this original.
    if hashlib.sha256(crop.tobytes()).hexdigest() != reply.crop_pixels_sha256:
        raise ValueError('ocr_crop_pixels_mismatch')
    crop_path = output/(row['id']+'_crop.png')
    ocr_path = output/(row['id']+'_ocr.png')
    resized = cv2.resize(crop, (PREPROCESS['width'], PREPROCESS['height']), interpolation=cv2.INTER_CUBIC)
    if not cv2.imwrite(str(crop_path), crop) or not cv2.imwrite(str(ocr_path), resized):
        raise OSError('crop_write_failed')
    row.update(status='recognized', raw_text=reply.text, recognition_confidence=reply.ocr_confidence,
               recognition_ms=reply.recognition_ms, ocr_attempted=True, crop_bounds_xyxy=bounds,
               crop_clipped=clipped, crop_image=str(crop_path), crop_sha256=sha256(crop_path),
               crop_pixels_sha256=reply.crop_pixels_sha256, ocr_image=str(ocr_path),
               ocr_image_sha256=sha256(ocr_path))


def assess(node, client, capture, rows, *, prefix=False):
    req = AssessPlateWindow.Request()
    req.capture_valid = capture['clock_advanced'] and not capture['capture_fault']
    # The full capture manifest is validated before testing any prefix.
    req.expected_count = len(rows) if prefix else capture['sampling']['count']
    for row in rows:
        sample = PlateWindowSample(sample_id=row['sample_id'], slot=row['slot'], status=row['status'],
                                   normalized_text=normalize_text(row['raw_text'] or ''),
                                   ocr_confidence=row['recognition_confidence'] or 0.,
                                   barrier_since_previous=bool(row.get('barrier_since_previous')))
        if row.get('observation'):
            set_message_fields(sample.observation, row['observation'])
        req.samples.append(sample)
    answer = call(node, client, req, 3.)
    return dict(success=answer.success, reason=answer.reason, text=answer.text or None,
                supporting_slots=list(answer.supporting_slots), trailing_count=len(answer.supporting_slots),
                rows_considered=answer.supporting_slots[-1]+1 if answer.success else len(rows),
                policy=dict(consecutive=answer.consecutive_required,
                min_ocr_confidence=answer.min_ocr_confidence, max_source_gap_sec=answer.max_source_gap_sec,
                min_box_iou=answer.min_box_iou))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    capture, = load_captures([args.capture])
    validate_inputs([capture])
    previous = None
    for source in capture['samples']:
        if source['image']:
            frame, stamp = source_key(source['image_header'])
            stream = (frame, source['image_size_wh'])
            if previous and (stamp <= previous[0] or stream != previous[1]):
                raise ValueError('inconsistent_capture_sequence')
            previous = (stamp, stream)
    args.output.mkdir(parents=True, exist_ok=False)
    # Older offline manifests have only a per-window slot ID.
    run_id = capture.get('request_namespace') or uuid4().hex
    if capture.get('request_namespace') and any(
            s.get('sample_id') != run_id+'_'+s['id'] for s in capture['samples']):
        raise ValueError('invalid_capture_sample_id')
    rows = []
    report = dict(rows=rows, request_namespace=run_id, capture_sha256=sha256(args.capture))
    rclpy.init(args=[])
    node = rclpy.create_node('numbered_plate_collector')
    try:
        detector = node.create_client(InspectPlateFrame, '/plate/inspect_frame')
        decision = node.create_client(AssessPlateWindow, '/plate/assess_window')
        ocr = node.create_client(ReadPlateCrop, '/plate/read_crop')
        for service in (detector, decision, ocr):
            if not service.wait_for_service(timeout_sec=5.):
                raise RuntimeError('service_unavailable:'+service.srv_name)
        evidence = None
        for source in capture['samples']:
            row = dict(source, observation=None, sample_id=run_id+'_'+source['id'],
                       status=source['capture_status'], raw_text=None,
                       ocr_attempted=False, recognition_confidence=None, recognition_ms=None)
            rows.append(row)
            if evidence and evidence['success']:
                if source['image']:
                    row['status'] = 'not_evaluated_after_success'
                continue
            if source['image']:
                try:
                    inspect(node, detector, row, args.output)
                except Exception as error:
                    row['status'] = 'inspection_failed'
                    row['error'] = str(error)
                    raise
            evidence = assess(node, decision, capture, rows, prefix=len(rows) < len(capture['samples']))
        report['decision'] = evidence
        # Compatible with the existing offline before/after evaluator. Preserve
        # the raw manifest (also when replaying an older experiment).
        paired = [{**original, 'sample_id': row['sample_id'], 'observation': row['observation']}
                  for original, row in zip(capture['samples'], rows)]
        paired_capture = args.output/'capture.json'
        write_json(paired_capture, {**capture, 'samples': paired})
        report['paired_capture'] = str(paired_capture)
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        write_json(args.output/'report.json', report)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report['decision'], ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
