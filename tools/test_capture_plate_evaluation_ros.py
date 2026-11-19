#!/usr/bin/env python3
"""Isolated ROS transport test with synthetic images; never robot commands.

Run after sourcing Humble and the workspace. Domain 122, localhost only.
Outputs are test fixtures, not Isaac Sim recovery results.
"""
import json
import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

os.environ.update(ROS_DOMAIN_ID='122', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from amr_reobservation.msg import PlateObservation
from PIL import Image as PILImage
from plate_evaluation import match_status

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sampling-mode', choices=['first_raw','first_pair'], default='first_raw')
    parser.add_argument('--omit-results', action='store_true')
    args=parser.parse_args()
    if args.omit_results and args.sampling_mode != 'first_pair':
        parser.error('--omit-results exercises first_pair missing-slot behavior')
    output=Path(tempfile.mkdtemp(prefix='plate_evaluation_ros_',dir='/tmp'))
    os.environ['ROS_LOG_DIR']=str(output/'ros_log')
    rclpy.init(args=[])
    node=rclpy.create_node('plate_evaluation_test_source')
    clocks=node.create_publisher(Clock,'/clock',qos_profile_sensor_data)
    images=node.create_publisher(Image,'/evaluation_test/image_raw',qos_profile_sensor_data)
    observations=node.create_publisher(PlateObservation,'/evaluation_test/observation',10)
    canvas=PILImage.new('RGB',(640,240),'#777777')
    plate=PILImage.open(ROOT/'assets/license_plate/plate_123ga4567.png').convert('RGB').resize((520,110))
    canvas.paste(plate,(50,60)); pixels=canvas.tobytes()
    ns=10*10**9
    child=None
    try:
        for phase in (('before',) if args.omit_results else ('before','after','returned')):
            folder=output/phase
            with (output/(phase+'.log')).open('w') as log:
                child=subprocess.Popen([sys.executable,str(ROOT/'tools/capture_plate_evaluation.py'),
                    '--trial-id','transport_test','--phase',phase,'--output',str(folder),
                    '--seconds','3','--hz','2','--image-topic','/evaluation_test/image_raw',
                    '--sampling-mode',args.sampling_mode,
                    '--observation-topic','/evaluation_test/observation'],stdout=log,stderr=subprocess.STDOUT,env=dict(os.environ))
                deadline=time.monotonic()+18
                while child.poll() is None and time.monotonic()<deadline:
                    ns+=50_000_000
                    c=Clock();c.clock.sec=ns//10**9;c.clock.nanosec=ns%10**9;clocks.publish(c)
                    image=Image();image.header.frame_id='test_camera'
                    image.header.stamp=c.clock;image.width=640;image.height=240
                    image.encoding='rgb8';image.step=640*3;image.data=pixels
                    obs=PlateObservation();obs.header=image.header
                    obs.input_valid=phase!='returned';obs.detected=phase=='after'
                    obs.usable=False;obs.status='low_confidence' if obs.detected else 'not_detected'
                    if phase=='returned': obs.status='inference_error'
                    obs.image_width=640;obs.image_height=240
                    if obs.detected:
                        obs.x=50.;obs.y=60.;obs.width=520.;obs.height=110.;obs.confidence=.3
                    # Alternate callback arrival order to test timestamp pairing.
                    if args.omit_results:
                        images.publish(image)
                    elif ns%100_000_000==0:
                        observations.publish(obs);images.publish(image)
                    else:
                        images.publish(image);observations.publish(obs)
                    end=time.monotonic()+.05
                    while time.monotonic()<end:
                        rclpy.spin_once(node,timeout_sec=.005)
                if child.poll() is None:
                    child.terminate();child.wait(timeout=3)
                    raise AssertionError('Capture exceeded time bound')
                if child.returncode != (1 if args.omit_results else 0):
                    raise AssertionError((output/(phase+'.log')).read_text())
            capture=json.loads((folder/'capture.json').read_text())
            assert capture['capture_fault'] is None, capture['capture_fault']
            assert capture['clock_advanced']
            assert len(capture['samples'])==6
            good=[s for s in capture['samples'] if s.get('image') and s.get('observation')]
            if args.omit_results:
                assert len(good)==0
                assert all(s['capture_status'] in ('missing_pair','missing_image') for s in capture['samples'])
                assert any(s['capture_status']=='missing_pair' for s in capture['samples'])
                print('No results: 6 scheduled slots retained as missing; exit code 1')
                continue
            assert len(good)>=3, capture['statistics']
            assert capture['detector_parameters'], 'Actual publisher parameters not captured'
            for sample in good:
                assert match_status(sample,sample['observation']) == {
                    'before':'not_detected','after':'ready_for_ocr','returned':'input_invalid'}[phase]
                assert PILImage.open(sample['image']).tobytes()==pixels, 'Raw pixels were modified'
            print(f'{phase}: {len(good)}/6 raw source matches; low confidence allowed; no pixels modified')
        print('TEST_FIXTURE_OUTPUT='+str(output))
    finally:
        if child is not None and child.poll() is None:
            child.terminate();child.wait(timeout=3)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
