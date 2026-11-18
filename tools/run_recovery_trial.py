#!/usr/bin/env python3
"""Bounded Isaac Sim experiment: confirm a snapshot, try poses, measure fresh images.

Default only selects and records. --execute enables a separate /cmd_vel guard
AFTER verifying the Isaac subscriber and exclusive ownership. No USD ground truth.
This is a local experiment runner, not a general navigation/recovery service.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from recovery_trial_math import PoseController, RotationController, angle, pose_dict, segment_clearance, preview_allows_frozen_target
from recovery_trial_math import validate_rotation_return
from recovery_trial_math import bounded_setup_target, guard_graph_ready, validate_trial_motion
from recovery_selection import CandidateSelector, SelectionConfig, choose_diverse_snapshot
from recovery_translation_selection import choose_translation_snapshot
from recovery_trial_budget import assess_translation_budget
from recovery_trial_metrics import ObservationWindow, match_measurement_image
from cfv_trial_selection import load_context, choose_cfv_snapshot
from cfv_view_trial import choose_view_snapshot, validate_view_leases, LIMITS as VIEW_LIMITS
from cfv_view_trial import CLOSE_LIMITS


def stamp_ns(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def clean(value):
    if isinstance(value, float) and not math.isfinite(value): return None
    if isinstance(value, dict): return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)): return [clean(v) for v in value]
    if hasattr(value,'tolist'): return clean(value.tolist())
    return value


def write_json(path, value):
    path.write_text(json.dumps(clean(value), indent=2, allow_nan=False)+'\n')


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.set_message import set_message_fields
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan, Image
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from tf2_ros import Buffer, TransformListener, TransformException
    from amr_reobservation.msg import RecoveryPreview, RecoveryRequest, PlateObservation
    from candidate_frame_math import quaternion_matrix

    if args.output.exists(): raise ValueError('Output directory already exists')
    args.output.mkdir(parents=True)
    started=time.monotonic()
    prior_path=args.retry_report or args.return_only_report
    prior=json.loads(prior_path.read_text()) if prior_path else None
    context_record=prior or (json.loads(args.context_report.read_text()) if args.context_report else None)
    cfv_context=load_context(args.cfv_inference_report) if args.cfv_inference_report else None
    reference_path=args.repeat_view_reference or args.cfv_close_reference
    close_reference=json.loads(reference_path.read_text()) if reference_path else None
    repeat_view=bool(args.repeat_view_reference)
    view_mode=args.cfv_view_c or args.cfv_fresh_close or bool(close_reference)
    extended_observation=repeat_view or args.cfv_fresh_close
    view_limits=CLOSE_LIMITS if close_reference or args.cfv_fresh_close else VIEW_LIMITS
    target_radius=view_limits['target_radius_m'] if view_mode else .51
    # A manual 0.5m lateral setup needs turn/drive/align at the same low speeds.
    # Only its one-leg deadline changes; trial goals and spatial guards stay bounded.
    leg_seconds=view_limits['per_leg_wall_sec'] if view_mode else 180. if args.setup_local_offset else 100.
    result={'mode':'simulation_execute' if args.execute else 'selection_only',
            'baseline':None,'selection':None,'trials':[], 'recovered':False,
            'comparison':{'rotation_only':args.rotation_only,
                'translation_comparison':args.translation_comparison,'required_aims':args.required_aims,
                'cfv_axis_comparison':bool(cfv_context), 'cfv_view_c':view_mode,
                'camera_range_target_m':view_limits['camera_range_m'] if view_mode else None,
                'distance_reference_report':str(reference_path) if close_reference else None,
                'repeat_view':repeat_view,'fresh_close_view':args.cfv_fresh_close,
                'setup_local_offset':args.setup_local_offset,
                'trial_observation_sim_sec':10. if extended_observation else 1.},
            'motion_limits':{'max_linear_m_s':0. if args.rotation_only else .07,'max_angular_rad_s':.25,
                'footprint_circle_m':1.2,'radius_from_start_m':.03 if args.rotation_only else view_limits['motion_radius_m'] if view_mode else .55,
                'target_radius_m':target_radius, 'per_leg_wall_sec':leg_seconds,'max_session_wall_sec':args.max_seconds},
            'limitations':['Observed obstacle clearance is not certified free space/navigation.',
                'Aims come from observed LiDAR surfaces; no plate/front/rear ground truth.',
                'Usable is the existing detection/quality proxy, not OCR success.']}
    if cfv_context:
        result['cfv_context']={k:v for k,v in cfv_context.items() if k!='reference_preview'}
    streams=(args.output/'events.jsonl').open('w')
    def event(kind, **data):
        row=clean(dict(kind=kind,wall_sec=time.monotonic()-started,**data))
        streams.write(json.dumps(row,allow_nan=False)+'\n'); streams.flush()
        if kind not in ('plate','pose','preview','command'):
            print(json.dumps({k:v for k,v in row.items() if k!='selection'},ensure_ascii=False),flush=True)
    rclpy.init(args=[])
    guard=None; guard_log=None; node=None
    try:
        node=Node('recovery_trial_'+str(os.getpid()),
                  parameter_overrides=[Parameter('use_sim_time',value=True)])
        tf=Buffer(node=node); listener=TransformListener(tf,node)
        latest={}; previews=deque(maxlen=300); plates=deque(maxlen=1000)
        image_history=deque(maxlen=10)
        clock_state={'ns':0,'advance_wall':time.monotonic(),'rollback':False}
        clock_history=deque(maxlen=1000)
        baseline_clock_rate=[None]
        def receive(kind,msg):
            now=time.monotonic(); latest[kind]=(now,msg)
            if kind=='preview':
                d=message_to_ordereddict(msg); previews.append((now,d))
                event('preview',message=d)
            elif kind=='plate':
                plates.append((now,msg)); event('plate',message=message_to_ordereddict(msg))
            elif kind=='odom': event('pose',message=message_to_ordereddict(msg))
        def clock_cb(msg):
            n=msg.clock.sec*1_000_000_000+msg.clock.nanosec
            if n<clock_state['ns']: clock_state['rollback']=True
            if n>clock_state['ns']:
                clock_state['advance_wall']=time.monotonic()
                clock_history.append((clock_state['advance_wall'],n))
            clock_state['ns']=n
        subscriptions=[node.create_subscription(Clock,'/clock',clock_cb,qos_profile_sensor_data),
            node.create_subscription(RecoveryPreview,'/reobservation/preview',lambda m:receive('preview',m),10),
            node.create_subscription(RecoveryRequest,'/reobservation/request',lambda m:receive('request',m),10),
            node.create_subscription(PlateObservation,'/plate/observation',lambda m:receive('plate',m),qos_profile_sensor_data),
            node.create_subscription(Odometry,'/chassis/odom',lambda m:receive('odom',m),qos_profile_sensor_data),
            node.create_subscription(Image,'/debug/plate_observation',image_history.append,qos_profile_sensor_data)]
        for topic in ['/observation_lidar/scan','/front_2d_lidar/scan','/back_2d_lidar/scan']:
            subscriptions.append(node.create_subscription(LaserScan,topic,lambda m,k=topic:receive(k,m),qos_profile_sensor_data))
        command_topic='/reobservation/trial_'+str(os.getpid())+'/velocity_request'
        publisher=node.create_publisher(Twist,command_topic,1)
        guard_name='recovery_command_guard_'+str(os.getpid())
        active_request=[None]
        active_request_body=[None]
        initial=[None]
        frozen=[bool(prior)]
        preparing=[True]
        fixture=[None]
        fixture_pub=node.create_publisher(RecoveryRequest,'/reobservation/request',10) if context_record else None
        fixture_last=[0.]
        last_command=[0.,0.]
        def command(v=0.,w=0.):
            validate_trial_motion(frozen[0],preparing[0],v,w)
            if args.rotation_only and v!=0.: raise RuntimeError('translation_forbidden_in_rotation_comparison')
            msg=Twist(); msg.linear.x=float(v); msg.angular.z=float(w)
            publisher.publish(msg); last_command[:]=[v,w]
            event('command',linear=v,angular=w)
        def spin(seconds=.04):
            end=time.monotonic()+seconds
            while time.monotonic()<end and rclpy.ok():
                if fixture[0] is not None and time.monotonic()-fixture_last[0]>.1:
                    fixture_pub.publish(fixture[0]); fixture_last[0]=time.monotonic()
                rclpy.spin_once(node,timeout_sec=min(.02,max(0,end-time.monotonic())))
        def now_ns(): return node.get_clock().now().nanoseconds
        def recent_clock_rate():
            if not clock_history: return 0.
            recent=[p for p in clock_history if p[0]>=clock_history[-1][0]-5.]
            if len(recent)<2 or recent[-1][0]-recent[0][0]<1.: return 0.
            return (recent[-1][1]-recent[0][1])/1e9/(recent[-1][0]-recent[0][0])
        def current():
            odom=latest['odom'][1]
            if odom.header.frame_id!='odom' or odom.child_frame_id!='base_link':
                raise RuntimeError('unexpected_odometry_frames')
            return pose_dict(message_to_ordereddict(odom.pose.pose))
        def health(require_preview=True,allow_contour_miss=False):
            wall=time.monotonic()
            if wall-started>args.max_seconds: raise RuntimeError('session_wall_limit')
            if clock_state['rollback']: raise RuntimeError('clock_rollback')
            if now_ns()<=0 or wall-clock_state['advance_wall']>1.5: raise RuntimeError('clock_stalled')
            for key,limit in [('odom',1.),('plate',1.5),('request',1.5),
                    ('/observation_lidar/scan',1.5),('/front_2d_lidar/scan',1.5),('/back_2d_lidar/scan',1.5)]:
                if key not in latest or wall-latest[key][0]>limit: raise RuntimeError('stale_'+key)
                if key!='request':
                    age=(now_ns()-stamp_ns(latest[key][1].header))/1e9
                    if not -.1<=age<=.65: raise RuntimeError('source_age_'+key)
            if not latest['plate'][1].input_valid: raise RuntimeError('plate_input_'+latest['plate'][1].status)
            request=latest['request'][1]
            identity=(request.request_id,stamp_ns(request.header))
            if not request.enabled: raise RuntimeError('request_disabled')
            if active_request[0] is not None and identity!=active_request[0]: raise RuntimeError('request_changed')
            if active_request_body[0] is not None and message_to_ordereddict(request)!=active_request_body[0]:
                raise RuntimeError('request_body_changed')
            lifetime=request.valid_for_sec
            if now_ns()/1e9>=identity[1]/1e9+lifetime: raise RuntimeError('request_expired')
            if require_preview:
                if 'preview' not in latest or wall-latest['preview'][0]>1.5: raise RuntimeError('stale_preview')
                p=latest['preview'][1]
                # Baseline/selection happens at rest. A temporary association
                # miss there is not a sensor fault; selection still requires
                # its own confirmed observations before move() can run.
                stationary_preparation=preparing[0] and last_command==[0.,0.]
                if not preview_allows_frozen_target(p.valid,p.status,frozen[0] or allow_contour_miss or stationary_preparation):
                    raise RuntimeError('preview_'+p.status)
                if active_request[0] and (p.request_id,stamp_ns(p.request_header))!=active_request[0]:
                    raise RuntimeError('preview_request_changed')
            if initial[0] is not None and math.dist(current()[:2],initial[0][:2])>result['motion_limits']['radius_from_start_m']:
                raise RuntimeError('motion_radius_exceeded')
            if guard:
                if guard.poll() is not None: raise RuntimeError('command_guard_exited')
                pubs=node.get_publishers_info_by_topic('/cmd_vel')
                if len(pubs)!=1 or pubs[0].node_name!=guard_name:
                    raise RuntimeError('cmd_vel_publisher_mismatch:'+repr([p.node_name for p in pubs]))
                subs=node.get_subscriptions_info_by_topic('/cmd_vel')
                if len(subs)!=1 or 'iw_hub_ROS_differential_drive_ros2_subscribe_twist' not in subs[0].node_name:
                    raise RuntimeError('simulation_subscriber_changed')
        def observed_points(only_observation=False,save_label=None):
            points=[]
            evidence=[]
            keys=['/observation_lidar/scan'] if only_observation else ['/observation_lidar/scan','/front_2d_lidar/scan','/back_2d_lidar/scan']
            scan_set={key:latest[key][1] for key in keys}
            for key in keys:
                scan=scan_set[key]
                deadline=time.monotonic()+.25
                while True:
                    try:
                        transform=tf.lookup_transform('odom',scan.header.frame_id,Time.from_msg(scan.header.stamp))
                        base_tf=tf.lookup_transform('base_link',scan.header.frame_id,Time.from_msg(scan.header.stamp))
                        break
                    except TransformException:
                        command(); spin(.02); health()
                        if time.monotonic()>deadline: raise
                q=transform.transform.rotation; t=transform.transform.translation
                r=quaternion_matrix((q.x,q.y,q.z,q.w))
                bq=base_tf.transform.rotation; bt=base_tf.transform.translation
                br=quaternion_matrix((bq.x,bq.y,bq.z,bq.w))
                if save_label:
                    evidence.append({'topic':key,'scan':message_to_ordereddict(scan),
                        'odom_from_scan':message_to_ordereddict(transform),
                        'base_from_scan':message_to_ordereddict(base_tf),
                        'range_counts':{'finite':sum(math.isfinite(v) for v in scan.ranges),
                            'positive_inf':sum(v==math.inf for v in scan.ranges),
                            'nan':sum(math.isnan(v) for v in scan.ranges)}})
                for i,value in enumerate(scan.ranges):
                    if not math.isfinite(value) or value<max(scan.range_min,.05) or value>min(scan.range_max,8.): continue
                    a=scan.angle_min+i*scan.angle_increment; x=value*math.cos(a); y=value*math.sin(a)
                    bx=br[0][0]*x+br[0][1]*y+bt.x; by=br[1][0]*x+br[1][1]*y+bt.y
                    # Stock iw_hub chassis self returns, in scan-time BASE frame.
                    # Known model: 1.42 x .65m centered x=-.32008; 2cm tolerance.
                    if key!='/observation_lidar/scan' and -1.05<=bx<=.41 and abs(by)<=.35: continue
                    points.append((r[0][0]*x+r[0][1]*y+t.x,r[1][0]*x+r[1][1]*y+t.y))
            if save_label:
                write_json(args.output/(save_label+'_scans.json'),{'scans':evidence,'points_xy_odom':points,
                    'note':'Nonfinite raw ranges serialized as null; these are observed returns, not certified free space.'})
            return points
        def clearance(target,save_label=None):
            value=segment_clearance(observed_points(save_label=save_label),current()[:2],target[:2])
            if value<1.2: raise RuntimeError('swept_clearance_'+str(round(value,3)))
            return value
        def save_evidence(label,matched,window,observations):
            from cv_bridge import CvBridge
            import cv2
            evidence=[]
            pairs=sorted(matched.values(),key=lambda pair:stamp_ns(pair[0].header),reverse=True)
            for m,d in pairs:
                verified=match_measurement_image(message_to_ordereddict(m.header),m.width,m.height,window,observations)
                if verified is None: continue
                filename=label+'.png' if not evidence else label+'_'+str(stamp_ns(m.header))+'.png'
                image=CvBridge().imgmsg_to_cv2(m,desired_encoding='bgr8')
                if not cv2.imwrite(str(args.output/filename),image): raise RuntimeError('image_write_failed')
                row={'image':filename,'source_stamp_ns':stamp_ns(m.header),
                     'observation':verified,'meaning':'Exact source image/result match; requires visual review.'}
                evidence.append(row); event('image_evidence',label=label,**row)
                command(); spin(.02)
            if not evidence: raise RuntimeError('no_matched_measurement_image_'+label)
            return evidence
        def stationary(seconds=.5):
            command(); begin=now_ns(); deadline=time.monotonic()+8
            anchor=current(); stable_start=now_ns()
            while (now_ns()-stable_start)/1e9<seconds:
                command(); spin(); health()
                p=current(); odom=latest['odom'][1].twist.twist
                if math.dist(p[:2],anchor[:2])>.01 or abs(angle(p[2]-anchor[2]))>.01 or abs(odom.linear.x)>.015 or abs(odom.angular.z)>.025:
                    stable_start=now_ns(); anchor=p
                if time.monotonic()>deadline: raise RuntimeError('settle_timeout')
            return now_ns()
        def measure(label,extended=False):
            cutoff=stationary(.35)
            anchor=current(); max_drift=0.; max_yaw_drift=0.
            window=ObservationWindow(cutoff,min_span_sec=10. if extended else 1.)
            deadline=time.monotonic()+(60. if extended else 14.); seen_wall=time.monotonic()
            observations={}; matched={}
            while time.monotonic()<deadline:
                command(); spin(.05); health()
                if extended:
                    p=current(); twist=latest['odom'][1].twist.twist
                    max_drift=max(max_drift,math.dist(p[:2],anchor[:2]))
                    max_yaw_drift=max(max_yaw_drift,abs(angle(p[2]-anchor[2])))
                    if (max_drift>.01 or max_yaw_drift>.01
                            or abs(twist.linear.x)>.015 or abs(twist.angular.z)>.025):
                        raise RuntimeError('observation_not_stationary_'+label)
                for receipt,msg in list(plates):
                    if receipt<=seen_wall: continue
                    d=message_to_ordereddict(msg)
                    window.add(d,now_ns(),receipt)
                    observations[(msg.header.frame_id,stamp_ns(msg.header))]=d
                if plates: seen_wall=plates[-1][0]
                for m in image_history:
                    d=match_measurement_image(message_to_ordereddict(m.header),m.width,m.height,window,observations)
                    if d is not None: matched[d['status']]=(m,d)
                s=window.summary()
                if s['complete']: break
            s=window.summary(); s['pose']=list(current()); s['cutoff_ros_ns']=cutoff
            if extended:
                s['stationary_window']={'anchor':anchor,'max_translation_m':max_drift,
                                       'max_yaw_rad':max_yaw_drift,'wall_limit_sec':60.}
            s['image_evidence']=save_evidence(label,matched,window,observations)
            event('measurement',label=label,summary=s)
            if not s['complete']: raise RuntimeError('incomplete_observation_window_'+label)
            return s
        def move(target,label):
            if not frozen[0]: raise RuntimeError('move_without_fixed_target')
            preparing[0]=False
            if args.rotation_only and math.dist(target[:2],initial[0][:2])>.01:
                raise RuntimeError('rotation_target_xy_changed')
            if math.dist(target[:2],initial[0][:2])>target_radius: raise RuntimeError('target_outside_local_trial')
            health(); safe=clearance(target,save_label=label)
            event('motion_start',label=label,target=target,clearance_m=safe)
            controller=(RotationController if args.rotation_only else PoseController)(target)
            deadline=time.monotonic()+leg_seconds
            progress_wall=time.monotonic(); previous=current(); last_check=0.; min_clearance=safe
            while True:
                health(); pose=current()
                if time.monotonic()-last_check>.15:
                    min_clearance=min(min_clearance,clearance(target)); last_check=time.monotonic()
                v,w=controller.command(pose)
                command(v,w)
                if controller.phase=='done': break
                if math.dist(pose[:2],previous[:2])>.005 or abs(angle(pose[2]-previous[2]))>.008:
                    previous=pose; progress_wall=time.monotonic()
                if time.monotonic()-progress_wall>8: raise RuntimeError('motion_no_progress')
                if time.monotonic()>deadline: raise RuntimeError('motion_timeout')
                spin(.04)
            command(); event('motion_arrived',label=label,actual_pose=current(),min_clearance_m=min_clearance)

        def select_candidates():
            # Keep every 6 x 3 preview pair until translation geometry is ranked.
            selector=CandidateSelector(SelectionConfig(max_trials=18)) if args.translation_comparison else CandidateSelector()
            seen=set(); select_started=time.monotonic(); deadline=select_started+12
            best_count=0; best_improving_count=0; last_selection_reason=None
            while time.monotonic()<deadline:
                spin(.08); health(allow_contour_miss=True)
                for receipt,p in list(previews):
                    key=(p['header']['stamp']['sec'],p['header']['stamp']['nanosec'])
                    if key in seen: continue
                    seen.add(key)
                    selected=selector.observe(p,now_ros_sec=now_ns()/1e9,now_wall_sec=time.monotonic())
                    chosen=(choose_view_snapshot(selected,p,cfv_context,now_ns()/1e9,close_reference,repeat_view,args.cfv_fresh_close) if view_mode else
                        choose_cfv_snapshot(selected,p,cfv_context,now_ns()/1e9) if cfv_context else
                        choose_translation_snapshot(selected,p,required_aims=args.required_aims,
                        max_trials=args.max_trials) if args.translation_comparison else
                        choose_diverse_snapshot(selected,required_aims=args.required_aims,
                            rotation_only=args.rotation_only,max_trials=args.max_trials))
                    best_count=max(best_count,len({c['aim_index'] for c in selected.get('queue',[])}))
                    if cfv_context:
                        best_improving_count=max(best_improving_count,len(chosen.get('queue',[])))
                        last_selection_reason=chosen['status']
                    elif args.translation_comparison:
                        diagnostics=chosen.get('diagnostics',{}).get('translation',{})
                        best_improving_count=max(best_improving_count,diagnostics.get('available_aims',0))
                        last_selection_reason=diagnostics.get('reason')
                    if chosen.get('queue'):
                        source=chosen['queue'][0]['header']['stamp']
                        if not -.1<=now_ns()/1e9-source['sec']-source['nanosec']/1e9<=.5:
                            raise RuntimeError('selected_snapshot_stale')
                        chosen['selection_wall_sec']=time.monotonic()-select_started
                        return chosen  # Freeze immediately; later single-aim frames cannot overwrite it.
            event('selection_incomplete',required_aims=args.required_aims,best_confirmed_aims=best_count,
                best_improving_aims=best_improving_count,last_reason=last_selection_reason)
            raise RuntimeError('insufficient_translation_candidates' if args.translation_comparison else 'insufficient_aim_diversity')

        # A retry is a NEW explicit fixed-odom fixture, never a lease extension or
        # a base-relative re-anchor at the robot's turned position.
        if context_record:
            spin(1.)
            # DDS discovery may take longer than one second on a loaded sim.
            # Wait for pose and advancing time before validating saved context;
            # the command guard is not running and no motion can be requested.
            discovery_deadline=time.monotonic()+15.
            while 'odom' not in latest or now_ns()<=0 or len(clock_history)<2:
                if time.monotonic()>discovery_deadline:
                    raise RuntimeError('context_discovery_timeout')
                spin(.1)
            if node.get_publishers_info_by_topic('/reobservation/request') and any(
                    p.node_name!=node.get_name() for p in node.get_publishers_info_by_topic('/reobservation/request')):
                raise RuntimeError('retry_requires_mock_false_and_no_upstream_publisher')
            if context_record['request']['header']['frame_id']!='odom' or (prior and prior['status']!='aborted'):
                raise RuntimeError('invalid_retry_report')
            p=current(); saved=context_record['final_pose']
            if math.dist(p[:2],saved[:2])>.03 or abs(angle(p[2]-saved[2]))>math.radians(2):
                raise RuntimeError('retry_scene_pose_changed')
            if args.return_only_report:
                validate_rotation_return(prior['initial_pose'],saved,p)
            old_stamp=context_record['selection']['queue'][0]['header']['stamp']
            old_final=context_record.get('return_observation',{}).get('last_source_stamp_ns',0)
            if now_ns()<=max(old_final,old_stamp['sec']*1_000_000_000+old_stamp['nanosec']):
                raise RuntimeError('retry_clock_epoch_changed')
            fixture[0]=RecoveryRequest(); set_message_fields(fixture[0],context_record['request'])
            fixture[0].header.stamp=node.get_clock().now().to_msg()
            fixture[0].request_id='manual_fixed_trial_'+str(os.getpid())
            fixture[0].valid_for_sec=args.max_seconds
            fixture[0].source='MOCK new bounded trial; saved manual odom context; no base re-anchor'
            active_request[0]=(fixture[0].request_id,stamp_ns(fixture[0].header))
            active_request_body[0]=message_to_ordereddict(fixture[0])
            if prior: initial[0]=tuple(prior['initial_pose'])
            result['context_provenance']={'report':str(prior_path or args.context_report),
                'previous_request':context_record['request'],
                'meaning':'New bounded manual context, no base re-anchor. '+
                    ('Finish original start yaw only; new scans and unchanged stopped pose required.' if args.return_only_report else
                     'Old targets revalidated with fresh scans.' if prior else 'Fresh candidate selection; old candidates are not reused.')}
            event('new_fixed_context',request=active_request_body[0])
        # Wait for discovery and fresh source data; no motion publisher exists yet.
        deadline=time.monotonic()+20
        while True:
            spin(.1)
            try: health(); break
            except RuntimeError as exc:
                if time.monotonic()>deadline: raise RuntimeError('readiness_'+str(exc))
        subs=node.get_subscriptions_info_by_topic('/cmd_vel')
        existing=node.get_publishers_info_by_topic('/cmd_vel')
        audit={'subscribers':[x.node_name for x in subs],'publishers':[x.node_name for x in existing]}
        event('control_audit',**audit)
        if args.execute and (existing or len(subs)!=1 or 'iw_hub_ROS_differential_drive_ros2_subscribe_twist' not in subs[0].node_name):
            raise RuntimeError('simulation_control_endpoint_not_exclusive')
        active_request[0]=(latest['request'][1].request_id,stamp_ns(latest['request'][1].header))
        active_request_body[0]=message_to_ordereddict(latest['request'][1])
        if view_mode:
            # A repeated mock heartbeat does NOT renew preview's own wall lease.
            # Check the two companion nodes, not only RecoveryRequest.valid_for_sec.
            from rcl_interfaces.srv import GetParameters
            def read_limits(remote,names):
                client=node.create_client(GetParameters,remote+'/get_parameters')
                try:
                    if not client.wait_for_service(timeout_sec=2.):
                        raise RuntimeError('view_lease_service_missing_'+remote)
                    request=GetParameters.Request(); request.names=names
                    future=client.call_async(request); deadline=time.monotonic()+3.
                    while not future.done():
                        spin(.03)
                        if time.monotonic()>deadline: raise RuntimeError('view_lease_service_timeout')
                    values=future.result().values
                    if len(values)!=len(names) or any(v.type!=3 for v in values):
                        raise RuntimeError('view_lease_parameter_type')
                    return [v.double_value for v in values]
                finally: node.destroy_client(client)
            preview_wall=read_limits('/recovery_preview',['request_timeout_sec'])[0]
            mock_wall,mock_ros=read_limits('/mock_recovery_request',['max_wall_age_sec','valid_for_sec'])
            validate_view_leases(preview_wall,mock_wall,mock_ros)
            result['view_lease_parameters']={'preview_wall_sec':preview_wall,
                'mock_wall_sec':mock_wall,'mock_ros_sec':mock_ros}
            event('view_lease_preflight',**result['view_lease_parameters'])
        if args.execute:
            guard_log=(args.output/'guard.log').open('w')
            guard=subprocess.Popen([sys.executable,str(Path(__file__).with_name('recovery_command_guard.py')),
                '--input-topic',command_topic,'--node-name',guard_name,'--max-session',str(args.max_seconds),
                '--max-linear','0' if args.rotation_only else '.08',
                '--max-angular','.25' if args.rotation_only else '.3','--timeout','.35'],
                stdout=guard_log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+5
            while True:
                command(); spin(.05)
                inputs=[p.node_name for p in node.get_subscriptions_info_by_topic(command_topic)]
                outputs=[p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')]
                if guard.poll() is not None: raise RuntimeError('command_guard_exited_during_discovery')
                if guard_graph_ready(inputs,outputs,guard_name):
                    event('guard_graph_ready',input_subscribers=inputs,output_publishers=outputs)
                    break
                if time.monotonic()>deadline:
                    raise RuntimeError('guard_discovery_timeout:'+repr((inputs,outputs)))
            health()
        stationary(.35)
        if not prior: initial[0]=current()
        result['initial_pose']=list(initial[0]); result['request']=message_to_ordereddict(latest['request'][1])
        return_target=tuple(close_reference['initial_pose']) if repeat_view else initial[0]
        result['return_target_pose']=list(return_target)
        safe=clearance(initial[0]); event('ready',initial_pose=initial[0],observed_clearance_m=safe)
        if args.setup_local_offset:
            # Change only the experimental starting layout; never present this
            # manually specified move as an AI-generated observation candidate.
            target=bounded_setup_target(initial[0],args.setup_local_offset)
            # This manual target is fixed before baseline sampling. Temporary
            # contour-association misses cannot invalidate its selection;
            # health still rejects missing scans/TF, bad inputs and stale data.
            frozen[0]=True
            result['return_target_pose']=None
            result['planned_stop_pose']=list(target)
            result['selection']={'status':'manual_setup_only','queue':[],
                'offset_forward_left_m_yaw_deg':args.setup_local_offset,'target_pose':target}
            write_json(args.output/'selection.json',result['selection'])
            baseline_wall=time.monotonic();baseline_ros=now_ns()
            result['baseline']=measure('baseline')
            rate=(now_ns()-baseline_ros)/1e9/(time.monotonic()-baseline_wall)
            safe=clearance(target,save_label='setup_preflight')
            result['path_preflight']={'outbound_observed_clearance_m':safe,'required_envelope_m':1.2}
            # Setup deliberately ends at the changed starting pose. Use that
            # same hold pose for the budget's second leg (zero motion), retaining
            # its conservative observation and cleanup reserves.
            budget=assess_translation_budget(current(),target,target,
                baseline_rate=rate,recent_rate=recent_clock_rate(),
                remaining_wall_sec=max(0.,args.max_seconds-(time.monotonic()-started)),
                per_leg_wall_limit_sec=leg_seconds)
            result['budget_assessments']=[budget]
            budget['purpose']='Manual setup: one outbound leg, then hold; no return leg requested.'
            if not budget['feasible']:raise RuntimeError('setup_insufficient_budget')
            if not args.execute:
                result['status']='setup_selected_only';return result
            move(target,'setup_placement')
            result['setup_observation']=measure('setup_placement')
            result['status']='setup_complete'
            return result
        if args.return_only_report:
            result['selection']={'status':'return_only','queue':[]}
            write_json(args.output/'selection.json',result['selection'])
            move(initial[0],'return_initial')
            result['return_observation']=measure('returned')
            result['status']='return_only_complete'
            return result
        if prior:
            result['original_baseline']=prior['baseline']
            result['baseline']=measure('baseline')
            chosen=prior['selection']
            queue=chosen['queue']
            completed={t['candidate']['candidate_index'] for t in prior['trials'] if 'observation' in t}
            queue=[c for c in queue if c['candidate_index'] not in completed][:args.max_trials]
            if not queue: raise RuntimeError('retry_no_remaining_candidate')
            supported=[]; seen_scans=set(); deadline=time.monotonic()+10
            while time.monotonic()<deadline and len(supported)<3:
                spin(.08); health()
                scan_stamp=stamp_ns(latest['/observation_lidar/scan'][1].header)
                if scan_stamp in seen_scans: continue
                seen_scans.add(scan_stamp); points=observed_points(only_observation=True)
                counts=[sum(math.dist(p,(c['look_at']['x'],c['look_at']['y']))<=.15 for p in points) for c in queue]
                if all(n>=3 for n in counts): supported.append({'stamp_ns':scan_stamp,'support_counts':counts})
                else: supported.clear()
            if len(supported)<3: raise RuntimeError('retry_aim_no_fresh_support')
            result['fresh_aim_support']=supported
            result['selection']=chosen
            event('retry_targets_revalidated',support=supported,targets=[c['label'] for c in queue])
        else:
            baseline_wall=time.monotonic(); baseline_ros=now_ns()
            result['baseline']=measure('baseline')
            baseline_clock_rate[0]=(now_ns()-baseline_ros)/1e9/(time.monotonic()-baseline_wall)
            result['baseline_clock_rate']=baseline_clock_rate[0]
            chosen=select_candidates()
            result['selection']=chosen
            queue=chosen['queue'][:args.max_trials]
        if view_mode:
            # Save all three scan-time transforms/ranges and inspect the entire
            # outbound AND reverse center path before any nonzero command.
            target=pose_dict(queue[0]['base_pose'])
            points=observed_points(save_label='preflight')
            outbound=segment_clearance(points,current()[:2],target[:2])
            returning=segment_clearance(points,target[:2],return_target[:2])
            result['path_preflight']={'outbound_observed_clearance_m':outbound,
                'return_observed_clearance_m':returning,'required_envelope_m':1.2,
                'scans_file':'preflight_scans.json','observed_point_count':len(points)}
            event('path_preflight',**result['path_preflight'])
            if min(outbound,returning)<1.2: raise RuntimeError('view_path_clearance')
        write_json(args.output/'selection.json',chosen)
        event('selected',selection=chosen)
        if not args.execute:
            result['status']='selected_only'; return result
        comparison_mode=args.rotation_only or args.translation_comparison
        if result['baseline']['recovered'] and not comparison_mode:
            result['status']='baseline_already_usable'; result['recovered']=True; return result
        frozen[0]=True
        for index,candidate in enumerate(queue):
            label='trial_'+str(index+1)
            target=pose_dict(candidate['base_pose'])
            if args.translation_comparison:
                extra_observation_reserve=46. if extended_observation else 0.
                budget=assess_translation_budget(current(),target,return_target,
                    baseline_rate=baseline_clock_rate[0],recent_rate=recent_clock_rate(),
                    remaining_wall_sec=max(0.,args.max_seconds-(time.monotonic()-started)-extra_observation_reserve),
                    per_leg_wall_limit_sec=leg_seconds)
                budget['extended_observation_reserve_sec']=extra_observation_reserve
                event('translation_budget',label=label,assessment=budget)
                result.setdefault('budget_assessments',[]).append(budget)
                if not budget['feasible']:
                    result['status']='translation_deferred_insufficient_budget'
                    result['deferred_candidate']=candidate['label']
                    result['plate_recovery_verified']=False
                    if index:
                        move(return_target,'return_initial')
                        result['return_observation']=measure('returned')
                    return result
            item={'candidate':candidate,'target':list(target),'label':label}; result['trials'].append(item)
            move(target,label)
            item['observation']=measure(label,extended=extended_observation)
            write_json(args.output/'report.json',result)
            if item['observation']['recovered'] and not comparison_mode:
                result['recovered']=True; result['status']='usable_recovered'; break
        else:
            result['status']=('rotation_comparison_complete' if args.rotation_only else
                'translation_comparison_complete' if args.translation_comparison else 'tested_candidates_not_usable')
            if comparison_mode:
                result['quality_proxy_pass_labels']=[t['candidate']['label'] for t in result['trials'] if t['observation']['recovered']]
                result['plate_recovery_verified']=False
                result['verification_status']='requires_visual_or_ocr_review'
            move(return_target,'return_initial')
            result['return_observation']=measure('returned')
        return result
    except (Exception,KeyboardInterrupt) as exc:
        result['status']='aborted'; result['error']=str(exc) or type(exc).__name__
        event('aborted',error=result['error'])
        return result
    finally:
        # Guard independently goes to zero if this process stalls or dies.
        if node is not None and rclpy.ok():
            end=time.monotonic()+.6
            while time.monotonic()<end:
                try: command(); spin(.03)
                except Exception: break
        if guard is not None:
            try:
                if guard.poll() is None: guard.send_signal(signal.SIGTERM)
                guard.wait(timeout=3)
            except Exception as exc: result['guard_cleanup_error']=str(exc)
            result['guard_exit_code']=guard.poll()
        if node is not None and rclpy.ok():
            try: spin(.3)  # Read post-stop odometry after the guard's final zeros.
            except Exception: pass
        if guard_log: guard_log.close()
        if node is not None:
            try:
                result['final_pose']=list(current()) if 'odom' in latest else None
                result['final_measured_twist']=message_to_ordereddict(latest['odom'][1].twist.twist) if 'odom' in latest else None
                node.destroy_node()
            except Exception: pass
        if rclpy.ok(): rclpy.shutdown()
        result['elapsed_wall_sec']=time.monotonic()-started
        write_json(args.output/'report.json',result); streams.close()
        compact={k:v for k,v in result.items() if k not in ('selection','request','retry_provenance','context_provenance')}
        compact['trials']=[{k:v for k,v in t.items() if k!='candidate'} for t in result['trials']]
        print(json.dumps(clean(compact),ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execute',action='store_true')
    context_group=parser.add_mutually_exclusive_group()
    context_group.add_argument('--retry-report',type=Path,help='New fixed-context trial from an aborted report; requires mock:=false')
    context_group.add_argument('--context-report',type=Path,help='Copy only saved odom mission context; freshly select candidates; requires mock:=false')
    context_group.add_argument('--return-only-report',type=Path,help='Finish yaw return only from an aborted trial already within 3cm of its start; requires mock:=false')
    mode_group=parser.add_mutually_exclusive_group()
    mode_group.add_argument('--rotation-only',action='store_true',help='Compare all selected aims using zero linear velocity, then return yaw')
    mode_group.add_argument('--translation-comparison',action='store_true',help='Rank fresh translations by observed surface incidence gain; measure then return regardless of usable proxy')
    mode_group.add_argument('--cfv-inference-report',type=Path,help='Three fresh stationary CFV images; rank new previews by end-face axis, try one translation and return')
    mode_group.add_argument('--setup-local-offset',type=float,nargs=3,metavar=('FORWARD_M','LEFT_M','YAW_DEG'),help='Manual start-layout change only, <=0.5m/20deg, then stop there; no AI candidate selection')
    view_group=parser.add_mutually_exclusive_group()
    view_group.add_argument('--cfv-view-c',action='store_true',help='Generate reviewed 45deg / 2.5m view from fresh confirmed surface; CFV input required, one bounded round trip')
    view_group.add_argument('--cfv-fresh-close',action='store_true',help='Compute a NEW 45deg / 2.0m view from fresh CFV and surface, observe 10 sim seconds, then return; no saved reference')
    view_group.add_argument('--cfv-close-reference',type=Path,help='Compare 2.0m on a measured C report ray after fresh CFV/surface revalidation; one bounded round trip')
    view_group.add_argument('--repeat-view-reference',type=Path,help='Revisit a completed usable 2.0m target, observe for 10 sim seconds, return to its fixed baseline; fresh CFV required')
    parser.add_argument('--required-aims',type=int,default=2)
    parser.add_argument('--max-trials',type=int,help='Default:1 for translation comparison, otherwise4')
    parser.add_argument('--max-seconds',type=float)
    a=parser.parse_args()
    if a.setup_local_offset:
        if a.retry_report or a.context_report or a.return_only_report:
            parser.error('setup cannot reuse a saved context')
        try:bounded_setup_target((0,0,0),a.setup_local_offset)
        except ValueError as exc:parser.error(str(exc))
        a.required_aims=1;a.max_trials=1
    if a.return_only_report:
        if a.rotation_only or a.translation_comparison or a.cfv_inference_report or a.cfv_view_c or a.cfv_fresh_close or a.cfv_close_reference or a.repeat_view_reference:
            parser.error('return-only cannot be combined with another trial mode')
        a.rotation_only=True; a.required_aims=1; a.max_trials=1
    if (a.cfv_view_c or a.cfv_fresh_close or a.cfv_close_reference or a.repeat_view_reference) and not a.cfv_inference_report: parser.error('view comparisons require --cfv-inference-report')
    view_session_cap=(CLOSE_LIMITS if a.cfv_fresh_close or a.cfv_close_reference or a.repeat_view_reference else VIEW_LIMITS)['session_wall_sec']
    if a.max_seconds is None: a.max_seconds=120. if a.return_only_report else view_session_cap if a.cfv_view_c or a.cfv_fresh_close or a.cfv_close_reference or a.repeat_view_reference else 260.
    if a.cfv_inference_report:
        if a.retry_report or a.context_report:
            parser.error('CFV comparison requires the current request, not a saved retry context')
        a.translation_comparison=True
        a.required_aims=1
        if a.max_trials not in (None,1): parser.error('CFV comparison is limited to one translation')
    if a.max_trials is None: a.max_trials=1 if a.translation_comparison else 4
    cap=120 if a.return_only_report else view_session_cap if a.cfv_view_c or a.cfv_fresh_close or a.cfv_close_reference or a.repeat_view_reference else 280
    if not 1<=a.max_trials<=8 or not 30<=a.max_seconds<=cap: parser.error('invalid bounded trial limits')
    if a.translation_comparison and a.max_trials>3: parser.error('translation comparison supports at most3 trials')
    if not a.return_only_report and not a.setup_local_offset and ((not a.cfv_inference_report and not 2<=a.required_aims<=3) or (not a.translation_comparison and a.required_aims>a.max_trials)):
        parser.error('required-aims must be2..3 and <=max-trials except in translation comparison')
    if a.retry_report and (a.rotation_only or a.translation_comparison):
        parser.error('comparison needs fresh selection; use context-report, not retry-report')
    r=run(a)
    sys.exit(1 if r['status']=='aborted' else 0)
