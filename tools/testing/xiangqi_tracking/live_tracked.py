"""Independent camera stream and passive stop/observe/lock/consume state machines."""
import json,os,signal,threading,time,uuid
from collections import Counter,deque
from pathlib import Path
import cv2,numpy as np,rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo,JointState
from std_msgs.msg import String
from chess_robot_interfaces.msg import ArmState
from chess_robot_vision.msg import FrameInfo
from rosidl_runtime_py.convert import message_to_ordereddict as convert
from cv_bridge import CvBridge
from shared_pipeline import Pipeline
from piece_tracker import PieceTracker
from qualification import revoke_base,expire_observation
from benchmark_lease import BenchmarkLease
from stable_observation import Policy,FeedbackWindow,StableObservation,AuditEpoch,observation_roi

OUT=Path(os.environ.get('XIANGQI_RUN_DIR',str(Path(__file__).resolve().parent)))
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def main():
    OUT.mkdir(parents=True,exist_ok=True);session=uuid.uuid4().hex
    lease=BenchmarkLease();pipeline=Pipeline();tracker=PieceTracker(OUT/('track-state-v2-'+session+'.json'))
    roi=None;detection_count=0;last_base_stamp=0;retire_serial=0;rejects=Counter()
    policy_path=os.environ.get('XIANGQI_OBSERVATION_POLICY')
    policy=Policy(**json.loads(Path(policy_path).read_text())) if policy_path else Policy()
    feedback=FeedbackWindow(policy)
    audit=AuditEpoch(os.environ.get('XIANGQI_FEEDBACK_AUDIT','/home/kuang/workspace/chess_robot/.runtime/audit/feedback.bin'))
    def save(event):
        with (OUT/'snapshot-audit.jsonl').open('a') as f:f.write(json.dumps(event,allow_nan=False)+'\n')
        if event['event']=='locked':
            s=event['snapshot'];(OUT/('snapshot-'+s['snapshot_id']+'.json')).write_text(json.dumps(s,indent=2,allow_nan=False))
            snapshot_pub.publish(String(data=json.dumps(s,allow_nan=False)))
    observation=StableObservation(policy,save)
    rclpy.init();node=rclpy.create_node('xiangqi_readonly_experiment');bridge=CvBridge();lock=threading.RLock()
    cache={k:{} for k in ('arm','joint','color','depth','info','frame')}
    pairs=deque(maxlen=512);requests=deque(maxlen=32);running=True;overflow=False
    def receive(k,m):
        nonlocal overflow
        with lock:
            t=stamp(m);cache[k][t]=m
            while len(cache[k])>(120 if k in ('arm','joint') else 8):del cache[k][min(cache[k])]
            if k in ('arm','joint') and t in cache['arm'] and t in cache['joint']:
                if len(pairs)==pairs.maxlen:overflow=True
                pairs.append((t,convert(cache['arm'][t]),convert(cache['joint'][t])))
    def request(m):
        with lock:
            if len(m.data)>4096 or len(requests)==requests.maxlen:return
            try:parsed=json.loads(m.data)
            except (ValueError,TypeError):return
            if isinstance(parsed,dict):requests.append(parsed)
    subscriptions=[]
    for k,cls,topic in [('arm',ArmState,'/arm/state'),('joint',JointState,'/arm/joint_states'),('color',Image,'/vision/color/image_rect'),('depth',Image,'/vision/aligned_depth/image_rect'),('info',CameraInfo,'/vision/color/camera_info'),('frame',FrameInfo,'/vision/frame_info')]:
        subscriptions.append(node.create_subscription(cls,topic,lambda m,k=k:receive(k,m),100 if k in ('arm','joint') else qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(String,'/pose/xiangqi/observation/request',request,32))
    image_pub=node.create_publisher(Image,'/pose/xiangqi/slow_debug_image',qos_profile_sensor_data)
    data_pub=node.create_publisher(String,'/pose/xiangqi/detections',10)
    response_pub=node.create_publisher(String,'/pose/xiangqi/observation/response',32)
    status_pub=node.create_publisher(String,'/pose/xiangqi/observation/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    snapshot_pub=node.create_publisher(String,'/pose/xiangqi/observation/snapshot',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def stop(*args):
        nonlocal running
        running=False
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    thread=threading.Thread(target=lambda:rclpy.spin(node),daemon=True);thread.start()
    last=0;written=0;count=0;begin=time.monotonic();status_written=0;config_check=0;file_identity=None;config_changed=False
    last_rgbd=None;camera_epoch=uuid.uuid4().hex
    try:
        while running and rclpy.ok():
            now=node.get_clock().now().nanoseconds;raw_good=audit.poll(now)
            with lock:
                pending=list(pairs);pairs.clear();commands=list(requests);requests.clear()
                dropped=overflow;overflow=False
            now=node.get_clock().now().nanoseconds
            raw_good=bool(audit.ok and 0<=(now-audit.host)*1e-9<=policy.state_age_s)
            if dropped:feedback._break('feedback queue overflow',True)
            for t,s,j in sorted(pending,key=lambda row:row[0]):
                if audit.controller is not None and t>=audit.epoch_start_ns:feedback.ingest(t,s,j,now,audit.token)
            held=feedback.at(now,now)
            trusted=bool(raw_good and held['ok'] and audit.token==feedback.epoch)
            if time.monotonic()-config_check>1:
                from recognize_tracked import WEIGHTS,CALIB,URDF
                paths=[WEIGHTS,CALIB,URDF,Path(__file__),Path(__file__).with_name('recognize_tracked.py')]
                if policy_path:paths.append(Path(policy_path))
                try:current=tuple((str(p),p.stat().st_mtime_ns,p.stat().st_size) for p in paths)
                except OSError:current=('configuration file missing',)
                if file_identity is None:file_identity=current
                config_changed=current!=file_identity
                config_check=time.monotonic()
            trusted=trusted and not config_changed
            observation.tick(now,trusted,feedback.generation,audit.token)
            if config_changed:observation.invalidate('on-disk model/configuration changed; visual restart required')
            for command in commands:
                response=observation.request(command,now)
                response_pub.publish(String(data=json.dumps(response,allow_nan=False)))
            if time.monotonic()-status_written>.1:
                status_pub.publish(String(data=json.dumps(observation.status(now),allow_nan=False)));status_written=time.monotonic()
            for job,record,overlay in pipeline.poll():
                context=job['context'];t=record['stamp_ns'];now=node.get_clock().now().nanoseconds
                check=feedback.at(t,now)
                safe=bool(trusted and check['ok'] and context['generation']==feedback.generation and context['epoch']==audit.token)
                fresh_image=0<=(now-t)*1e-9<=.5
                if not safe:revoke_base(record,check['reason'] if not check['ok'] else 'feedback continuity changed during pipeline')
                if not fresh_image:expire_observation(record);overlay=None
                retire=feedback.retire_serial!=retire_serial or (last_base_stamp and (t-last_base_stamp)*1e-9>policy.retire_gap_s)
                if retire:tracker.tracks.clear();tracker.startup_ids.clear();last_base_stamp=0
                retire_serial=feedback.retire_serial
                if record['base_transform_valid']:
                    tracking_start=time.monotonic();tracker.update(record['detections'],t);record['tracking_ms']=(time.monotonic()-tracking_start)*1000
                    if any(d.get('track_id') is not None for d in record['detections']):last_base_stamp=t
                else:
                    for d in record['detections']:d.update(track_id=None,track_confirmed=False)
                record.update(track_namespace=session,feedback_stamp_ns=job['metadata'].get('feedback_stamp_ns'),
                              controller_epoch=context['epoch'],stationary_generation=context['generation'],
                              source_K=job['metadata']['K'],observation_frame_index=count,
                              rgbd_frame_key=job['metadata'].get('rgbd_frame_key'),camera_epoch=job['metadata'].get('camera_epoch'),
                              tracker_state='retired' if retire else ('observed' if record['base_transform_valid'] else 'suspended'))
                if not record['base_transform_valid']:rejects[record['base_transform_reason']]+=1
                record['base_rejection_counts']=dict(rejects)
                observation.observe(record,now,safe and fresh_image,feedback.generation,audit.token)
                if config_changed:observation.invalidate('on-disk model/configuration changed; visual restart required')
                record['observation']=observation.status(now)
                usable=[d['box_xyxy'] for d in record['detections'] if d.get('camera_complete_center_target') and fresh_image]
                if usable:
                    roi=observation_roi(usable)
                for d in record['detections']:d['track_namespace']=session if d.get('track_id') is not None else None
                record['publish_latency_s']=(node.get_clock().now().nanoseconds-t)*1e-9
                record['pipeline']='bounded_shared_memory_gpu_cpu_overlap'
                record['live_stage_ms']=dict(image_decode=context['decode_ms'],shared_memory_copy=context['shared_copy_ms'],image_age_at_processing_start=context['image_age_ms'],analysis=record['processing_s']*1000,gpu_stage=record['detector_ms'],geometry_stage=record['geometry_ms'],tracking=record.get('tracking_ms',0),frame_pipeline_elapsed=(time.monotonic()-job['started'])*1000,debug_image_generated=job['draw'])
                data_pub.publish(String(data=json.dumps(record,allow_nan=False)))
                if overlay is not None:
                    message=bridge.cv2_to_imgmsg(overlay,'bgr8');message.header.stamp.sec=t//10**9;message.header.stamp.nanosec=t%10**9;message.header.frame_id='d405_color_optical_frame';image_pub.publish(message)
                count+=1
                if time.monotonic()-written>3:
                    record['published_frames']=count;record['running_seconds']=time.monotonic()-begin
                    tmp=OUT/'live-latest.json.tmp';tmp.write_text(json.dumps(record,indent=2,allow_nan=False));tmp.replace(OUT/'live-latest.json');written=time.monotonic()
                    print(json.dumps(dict(frames=count,valid=record['base_transform_valid'],observation=record['observation'],rejects=dict(rejects),feedback_diagnostics=dict(raw_age_s=(now-audit.host)*1e-9,ros_age_s=(now-feedback.last)*1e-9,generation=feedback.generation,retire_serial=feedback.retire_serial,raw_reason=audit.reason,feedback_reason=feedback.reason))),flush=True)
            if not lease.can_submit(pipeline.busy()) or not pipeline.can_submit():time.sleep(.001);continue
            chosen=None
            with lock:
                now=node.get_clock().now().nanoseconds;common=set.intersection(*(set(cache[k]) for k in ('color','depth','info','frame')))
                eligible=[t for t in common if t>last and 0<=(now-t)*1e-9<=.2]
                if eligible:
                    t=max(eligible);last=t;im,dep,ci,fi=(cache[k][t] for k in ('color','depth','info','frame'))
                    rgbd=(int(fi.color_frame_number),int(fi.depth_frame_number))
                    repeated=last_rgbd is not None and (rgbd[0]==last_rgbd[0] or rgbd[1]==last_rgbd[1])
                    if last_rgbd is not None and (rgbd[0]<last_rgbd[0] or rgbd[1]<last_rgbd[1]):
                        camera_epoch=uuid.uuid4().hex;observation.invalidate('camera frame counter reset');tracker.tracks.clear();tracker.startup_ids.clear()
                    last_rgbd=rgbd
                    check=feedback.at(t,now);base_good=trusted and check['ok']
                    image_good=fi.stream_ok and im.encoding=='bgr8' and dep.encoding=='32FC1' and im.width==dep.width==ci.width==1280 and im.height==dep.height==ci.height==720 and not any(ci.d) and all(m.header.frame_id=='d405_color_optical_frame' for m in (im,dep,ci,fi)) and np.isfinite(ci.k).all() and ci.k[0]>0 and ci.k[4]>0 and np.allclose(np.array(ci.k).reshape(3,3),np.array(ci.p).reshape(3,4)[:,:3])
                    if image_good and not repeated:
                        metadata=dict(stamp_ns=t,K=list(ci.k),base_eligible=bool(base_good),base_reason=check['reason'] if not check['ok'] else audit.reason,feedback_stamp_ns=check['feedback_stamp_ns'])
                        metadata.update(rgbd_frame_key=[camera_epoch,*rgbd],camera_epoch=camera_epoch)
                        if base_good:
                            row=check['row'];metadata.update(arm=row['arm'],joint=row['joint'],joint_age_s=(now-row['t'])*1e-9)
                        chosen=(im,dep,metadata,feedback.generation,audit.token)
                    else:
                        rejects['image/RGB-D/intrinsics qualification failed']+=1
                        data_pub.publish(String(data=json.dumps(dict(stamp_ns=t,valid=False,detection_valid=False,base_transform_valid=False,detections=[],reason='image/RGB-D/intrinsics qualification failed',observation=observation.status(now)))))
            if chosen is None:time.sleep(.002);continue
            im,dep,metadata,generation,epoch=chosen;decode=time.monotonic();color=bridge.imgmsg_to_cv2(im,'bgr8');depth=bridge.imgmsg_to_cv2(dep,'32FC1');decode_ms=(time.monotonic()-decode)*1000
            context=dict(generation=generation,epoch=epoch,decode_ms=decode_ms,image_age_ms=(node.get_clock().now().nanoseconds-metadata['stamp_ns'])*1e-6,shared_copy_ms=0.)
            current_roi=None if observation.requested or detection_count%18==0 else roi;copy_start=time.monotonic()
            pipeline.submit(color,depth,metadata,current_roi,draw=bool(image_pub.get_subscription_count()),context=context)
            context['shared_copy_ms']=(time.monotonic()-copy_start)*1000;detection_count+=1
    finally:
        try:pipeline.close()
        finally:audit.close();lease.close();rclpy.shutdown();thread.join(timeout=2);node.destroy_node()
if __name__=='__main__':main()
