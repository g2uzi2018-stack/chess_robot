"""Read-only shared-memory pipeline: unchanged FP32 inference and geometry, overlapped by frame."""
import json,os,signal,threading,time
from pathlib import Path
import cv2,numpy as np,rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,JointState
from std_msgs.msg import String
from chess_robot_interfaces.msg import ArmState
from chess_robot_vision.msg import FrameInfo
from rosidl_runtime_py.convert import message_to_ordereddict as convert
from cv_bridge import CvBridge
from shared_pipeline import Pipeline
from piece_tracker import PieceTracker

OUT=Path(os.environ.get('XIANGQI_RUN_DIR',str(Path(__file__).resolve().parent)))
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def main():
    pipeline=Pipeline();tracker=PieceTracker(OUT/'track-state.json');roi=None;detection_count=0
    rclpy.init();node=rclpy.create_node('xiangqi_readonly_experiment');bridge=CvBridge();lock=threading.RLock()
    cache={k:{} for k in ('arm','joint','color','depth','info','frame')};state=dict(seen=0,start=None,previous=None,generation=0);running=True
    def receive(k,m):
        with lock:
            cache[k][stamp(m)]=m
            while len(cache[k])>(120 if k in ('arm','joint') else 8):del cache[k][min(cache[k])]
            if k not in ('arm','joint'):return
            pairs=set(cache['arm'])&set(cache['joint'])
            for t in sorted(t for t in pairs if t>state['seen']):
                state['seen']=t;s,j=cache['arm'][t],cache['joint'][t]
                good=(0<=(node.get_clock().now().nanoseconds-t)*1e-9<=.2 and s.feedback_fresh and s.reference_valid and not s.busy and not s.simulated and s.controller_mode==5 and len(j.name)==len(set(j.name)) and len(j.position)==len(j.velocity)==len(j.name) and len(j.position)>=6 and np.isfinite(j.position).all() and np.isfinite(j.velocity).all() and max(abs(v) for v in j.velocity)<=.002)
                if state['previous']:
                    pt,pq=state['previous'];dt=(t-pt)*1e-9
                    good=good and 0<dt<=.2 and max(abs(a-b) for a,b in zip(j.position,pq))/max(dt,1e-9)<=.002
                if not good:state['start']=None;state['generation']+=1
                elif state['start'] is None:state['start']=t
                state['previous']=t,list(j.position)
    subscriptions=[]
    for k,cls,topic in [('arm',ArmState,'/arm/state'),('joint',JointState,'/arm/joint_states'),('color',Image,'/vision/color/image_rect'),('depth',Image,'/vision/aligned_depth/image_rect'),('info',CameraInfo,'/vision/color/camera_info'),('frame',FrameInfo,'/vision/frame_info')]:
        subscriptions.append(node.create_subscription(cls,topic,lambda m,k=k:receive(k,m),100 if k in ('arm','joint') else qos_profile_sensor_data))
    image_pub=node.create_publisher(Image,'/pose/xiangqi/slow_debug_image',qos_profile_sensor_data);data_pub=node.create_publisher(String,'/pose/xiangqi/detections',10)
    def stop(*args):
        nonlocal running
        running=False
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    thread=threading.Thread(target=lambda:rclpy.spin(node),daemon=True);thread.start();last=0;written=0;count=0;begin=time.monotonic()
    try:
        while running and rclpy.ok():
            for job,record,overlay in pipeline.poll():
                context=job['context'];t=record['stamp_ns']
                with lock:
                    safe=context['generation']==state['generation'] and state['start'] is not None and (node.get_clock().now().nanoseconds-state['seen'])*1e-9<=.2 and 0<=(node.get_clock().now().nanoseconds-t)*1e-9<=.5
                if not safe:
                    for d in record['detections']:
                        d.pop('base_surface_center_m',None);d.pop('base_surface_normal',None);d['valid']=False;d['reason']='arm changed or observation aged during pipeline'
                    overlay=None
                else:
                    tracking_start=time.monotonic();tracker.update(record['detections'],t);record['tracking_ms']=(time.monotonic()-tracking_start)*1000
                    usable=[d['box_xyxy'] for d in record['detections'] if d.get('complete_center_target')]
                    if usable:
                        boxes=np.array(usable);lo=boxes[:,:2].min(0);hi=boxes[:,2:].max(0);size=max(480,int(np.ceil((max(hi-lo)+160)/32))*32);size=min(size,720,1280)
                        cx,cy=np.round((lo+hi)/2/32)*32;x=max(0,min(1280-size,int(cx-size/2)));y=max(0,min(720-size,int(cy-size/2)));roi=[x,y,x+size,y+size]
                record['publish_latency_s']=(node.get_clock().now().nanoseconds-t)*1e-9
                record['pipeline']='bounded_shared_memory_gpu_cpu_overlap';record['live_stage_ms']=dict(image_decode=context['decode_ms'],shared_memory_copy=context['shared_copy_ms'],image_age_at_processing_start=context['image_age_ms'],analysis=record['processing_s']*1000,gpu_stage=record['detector_ms'],geometry_stage=record['geometry_ms'],tracking=record.get('tracking_ms',0),frame_pipeline_elapsed=(time.monotonic()-job['started'])*1000,debug_image_generated=job['draw'])
                data_pub.publish(String(data=json.dumps(record,allow_nan=False)))
                if overlay is not None:
                    message=bridge.cv2_to_imgmsg(overlay,'bgr8');message.header.stamp.sec=t//10**9;message.header.stamp.nanosec=t%10**9;message.header.frame_id='d405_color_optical_frame';image_pub.publish(message)
                count+=1
                if time.monotonic()-written>3:
                    record['published_frames']=count;record['running_seconds']=time.monotonic()-begin
                    tmp=OUT/'live-latest.json.tmp';tmp.write_text(json.dumps(record,indent=2,allow_nan=False));tmp.replace(OUT/'live-latest.json');written=time.monotonic()
                    print(json.dumps(dict(frames=count,names=[d['class_name'] for d in record['detections']],latency_s=record['publish_latency_s'],valid=safe,pipeline=True)),flush=True)
            if not pipeline.can_submit():time.sleep(.001);continue
            chosen=None
            with lock:
                now=node.get_clock().now().nanoseconds;common=set.intersection(*(set(cache[k]) for k in ('color','depth','info','frame')))
                eligible=[t for t in common if t>last and 0<=(now-t)*1e-9<=.2]
                if state['start'] is not None and 0<=(now-state['seen'])*1e-9<=.2:eligible=[t for t in eligible if t<=state['seen']]
                if eligible:
                    t=max(eligible);last=t;im,dep,ci,fi=(cache[k][t] for k in ('color','depth','info','frame'));pairs=set(cache['arm'])&set(cache['joint']);a=min(pairs,key=lambda v:abs(v-t)) if pairs else None
                    stopped=state['start'] is not None and (state['seen']-state['start'])*1e-9>=.6 and 0<=(now-state['seen'])*1e-9<=.2
                    good=stopped and a is not None and abs(a-t)*1e-9<=.025 and state['start']<=t<=state['seen'] and a>=state['start']
                    if good:
                        s,j=cache['arm'][a],cache['joint'][a]
                        good=fi.stream_ok and im.encoding=='bgr8' and dep.encoding=='32FC1' and im.width==dep.width==ci.width==1280 and im.height==dep.height==ci.height==720 and not any(ci.d) and all(m.header.frame_id=='d405_color_optical_frame' for m in (im,dep,ci,fi)) and np.allclose(np.array(ci.k).reshape(3,3),np.array(ci.p).reshape(3,4)[:,:3])
                    if good:chosen=(im,dep,dict(stamp_ns=t,K=list(ci.k),arm=convert(s),joint=convert(j)),state['generation'])
                    elif fi.stream_ok:data_pub.publish(String(data=json.dumps(dict(stamp_ns=t,valid=False,detections=[],reason='synchronized stopped feedback unavailable'))))
            if chosen is None:time.sleep(.002);continue
            im,dep,metadata,generation=chosen;decode=time.monotonic();color=bridge.imgmsg_to_cv2(im,'bgr8');depth=bridge.imgmsg_to_cv2(dep,'32FC1');decode_ms=(time.monotonic()-decode)*1000
            context=dict(generation=generation,decode_ms=decode_ms,image_age_ms=(node.get_clock().now().nanoseconds-metadata['stamp_ns'])*1e-6,shared_copy_ms=0.)
            current_roi=None if detection_count%18==0 else roi;copy_start=time.monotonic()
            job_id=pipeline.submit(color,depth,metadata,current_roi,draw=bool(image_pub.get_subscription_count()),context=context)
            # Context is a small parent-owned dictionary, read only when the completed job returns.
            context['shared_copy_ms']=(time.monotonic()-copy_start)*1000;detection_count+=1
    finally:
        try:pipeline.close()
        finally:rclpy.shutdown();thread.join(timeout=2);node.destroy_node()
if __name__=='__main__':main()
