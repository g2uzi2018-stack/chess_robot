"""Bounded live vision comparisons. No arm/gripper command interface is used."""
import argparse
import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String
from chess_robot_interfaces.msg import ArmState
from chess_robot_vision.msg import FrameInfo
from rosidl_runtime_py.convert import message_to_ordereddict as convert

ROOT = Path('/home/kuang/workspace/chess_robot')
SOURCE = ROOT/'runs/history_vision_decouple_20261101_01/vision-source-02'
BACKEND = ROOT/'runs/perf_ceiling_20261031/01_backend'
RESOURCE = ROOT/'runs/perf_ceiling_20261031/02_resources'
TRAIN = ROOT.parent/'xiangqi-yolo-training/20261031-yolo26s-seg-v1'
RUNTIME = TRAIN/'runtime/python-yolo'
PT = TRAIN/'runs/train/weights/best.pt'
TRT = BACKEND/'weights/20261031T143600930755709/best.engine'
LOCK = ROOT/'runs/perf_ceiling_20261031/benchmark.lock'
PROFILE = ROOT/'install_vision/chess_robot_vision/share/chess_robot_vision/config/large_images_fastdds.xml'
URL = 'http://192.168.30.26:8088'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def command(args, timeout=25):
    p = subprocess.run([str(a) for a in args], capture_output=True, text=True, timeout=timeout)
    if p.returncode:
        raise RuntimeError(f'{args}: {p.returncode}: {p.stdout[-1000:]} {p.stderr[-1000:]}')
    return p.stdout


def quantiles(values):
    vals = sorted(float(v) for v in values if v is not None and math.isfinite(v))
    def at(p):
        if not vals:return None
        pos=(len(vals)-1)*p;lo=int(pos);hi=min(lo+1,len(vals)-1)
        return vals[lo]+(vals[hi]-vals[lo])*(pos-lo)
    return dict(n=len(vals),p50=at(.5),p95=at(.95),p99=at(.99),maximum=vals[-1] if vals else None)


def stop_process(p):
    if p is None or p.poll() is not None:return
    p.send_signal(signal.SIGTERM)
    try:p.wait(timeout=25)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid,signal.SIGTERM)
        try:p.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)


class Observer:
    def __init__(self, out):
        self.out=out;self.node=rclpy.create_node('live_perf_comparison_observer')
        self.lock=threading.RLock();self.rows=[];self.frames=[];self.arms=[];self.joints=[];self.previews=[];self.http=[]
        self.raw={};self.snapshot=None
        self.opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with self.opener.open(URL+'/api/channels',timeout=3) as response:
            assert response.status==200
        self.subs=[
            self.node.create_subscription(String,'/pose/xiangqi/detections',self.detection,50),
            self.node.create_subscription(FrameInfo,'/vision/frame_info',lambda m:self.append(self.frames,m),qos_profile_sensor_data),
            self.node.create_subscription(ArmState,'/arm/state',lambda m:self.append(self.arms,m),100),
            self.node.create_subscription(JointState,'/arm/joint_states',lambda m:self.append(self.joints,m),100),
            self.node.create_subscription(String,'/pose/xiangqi/preview_metrics',self.preview,20),
        ]
        self.running=True
        self.thread=threading.Thread(target=self.spin,daemon=True);self.thread.start()
        self.http_thread=threading.Thread(target=self.viewer,daemon=True);self.http_thread.start()

    def append(self, dest, m):
        with self.lock:dest.append((time.monotonic(),m))

    def detection(self,m):
        try:
            r=json.loads(m.data)
            with self.lock:self.rows.append((time.monotonic(),r))
        except (ValueError,TypeError):pass

    def preview(self,m):
        try:
            with self.lock:self.previews.append((time.monotonic(),json.loads(m.data)))
        except ValueError:pass

    def spin(self):
        while self.running and rclpy.ok():rclpy.spin_once(self.node,timeout_sec=.05)

    def viewer(self):
        after='';client='codex_live_ab_20261101'
        while self.running:
            start=time.monotonic()
            try:
                with self.opener.open(f'{URL}/api/frame/detection?client={client}&after={after}',timeout=2) as response:
                    payload=response.read();seq=response.headers.get('X-Frame-Sequence')
                    with self.lock:self.http.append((time.monotonic(),dict(status=response.status,seq=seq,bytes=len(payload),request_ms=(time.monotonic()-start)*1000)))
                    if seq:
                        after=seq
                        if not (self.out/'scene-preview.jpg').exists():(self.out/'scene-preview.jpg').write_bytes(payload)
                    if response.status!=200:time.sleep(.03)
            except Exception as exc:
                with self.lock:self.http.append((time.monotonic(),dict(error=str(exc))))
                time.sleep(.1)

    def stationary(self):
        now=time.monotonic()
        with self.lock:
            arms=[m for t,m in self.arms if now-t<3]
            joints=[m for t,m in self.joints if now-t<3]
        return len(arms)>=20 and len(joints)>=20 and all(m.feedback_fresh and not m.busy and not m.simulated and m.controller_mode in (4,5) for m in arms) and all(m.velocity and max(abs(v) for v in m.velocity)<=.002 for m in joints)

    def wait(self, seconds, candidate=None):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if candidate is not None and candidate.poll() is not None:raise RuntimeError(f'candidate exited {candidate.returncode}')
            time.sleep(.1)

    def measure(self, directory, seconds, candidate=None, label=None):
        directory.mkdir(parents=True,exist_ok=False)
        start=time.monotonic();self.wait(seconds,candidate);end=time.monotonic()
        with self.lock:
            rows=[dict(receive_monotonic=t,record=r) for t,r in self.rows if start<=t<=end]
            frames=[(t,m) for t,m in self.frames if start<=t<=end]
            arms=[m for t,m in self.arms if start<=t<=end]
            previews=[dict(receive_monotonic=t,record=m) for t,m in self.previews if start<=t<=end]
            http=[dict(receive_monotonic=t,**r) for t,r in self.http if start<=t<=end]
        if label:rows=[r for r in rows if r['record'].get('compute',{}).get('live_ab_label')==label]
        complete=[r for r in rows if r['record'].get('detection_valid') and 'pipeline_job_id' in r['record']]
        unique={r['record']['stamp_ns']:r for r in complete};records=[r['record'] for r in unique.values()]
        stages=sorted(set(k for r in records for k in r.get('live_stage_ms',{})))
        frame_stamps={m.header.stamp.sec*10**9+m.header.stamp.nanosec for t,m in frames}
        http_unique={r['seq'] for r in http if r.get('status')==200 and r.get('seq')}
        source_span=(max(frame_stamps)-min(frame_stamps))*1e-9 if len(frame_stamps)>1 else None
        summary=dict(window_s=end-start,records=len(rows),complete_unique_frames=len(unique),completion_hz=len(unique)/(end-start),frame_info_unique=len(frame_stamps),frame_info_hz=len(frame_stamps)/(end-start),frame_info_source_stamp_hz=(len(frame_stamps)-1)/source_span if source_span else None,completion_to_frame_info_ratio=len(unique)/len(frame_stamps) if frame_stamps else None,frame_info_is_not_four_image_topic_matching=True,duplicates=len(complete)-len(unique),camera_geometry_frames=sum(bool(r.get('camera_geometry_valid')) for r in records),base_transform_frames=sum(bool(r.get('base_transform_valid')) for r in records),motion_ready_frames=sum(bool(r.get('motion_ready')) for r in records),grasp_ready_frames=sum(bool(r.get('grasp_ready')) for r in records),latency_host_sdk_stamp_to_publish_ms=quantiles([r.get('publish_latency_s',0)*1000 for r in records]),live_stage_ms={k:quantiles([r.get('live_stage_ms',{}).get(k) for r in records]) for k in stages},detections_per_frame=quantiles([len(r.get('detections',[])) for r in records]),arm_samples=len(arms),arm_reference_values=sorted(set(m.reference_valid for m in arms)),arm_modes=sorted(set(m.controller_mode for m in arms)),arm_feedback_fresh_fraction=sum(m.feedback_fresh for m in arms)/len(arms) if arms else None,http_unique_jpeg=len(http_unique),http_jpeg_hz=len(http_unique)/(end-start),preview_latest=previews[-1]['record'] if previews else None,compute_samples=[r.get('compute') for r in records[:2]],roi_shapes=sorted(set(tuple(r.get('inference_roi_xyxy',[])) for r in records)))
        (directory/'records.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in rows))
        (directory/'preview.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in previews))
        (directory/'http.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in http))
        (directory/'frame_info.jsonl').write_text(''.join(json.dumps(dict(receive_monotonic=t,record=convert(m)))+'\n' for t,m in frames))
        (directory/'arm_state.jsonl').write_text(''.join(json.dumps(convert(m))+'\n' for m in arms))
        (directory/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
        assert summary['motion_ready_frames']==summary['grasp_ready_frames']==0
        assert summary['http_jpeg_hz']>10,'Controlled JPEG preview load was not established'
        return summary

    def close(self):
        self.running=False;self.thread.join(3);self.http_thread.join(3)
        try:
            self.opener.open(urllib.request.Request(URL+'/api/release?client=codex_live_ab_20261101',data=b'',method='POST'),timeout=2).read()
        except Exception:pass
        self.node.destroy_node()


def build_source(out):
    source=out/'source';shutil.copytree(SOURCE,source,ignore=shutil.ignore_patterns('__pycache__','*.json'))
    shutil.move(source/'recognize_tracked.py',source/'recognize_native.py')
    shutil.copy2(Path(__file__).parent/'recognize_tracked.py',source/'recognize_tracked.py')
    adapter=(BACKEND/'adapter.py').read_text()
    old='masks = (result.masks.data.detach().cpu().numpy() > 0.5).astype("uint8") if result.masks is not None else None'
    new='masks = (((result.masks.data.detach() > 0.5).to(torch.uint8).cpu().numpy() if __import__("os").environ.get("LIVE_AB_MASK_GPU") == "1" else (result.masks.data.detach().cpu().numpy() > 0.5).astype("uint8")) if result.masks is not None else None)'
    assert adapter.count(old)==1
    (source/'adapter.py').write_text(adapter.replace(old,new))
    pipeline=(source/'shared_pipeline.py').read_text()
    marker="    record['pipeline_job_id']=job['job_id'];"
    assert pipeline.count(marker)==1
    pipeline=pipeline.replace(marker,"    record['detector_stage_times_ms']=detection['packet'].get('stage_times_ms',{})\n"+marker)
    (source/'shared_pipeline.py').write_text(pipeline)
    return source


def environment():
    protected=[ROOT/'.runtime/arm.yaml',ROOT/'.runtime/arm-checkpoint.save',ROOT/'.runtime/arm-install-prefix',ROOT/'.runtime/audit/commands.csv']
    return dict(time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),protected_hashes={str(p):sha(p) for p in protected if p.exists()},services=command(['systemctl','--user','show','chess-robot-xiangqi-preview-20261031.service','chess-robot-xiangqi-render-20261031.service','chess_robot_preview.service','-p','Id','-p','ActiveState','-p','MainPID']),processes=command(['ps','-eo','pid,comm,pcpu,pmem,args']),clocks=command(['sudo','-n','jetson_clocks','--show']),power=command(['nvpmodel','-q']),pt_sha256=sha(PT),trt_sha256=sha(TRT))


def candidate(source, directory, label, backend, mask, dds):
    directory.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy();env.pop('XIANGQI_BENCHMARK_LOCK',None)
    deps=BACKEND/'deps';trt=deps/'trt-root'
    env['PYTHONPATH']=':'.join(str(p) for p in [deps/'python',trt/'usr/lib/python3.10/dist-packages',trt/'usr/lib/python3/dist-packages'])+':'+env.get('PYTHONPATH','')
    env['LD_LIBRARY_PATH']=':'.join(str(p) for p in [trt/'usr/lib/aarch64-linux-gnu',trt/'usr/lib/aarch64-linux-gnu/nvidia',trt/'usr/lib/aarch64-linux-gnu/tegra'])+':'+env.get('LD_LIBRARY_PATH','')
    env.update(XIANGQI_RUN_DIR=str(directory),LIVE_AB_MODEL=str(TRT if backend=='trt' else PT),LIVE_AB_LABEL=label,LIVE_AB_MASK_GPU='1' if mask else '0',OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',ROS_DOMAIN_ID='90',ROS_LOCALHOST_ONLY='1')
    if dds:env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(PROFILE)
    else:env.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
    log=(directory/'candidate.log').open('w')
    p=subprocess.Popen([str(RUNTIME),str(source/'live_tracked.py'),'--ros-args','-r','__node:=xiangqi_live_ab_candidate'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    return p,log


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--seconds',type=int,default=30);ap.add_argument('--rounds',type=int,default=3);ap.add_argument('--stable-seconds',type=int,default=120);ap.add_argument('--conditions',default='');args=ap.parse_args()
    out=Path(__file__).parent;source=build_source(out)
    before=environment();(out/'environment_before.json').write_text(json.dumps(before,indent=2))
    assert before['trt_sha256']=='26103821bc0788d1dc0a2d0051c91362530d5b6b1cc92347f2289d12c9469d35'
    assert 'MAXN' in before['power']
    rclpy.init();obs=Observer(out);active=None;log=None;telemetry=None;lock=None;clock_changed=False;snapshot=out/'clock-original.conf';results=[];errors=[]
    def restore():
        nonlocal clock_changed
        if clock_changed:
            command(['sudo','-n','jetson_clocks','--restore',snapshot])
            show=command(['sudo','-n','jetson_clocks','--show']);(out/'clock_after_restore.txt').write_text(show)
            text=command(['sudo','-n','python3',RESOURCE/'verify_restore.py',snapshot,out/'clock_before.txt',out/'clock_after_restore.txt'])
            with (out/'restoration_checks.log').open('a') as f:f.write(text)
            clock_changed=False
    def handler(signum,frame):raise KeyboardInterrupt(f'signal {signum}')
    signal.signal(signal.SIGINT,handler);signal.signal(signal.SIGTERM,handler)
    try:
        obs.wait(6)
        assert obs.stationary(),'No fresh measured stationary feedback; fixed-clock testing not started.'
        assert sum(r.get('status')==200 and bool(r.get('seq')) for t,r in obs.http)>10,'No actual HTTP JPEG traffic in preflight'
        print('PHASE production_baseline',flush=True)
        prod=obs.measure(out/'production_baseline',args.seconds);print(json.dumps(dict(label='production_baseline',fps=prod['completion_hz'])),flush=True)
        lock=LOCK.open('a')
        def lock_timeout(signum,frame):raise TimeoutError('benchmark lock unavailable for 30 seconds')
        old_alarm=signal.signal(signal.SIGALRM,lock_timeout);signal.alarm(30)
        try:fcntl.flock(lock,fcntl.LOCK_EX)
        finally:signal.alarm(0);signal.signal(signal.SIGALRM,old_alarm)
        obs.wait(2)
        command(['sudo','-n','jetson_clocks','--store',snapshot]);assert snapshot.stat().st_size>0
        (out/'clock_before.txt').write_text(command(['sudo','-n','jetson_clocks','--show']))
        tlog=(out/'tegrastats.log').open('w');telemetry=subprocess.Popen(['tegrastats','--interval','1000'],stdout=tlog,stderr=subprocess.STDOUT,start_new_session=True)
        conditions=[('pt_dynamic','pt',False,False,False),('pt_fixed','pt',True,False,False),('trt_fixed','trt',True,False,False),('trt_dynamic','trt',False,False,False),('mask_gpu_dynamic','pt',False,True,False),('dds_dynamic','pt',False,False,True)]
        if args.conditions:
            wanted=args.conditions.split(',');conditions=[c for c in conditions if c[0] in wanted];assert len(conditions)==len(wanted)
        for rnd in range(args.rounds):
            order=conditions if rnd%2==0 else list(reversed(conditions))
            for label,backend,fixed,mask,dds in order:
                assert obs.stationary(),'Measured feedback no longer fresh/stationary'
                if fixed and not clock_changed:
                    clock_changed=True
                    command(['sudo','-n','jetson_clocks'])
                    command(['sudo','-n','python3',RESOURCE/'restore_cpuidle.py',snapshot,'--apply'])
                elif not fixed:restore()
                phase=out/f'round_{rnd+1}_{label}';print('PHASE',phase.name,flush=True)
                active,log=candidate(source,phase/'runtime',label,backend,mask,dds)
                deadline=time.monotonic()+50;initial=time.monotonic()
                while True:
                    obs.wait(.25,active)
                    with obs.lock:fresh=[r for t,r in obs.rows if t>=initial and r.get('compute',{}).get('live_ab_label')==label]
                    if len(fresh)>=20 and time.monotonic()-initial>=10:break
                    if time.monotonic()>deadline:raise RuntimeError('candidate warmup timed out')
                measurement=obs.measure(phase/'measurement',args.seconds,active,label)
                measurement.update(label=label,backend=backend,fixed=fixed,mask_gpu=mask,explicit_dds=dds,round=rnd+1)
                results.append(measurement);(out/'running_results.json').write_text(json.dumps(results,indent=2))
                print(json.dumps(dict(label=label,round=rnd+1,fps=measurement['completion_hz'],latency_p95=measurement['latency_host_sdk_stamp_to_publish_ms']['p95'])),flush=True)
                stop_process(active);active=None;log.close();log=None
        grouped={label:[r for r in results if r['label']==label] for label,*_ in conditions}
        rankings={label:dict(trial_fps=[r['completion_hz'] for r in rows],median_fps=statistics.median(r['completion_hz'] for r in rows),median_latency_p95_ms=statistics.median(r['latency_host_sdk_stamp_to_publish_ms']['p95'] for r in rows)) for label,rows in grouped.items()}
        best=max(rankings,key=lambda x:rankings[x]['median_fps']);spec=next(c for c in conditions if c[0]==best)
        label,backend,fixed,mask,dds=spec
        if fixed and not clock_changed:
            clock_changed=True;command(['sudo','-n','jetson_clocks']);command(['sudo','-n','python3',RESOURCE/'restore_cpuidle.py',snapshot,'--apply'])
        elif not fixed:restore()
        print('PHASE stability',best,flush=True)
        active,log=candidate(source,out/'stability_runtime',best,backend,mask,dds)
        initial=time.monotonic();deadline=initial+50
        while True:
            obs.wait(.25,active)
            with obs.lock:fresh=[r for t,r in obs.rows if t>=initial and r.get('compute',{}).get('live_ab_label')==best]
            if len(fresh)>=20 and time.monotonic()-initial>=10:break
            if time.monotonic()>deadline:raise RuntimeError('stability candidate warmup timed out')
        stable=obs.measure(out/'stability',args.stable_seconds,active,best)
        stop_process(active);active=None;log.close();log=None;restore()
        (out/'comparison.json').write_text(json.dumps(dict(status='complete',production_baseline=prod,rankings=rankings,best_condition=best,stability=stable,results=results,scope='live camera + unchanged decoupled geometry/tracker + existing renderer + one controlled HTTP JPEG client; no robot commands',limitations=['Actual scene, not labelled independent-scene accuracy test.','FrameInfo arrival rate is not four-topic full matching rate.','Observer uses small metadata only during timed windows.','Strict FP32 adapter is used in all controlled conditions; native production baseline is reported separately.','Stability duration is bounded and not a long-term certification.']),indent=2))
    except BaseException as exc:
        errors.append(f'{type(exc).__name__}: {exc}');(out/'failure.json').write_text(json.dumps(dict(errors=errors,completed_phases=results),indent=2));raise
    finally:
        stop_process(active)
        if log:log.close()
        try:restore()
        finally:
            stop_process(telemetry)
            if lock:fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
            obs.wait(4)
            after=environment();(out/'environment_after.json').write_text(json.dumps(after,indent=2))
            protected_equal=before['protected_hashes']==after['protected_hashes'];(out/'protected_check.json').write_text(json.dumps(dict(unchanged=protected_equal,before=before['protected_hashes'],after=after['protected_hashes'],services_unchanged=before['services']==after['services']),indent=2))
            obs.close();rclpy.shutdown()
            print('RESTORED',json.dumps(dict(protected_equal=protected_equal,services_unchanged=before['services']==after['services'])),flush=True)


if __name__=='__main__':main()
