"""Passive, bounded observation state machines. This module has no motion API."""
import copy
import hashlib
import json
import math
import uuid
import struct
from pathlib import Path
from collections import Counter, deque
from dataclasses import asdict, dataclass
import numpy as np
from scipy.optimize import linear_sum_assignment
from qualification import measured_stopped


@dataclass(frozen=True)
class Policy:
    state_age_s: float = .2
    settle_s: float = .6
    pose_drift_rad: float = .001
    retire_gap_s: float = 2.
    sample_s: float = 2.
    min_frames: int = 20
    max_gap_s: float = .3
    min_valid_fraction: float = .8
    position_p95_m: float = .003
    normal_rms_deg: float = 1.5
    scene_distance_m: float = .006
    scene_age_s: float = .5
    lock_lifetime_s: float = 60.


def observation_roi(boxes,width=1280,height=720):
    boxes=np.asarray(boxes);lo=boxes[:,:2].min(0);hi=boxes[:,2:].max(0)
    size=max(480,int(np.ceil((max(hi-lo)+160)/32))*32)
    if size>min(width,height):return None
    cx,cy=np.round((lo+hi)/2/32)*32
    x=max(0,min(width-size,int(cx-size/2)));y=max(0,min(height-size,int(cy-size/2)))
    return [x,y,x+size,y+size]


class FeedbackWindow:
    """Zero order hold inside a verified stationary interval, never timestamp renewal.

    epoch must come from a passive controller sequence observer; a ROS receive
    stamp alone cannot reveal a controller reboot hidden by the driver.
    """
    def __init__(self, policy=None):
        self.p = policy or Policy()
        self.rows = deque(maxlen=128)
        self.start = None
        self.anchor = None
        self.epoch = None
        self.identity = None
        self.generation = 0
        self.reason = 'waiting for measured feedback'
        self.retire_serial = 0
        self.last = 0
        self.counters = Counter()

    def _break(self, reason, retire=False):
        if self.start is not None or reason != self.reason:
            self.generation += 1
        self.start = None
        self.anchor = None
        self.reason = reason
        if retire:
            self.retire_serial += 1

    def ingest(self, t, arm, joint, now, epoch):
        if t < self.last and epoch == self.epoch:
            self.counters['duplicate_or_reordered'] += 1
            return
        identity = (arm.get('backend'), arm.get('reference_valid'),
                    arm.get('simulated'), arm.get('controller_mode'), arm.get('busy'))
        if epoch is None:
            self._break('controller epoch unverified')
            return
        if self.epoch is not None and (epoch != self.epoch or identity != self.identity):
            self._break('controller epoch or reference/policy identity changed', True)
            self.rows.clear(); self.last = 0
        self.epoch, self.identity = epoch, identity
        if t <= self.last:
            # Transport reorder and duplicate publication do not renew freshness.
            self.counters['duplicate_or_reordered'] += 1
            if not measured_stopped(arm, joint, 0):
                self._break('untrusted or moving feedback', True)
            return
        names = joint.get('name', [])
        if not measured_stopped(arm, joint, 0):
            self.last = t
            self._break('untrusted or moving feedback', True)
            return
        if not 0 <= (now-t)*1e-9 <= self.p.state_age_s:
            self.counters['late_feedback_callback'] += 1
            return
        q = np.array([joint['position'][names.index('joint'+str(i))] for i in range(1,7)])
        previous = self.rows[-1] if self.rows else None
        gap = (t-self.last)*1e-9 if self.last else 0.
        if previous and gap > 0:
            dq = float(np.max(np.abs(q-previous['q'])))
            if gap > self.p.retire_gap_s:
                self._break('long feedback gap', True)
                self.rows.clear()
            elif gap > self.p.state_age_s:
                if dq > self.p.pose_drift_rad:
                    self._break('pose changed across feedback gap', True)
                else:
                    # Preserve historical identity and snapshot, then settle
                    # again before restoring current eligibility.
                    self.start=None;self.anchor=None;self.reason='feedback gap; pose recheck'
            elif dq / gap > .002:
                self._break('measured joint motion', True)
        if self.anchor is not None and np.max(np.abs(q-self.anchor)) > self.p.pose_drift_rad:
            self._break('stationary pose drift', True)
        if self.start is None:
            self.start = t; self.anchor = q.copy()
        self.last = t
        self.rows.append(dict(t=t, arm=copy.deepcopy(arm), joint=copy.deepcopy(joint), q=q))
        self.reason = 'settling'
        self.counters['unique_feedback'] += 1

    def at(self, image_t, now):
        reason = None
        if self.start is None: reason = self.reason
        elif not 0 <= (now-self.last)*1e-9 <= self.p.state_age_s: reason = 'feedback stale'
        elif (self.last-self.start)*1e-9 < self.p.settle_s: reason = 'settling'
        elif image_t < self.start: reason = 'image predates stationary interval'
        row = next((r for r in reversed(self.rows) if r['t'] <= image_t), None)
        if reason is None and (row is None or not 0 <= (image_t-row['t'])*1e-9 <= self.p.state_age_s):
            reason = 'no sufficiently recent preceding feedback'
        # The image-time row may age while processing. Current stationary
        # continuity is proved by self.last, without renewing the row timestamp.
        return dict(ok=reason is None, reason=reason or 'stationary measured state held',
                    generation=self.generation, epoch=self.epoch,
                    stationary_start_ns=self.start, feedback_stamp_ns=row['t'] if row else None,
                    row=row)


class AuditEpoch:
    """Read only the running driver's existing rotating audit, never a TCP socket.

    An open descriptor drains the old inode before following a rotation. Missing
    audit continuity suspends qualification. Tokens identify observed epochs,
    and are deliberately not claimed to be controller hardware boot IDs.
    """
    def __init__(self,path):
        self.path=Path(path);self.stream=None;self.token=uuid.uuid4().hex
        self.controller=None;self.host=0;self.epoch_start_ns=0;self.ok=False
        self.q=None;self.reason='waiting for passive controller epoch evidence'

    def poll(self,now):
        try:
            stat=self.path.stat()
            if self.stream is None:
                self.stream=self.path.open('rb')
                self.stream.seek(max(0,(stat.st_size//1448-1)*1448))
            elif stat.st_ino==__import__('os').fstat(self.stream.fileno()).st_ino and stat.st_size<self.stream.tell():
                self.stream.seek(0);self.token=uuid.uuid4().hex;self.epoch_start_ns=0;self.controller=None;self.ok=False
            # Drain old descriptor and then follow the new path, at most twice.
            for _ in range(2):
                for _ in range(512):
                    pos=self.stream.tell();raw=self.stream.read(1448)
                    if len(raw)!=1448:self.stream.seek(pos);break
                    host=struct.unpack_from('<q',raw)[0];frame=memoryview(raw)[8:]
                    if struct.unpack_from('<H',frame)[0]!=1440 or struct.unpack_from('<Q',frame,48)[0]!=0x0123456789ABCDEF:
                        raise ValueError('invalid existing feedback audit framing')
                    controller=struct.unpack_from('<Q',frame,32)[0]
                    if self.controller is not None and (controller<self.controller or host-self.host>2e9 or host<self.host):
                        self.token=uuid.uuid4().hex;self.epoch_start_ns=host
                    if not self.epoch_start_ns:self.epoch_start_ns=host
                    if controller==self.controller:continue
                    previous_q=self.q;previous_host=self.host
                    self.controller=controller;self.host=host
                    self.q=np.radians(struct.unpack_from('<6d',frame,432))
                    speed=np.radians(struct.unpack_from('<6d',frame,480));mode=struct.unpack_from('<Q',frame,24)[0]
                    good=bool(mode in (4,5) and np.isfinite(self.q).all() and np.isfinite(speed).all() and max(abs(speed))<=.002)
                    if previous_q is not None and host>previous_host:
                        good=good and max(abs(self.q-previous_q))/((host-previous_host)*1e-9)<=.002
                    if self.ok and not good:
                        self.token=uuid.uuid4().hex;self.epoch_start_ns=host
                    self.ok=bool(good)
                if self.path.stat().st_ino==__import__('os').fstat(self.stream.fileno()).st_ino:break
                self.stream.close();self.stream=self.path.open('rb')
            fresh=0 <= (now-self.host)*1e-9 <= .2
            self.reason='passive controller audit current' if fresh and self.ok else 'passive controller audit stale or moving'
            return fresh and self.ok
        except (OSError,ValueError,struct.error) as error:
            self.reason=str(error);return False

    def close(self):
        if self.stream:self.stream.close()


def fingerprint(record, policy):
    data = dict(identity=record.get('identity'), K=record.get('source_K'),
                camera_frame=record.get('camera_frame_id'),base_frame=record.get('base_frame_id'),
                image_size=record.get('image_size'),camera_epoch=record.get('camera_epoch'),policy=asdict(policy))
    return hashlib.sha256(json.dumps(data,sort_keys=True,allow_nan=False).encode()).hexdigest()


class StableObservation:
    def __init__(self, policy=None, save=None):
        self.p = policy or Policy(); self.save = save or (lambda event: None)
        self.requested = True; self.frames = []; self.snapshot = None
        self.state = 'collecting'; self.reason = 'waiting for stopped valid observations'
        self.consumed_by = None; self.revoked = False; self.pending = True
        self.last_scene_ns = 0; self.mismatch = 0; self.last_t = 0
        self.generation = None; self.config = None; self.epoch = None
        self.last_status = None; self.counts = Counter()
        self.operations = {}
        self.frame_keys=deque(maxlen=600)

    def invalidate(self, reason):
        self.frames.clear(); self.revoked = True; self.pending = True
        self.state = 'needs_resample'; self.reason = reason

    def collect(self):
        self.frames.clear(); self.requested = True; self.revoked = True
        self.pending = True; self.state = 'collecting'; self.reason = 'new two second collection requested'

    def tick(self, now, trusted, generation, epoch):
        if self.generation is not None and (generation != self.generation or epoch != self.epoch):
            self.invalidate('motion, feedback continuity or epoch changed')
            self.requested = True
        self.generation, self.epoch = generation, epoch
        if not trusted:
            self.pending = True
            self.state = 'pending_confirmation'; self.reason = 'fresh stopped feedback required'
        elif self.requested and self.state=='pending_confirmation':
            self.state='collecting';self.reason='waiting for valid independent RGB-D geometry'
        if self.snapshot and not self.revoked and not self.consumed_by and (now-self.snapshot['locked_ns'])*1e-9 > self.p.lock_lifetime_s:
            self.invalidate('snapshot lifetime exceeded')

    def _scene_matches(self, record):
        old = self.snapshot['targets']
        current = [d for d in record['detections'] if d.get('camera_complete_center_target')]
        if len(current) != len(old) or not current: return False
        costs = np.linalg.norm(np.array([d['camera_surface_center_m'] for d in current])[:,None,:]
                               - np.array([d['camera_surface_center_m'] for d in old])[None,:,:],axis=2)
        rows, cols = linear_sum_assignment(costs)
        if np.max(costs[rows,cols]) > self.p.scene_distance_m: return False
        for row,col in zip(rows,cols):
            if (old[col]['class_vote_fraction']>=.7 and current[row].get('confidence',0)>=.6
                    and old[col]['class_name']!=current[row].get('class_name')):return False
        # Close ties are unobservable identity, never quietly inherited.
        if len(old)>1 and np.any(np.diff(np.sort(costs,axis=1),axis=1)[:,0] < .006): return False
        return True

    def observe(self, record, now, trusted, generation, epoch):
        self.tick(now,trusted,generation,epoch)
        t = record['stamp_ns']
        key=tuple(record.get('rgbd_frame_key') or ('stamp',t))
        if t <= self.last_t or key in self.frame_keys:
            self.counts['duplicate_image'] += 1; return
        self.frame_keys.append(key)
        self.last_t = t
        config = fingerprint(record,self.p)
        if self.config is not None and config != self.config:
            self.invalidate('configuration, model, intrinsics or extrinsics changed');self.requested=True
        self.config = config
        fresh = record.get('detection_valid',False) and 0 <= (now-t)*1e-9 <= .5
        if self.snapshot and not self.revoked and fresh:
            matches = self._scene_matches(record)
            self.last_scene_ns = t; self.pending = not matches or not trusted
            self.mismatch = 0 if matches else self.mismatch+1
            if self.mismatch >= 3:
                self.invalidate('scene changed or scene identity ambiguous')
                self.requested = True
        if not self.requested:
            if not self.revoked and not self.pending:
                self.state = 'consumed' if self.consumed_by else 'locked'
                self.reason = 'immutable snapshot; one operation only'
            return
        good = [d for d in record['detections'] if d.get('valid') and d.get('complete_center_target')
                and d.get('track_confirmed') and not d.get('identity_ambiguous')]
        if not (fresh and trusted and record.get('base_transform_valid') and good):
            self.counts['ineligible_frame'] += 1
            if self.frames and (t-self.frames[-1]['stamp_ns'])*1e-9 > self.p.max_gap_s:
                self.frames.clear();self.reason='observation gap exceeded'
            return
        ids = {d['track_id'] for d in good}
        if self.frames:
            gap = (t-self.frames[-1]['stamp_ns'])*1e-9
            if gap > self.p.max_gap_s or not ids <= set(self.frames[0]['targets']) or len(record['detections'])>self.frames[0]['detection_count']:
                self.frames.clear();self.reason='gap or physical target set changed'
        if not self.frames and len(good)!=len(record['detections']):return
        self.frames.append(dict(stamp_ns=t,targets={d['track_id']:copy.deepcopy(d) for d in good},
                                q=record['source_joint_position_rad'],
                                detection_count=len(record['detections']),
                                frame_index=record.get('observation_frame_index'),
                                feedback_stamp_ns=record['feedback_stamp_ns']))
        # Bound memory even if caller supplies a nonsensical very high frame rate.
        if len(self.frames)>600:self.frames=self.frames[-600:]
        self.state='collecting'; self.reason='collecting independent RGB-D frames'
        if len(self.frames)<self.p.min_frames or (t-self.frames[0]['stamp_ns'])*1e-9<self.p.sample_s:return
        self._lock(record,now)

    def _lock(self,record,now):
        frames=self.frames
        if np.max(np.ptp(np.array([f['q'] for f in frames]),axis=0))>self.p.pose_drift_rad:
            self.frames=[];self.counts['pose_drift']+=1;return
        # Include rejected/empty unique images in the denominator.
        first_index=frames[0]['frame_index']
        seen = record.get('observation_frame_index',0)-first_index+1 if first_index is not None else len(frames)
        # Frame index span is supplied by integration; standalone fixtures use frame count.
        total=max(len(frames),seen)
        targets=[]
        for tid in frames[0]['targets']:
            ds=[f['targets'][tid] for f in frames if tid in f['targets']]
            if len(ds)/total<self.p.min_valid_fraction:
                self.frames=[];self.counts['valid_fraction']+=1;return
            times=[f['stamp_ns'] for f in frames if tid in f['targets']]
            if max([times[0]-frames[0]['stamp_ns'],frames[-1]['stamp_ns']-times[-1]]+
                   [b-a for a,b in zip(times,times[1:])])*1e-9>self.p.max_gap_s:
                self.frames=[];self.counts['target_gap']+=1;return
            target=dict(track_id=tid,samples=len(ds),valid_fraction=len(ds)/total)
            for key in ('camera_surface_center_m','base_surface_center_m'):
                points=np.array([d[key] for d in ds]);median=np.median(points,axis=0)
                radial=np.linalg.norm(points-median,axis=1);p95=float(np.percentile(radial,95))
                if p95>self.p.position_p95_m:
                    self.frames=[];self.counts['position_dispersion']+=1;return
                target[key]=median.tolist();target[key+'_mad_m']=np.median(np.abs(points-median),axis=0).tolist()
                target[key+'_radial_p95_m']=p95
            for key in ('camera_surface_normal','base_surface_normal'):
                normals=np.array([d[key] for d in ds]);n=np.median(normals,axis=0);length=np.linalg.norm(n)
                if length<.5:self.frames=[];return
                n/=length;rms=float(np.sqrt(np.mean(np.degrees(np.arccos(np.clip(normals@n,-1,1)))**2)))
                target[key]=n.tolist();target[key+'_rms_deg']=rms
                target[key+'_temporally_stable']=rms<=self.p.normal_rms_deg
            votes=Counter()
            for d in ds:votes[d['class_name']]+=d['confidence']
            name=votes.most_common(1)[0][0]
            target.update(class_name=name,class_vote_fraction=votes[name]/sum(votes.values()),
                          plane_rms_m_max=max(d['plane_rms_m'] for d in ds),
                          class_verified=False,yaw_observed=False,orientation_valid_for_motion=False,
                          motion_ready=False,grasp_ready=False)
            targets.append(target)
        data=dict(schema='xiangqi_stable_snapshot/v1',snapshot_id=uuid.uuid4().hex,
                  sample_start_ns=frames[0]['stamp_ns'],sample_end_ns=frames[-1]['stamp_ns'],locked_ns=now,
                  unique_frames=len(frames),unique_feedback_states=len({f['feedback_stamp_ns'] for f in frames}),
                  max_observation_gap_s=max((b['stamp_ns']-a['stamp_ns'])*1e-9 for a,b in zip(frames,frames[1:])),
                  controller_epoch=self.epoch,stationary_generation=self.generation,
                  configuration_sha256=self.config,identity=record.get('identity'),policy=asdict(self.p),
                  joint_position_rad_median=np.median([f['q'] for f in frames],axis=0).tolist(),
                  frame_id='base_link',camera_frame_id=record.get('camera_frame_id'),targets=targets,
                  stamp_semantics='host SDK acquisition stamp; not exposure time',
                  point_semantics='visible top surface center; not grasp TCP',
                  motion_ready=False,grasp_ready=False)
        self.snapshot=copy.deepcopy(data);self.consumed_by=None;self.revoked=False;self.pending=False
        self.last_scene_ns=record['stamp_ns'];self.mismatch=0;self.requested=False;self.frames=[]
        self.state='locked';self.reason='two second immutable snapshot locked';self.counts['locked']+=1
        self.save(dict(event='locked',snapshot=data))

    def status(self,now):
        scene_fresh=0 <= (now-self.last_scene_ns)*1e-9 <= self.p.scene_age_s
        eligible=bool(self.snapshot and not self.revoked and not self.pending and not self.consumed_by and scene_fresh
                      and 0 <= (now-self.snapshot['locked_ns'])*1e-9 <= self.p.lock_lifetime_s)
        missing_scene=bool(self.snapshot and not self.revoked and not self.consumed_by and not scene_fresh)
        return dict(state='pending_confirmation' if missing_scene else self.state,
                    reason='fresh scene confirmation required' if missing_scene else self.reason,snapshot_id=self.snapshot['snapshot_id'] if self.snapshot else None,
                    observation_eligible=eligible,scene_confirmation_fresh=scene_fresh,consumed_by=self.consumed_by,
                    sample_frames=len(self.frames),sample_elapsed_s=(self.frames[-1]['stamp_ns']-self.frames[0]['stamp_ns'])*1e-9 if self.frames else 0.,
                    counters=dict(self.counts),motion_ready=False,grasp_ready=False)

    def request(self,request,now):
        action=request.get('action');response=dict(request_id=request.get('request_id'),action=action,ok=False)
        if action=='collect':
            self.collect();response['ok']=True
        elif action=='query':
            if self.snapshot and request.get('snapshot_id',self.snapshot['snapshot_id'])==self.snapshot['snapshot_id']:
                response.update(ok=True,snapshot=copy.deepcopy(self.snapshot))
            else:response['error']='snapshot unavailable'
        elif action=='consume':
            op=request.get('operation_id');sid=request.get('snapshot_id')
            if not isinstance(op,str) or not 1<=len(op)<=128:response['error']='operation_id required (1..128 chars)'
            elif op in self.operations:
                old=self.operations[op]
                if old['snapshot_id']==sid:response.update(ok=True,idempotent=True,snapshot=copy.deepcopy(old))
                else:response['error']='operation_id already bound to another snapshot'
            elif not self.snapshot or sid!=self.snapshot['snapshot_id']:response['error']='snapshot_id mismatch'
            elif self.consumed_by==op:
                response.update(ok=True,idempotent=True,snapshot=copy.deepcopy(self.snapshot))
            elif len(self.operations)>=1024:response['error']='operation ledger full; archive and restart visual session'
            elif self.status(now)['observation_eligible']:
                self.consumed_by=op;self.state='consumed';response.update(ok=True,idempotent=False,snapshot=copy.deepcopy(self.snapshot))
                self.operations[op]=copy.deepcopy(self.snapshot)
                self.save(dict(event='consumed',operation_id=op,snapshot=copy.deepcopy(self.snapshot),consumed_ns=now))
            else:response['error']='snapshot not currently eligible or already consumed'
        else:response['error']='action must be collect, query or consume'
        response['status']=self.status(now)
        return response
