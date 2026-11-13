"""Persistent 3-D track IDs. Association never uses detection list indices as identity."""
import json,math
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment

def update_surface_normals(pairs):
    if not pairs:return []
    normals=np.array([d['base_surface_normal'] for t,d in pairs])
    normals/=np.linalg.norm(normals,axis=1)[:,None]
    histories=np.zeros((len(pairs),12,3));counts=[]
    for i,((t,d),normal) in enumerate(zip(pairs,normals)):
        history=(t.get('normal_history',[])+[normal.tolist()])[-12:]
        t['normal_history']=history;counts.append(len(history))
        histories[i,:len(history)]=history
    counts=np.array(counts)
    mean=histories.sum(axis=1)/counts[:,None]
    mean/=np.linalg.norm(mean,axis=1)[:,None]
    angles=np.degrees(np.arccos(np.clip((histories*mean[:,None,:]).sum(axis=2),-1,1)))
    angles[np.arange(12)[None,:]>=counts[:,None]]=0.
    spread=np.sqrt((angles**2).sum(axis=1)/counts)
    blended=.7*np.array([t['normal'] for t,d in pairs])+.3*normals
    unit=blended/np.linalg.norm(blended,axis=1)[:,None]
    tilt=np.degrees(np.arccos(np.clip(unit[:,2],-1,1)))
    for (t,d),normal in zip(pairs,blended):t['normal']=normal.tolist()
    return zip(unit,tilt,spread,counts)


class PieceTracker:
    def __init__(self,path=None):
        self.path=Path(path) if path else None;self.tracks={};self.next_id=1
        if self.path and self.path.exists():
            data=json.loads(self.path.read_text());assert data['schema']=='xiangqi_tracks/v1'
            self.tracks={int(t['id']):t for t in data['tracks']};self.next_id=data['next_id']
        self.startup_ids=set(self.tracks);self.startup_frames_left=3
    def update(self,detections,stamp_ns):
        current=[d for d in detections if d.get('valid') and d.get('complete_center_target')
                 and d.get('classification_valid',True) and 'base_surface_center_m' in d]
        active=[t for t in self.tracks.values() if 0<=(stamp_ns-t['last_ns'])*1e-9 and ((stamp_ns-t['last_ns'])*1e-9<60 or (self.startup_frames_left>0 and t['id'] in self.startup_ids))]
        costs=np.full((len(active),len(current)),1000.)
        if active and current:
            dt=np.array([(stamp_ns-t['last_ns'])*1e-9 for t in active])
            resuming=np.array([d>=60 and t['id'] in self.startup_ids for d,t in zip(dt,active)])
            previous=np.array([t['position_m'] for t in active])
            velocities=np.array([t['velocity_m_s'] for t in active])
            predicted=previous+velocities*np.where(resuming,0.,np.minimum(dt,.3))[:,None]
            gate=np.where(resuming,.005,np.minimum(.085,.018+.20*dt))
            positions=np.array([d['base_surface_center_m'] for d in current])
            distances=np.linalg.norm(positions[None,:,:]-predicted[:,None,:],axis=2)
            height_gap=np.abs(positions[None,:,2]-previous[:,None,2])
            old_camps=np.array([max(t['class_votes'],key=t['class_votes'].get).split('_')[0] for t in active])
            new_camps=np.array([d['class_name'].split('_')[0] for d in current])
            costs=np.where((distances<=gate[:,None]) & (height_gap<=.025),
                           distances+.02*(old_camps[:,None]!=new_camps[None,:]),1000.)
        matched=set();pairs=[]
        if costs.size:
            rows,cols=linear_sum_assignment(costs)
            for i,j in zip(rows,cols):
                if costs[i,j]>=999:continue
                # Refuse a near-tie between two existing tracks instead of silently swapping IDs.
                choices=sorted(costs[:,j])
                if len(choices)>1 and choices[1]<999 and choices[1]-choices[0]<.006:
                    current[j]['identity_ambiguous']=True;continue
                pairs.append((active[i],current[j]));matched.add(j)
        for j in sorted((j for j in range(len(current)) if j not in matched),key=lambda j:tuple(current[j]['base_surface_center_m'])):
            d=current[j]
            if d.get('identity_ambiguous'):continue
            track=dict(id=self.next_id,position_m=d['base_surface_center_m'],velocity_m_s=[0.,0.,0.],first_ns=stamp_ns,last_ns=stamp_ns,hits=0,class_votes={},normal=[0.,0.,1.])
            self.tracks[self.next_id]=track;self.next_id+=1;pairs.append((track,d))
        surface=update_surface_normals(pairs)
        for (t,d),(n,tilt,spread,normal_count) in zip(pairs,surface):
            dt=(stamp_ns-t['last_ns'])*1e-9;p=np.array(d['base_surface_center_m'])
            d['reacquired_by_position_after_restart']=bool(dt>=60 and t['id'] in self.startup_ids)
            self.startup_ids.discard(t['id'])
            if t['hits'] and 0<dt<1:
                velocity=(p-np.array(t['position_m']))/dt
                if np.linalg.norm(velocity)<.4:t['velocity_m_s']=(.5*np.array(t['velocity_m_s'])+.5*velocity).tolist()
            else:t['velocity_m_s']=[0.,0.,0.]
            t['position_m']=p.tolist();t['last_ns']=stamp_ns;t['hits']+=1
            votes={k:v*.94 for k,v in t['class_votes'].items()};name=d['class_name'];votes[name]=votes.get(name,0)+float(d['confidence']);t['class_votes']=votes
            label=max(votes,key=votes.get);fraction=votes[label]/sum(votes.values())
            d.update(track_id=t['id'],track_confirmed=t['hits']>=3,track_hits=t['hits'],track_class_name=label,track_class_vote_fraction=fraction,position_stamp_ns=stamp_ns,orientation_observed='experimental_depth_surface_normal_only',surface_normal_base=n.tolist(),surface_tilt_deg=float(tilt),normal_spread_deg=float(spread),normal_temporally_stable=bool(normal_count>=8 and spread<1.5),orientation_valid_for_motion=False,yaw_observed=False,yaw_rad=None)
        for d in detections:
            d.setdefault('track_id',None);d.setdefault('track_confirmed',False)
        if current:self.startup_frames_left=max(0,self.startup_frames_left-1)
        if self.path:
            data=dict(schema='xiangqi_tracks/v1',next_id=self.next_id,tracks=list(self.tracks.values()))
            tmp=self.path.with_suffix('.tmp');tmp.write_text(json.dumps(data,separators=(',', ':'),allow_nan=False));tmp.replace(self.path)
        return detections

def check():
    def d(x,y,label='red_pawn'):
        return dict(valid=True,complete_center_target=True,base_surface_center_m=[x,y,.01],base_surface_normal=[0.,0.,1.],class_name=label,confidence=.9)
    tracker=PieceTracker();start=100000000000
    first=tracker.update([d(.1,.1),d(.2,.1)],start);a,b=[x['track_id'] for x in first]
    assert a!=b
    second=tracker.update([d(.202,.1,'red_king'),d(.102,.1)],start+100000000)
    assert [x['track_id'] for x in second]==[b,a]
    third=tracker.update([d(.204,.1),d(.104,.1)],start+200000000)
    assert [x['track_id'] for x in third]==[b,a] and all(x['track_confirmed'] for x in third)
    fourth=tracker.update([d(.107,.1)],start+300000000);assert fourth[0]['track_id']==a
    fifth=tracker.update([d(.208,.1),d(.110,.1)],start+400000000);assert [x['track_id'] for x in fifth]==[b,a]
    assert all(x['surface_tilt_deg']==0 and not x['yaw_observed'] for x in fifth)
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        path=Path(directory)/'tracks.json';saved=PieceTracker(path);old=saved.update([d(.1,.1)],start)[0]['track_id']
        restored=PieceTracker(path);assert restored.update([d(.102,.1)],start+100000000)[0]['track_id']==old
        resumed=PieceTracker(path);again=resumed.update([d(.103,.1)],start+100000000000)[0];assert again['track_id']==old and again['reacquired_by_position_after_restart']
        moved=PieceTracker(path);assert moved.update([d(.13,.1)],start+200000000000)[0]['track_id']!=old
    assert all(not x['orientation_valid_for_motion'] for x in fifth)
    print('PASS: order changes, same-class movement, class flip, brief occlusion, and surface-normal/yaw semantics')
if __name__=='__main__':check()
