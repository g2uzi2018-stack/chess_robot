"""Persistent 3-D track IDs. Association never uses detection list indices as identity."""
import json,math
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment

class PieceTracker:
    def __init__(self,path=None):
        self.path=Path(path) if path else None;self.tracks={};self.next_id=1
        if self.path and self.path.exists():
            data=json.loads(self.path.read_text());assert data['schema']=='xiangqi_tracks/v1'
            self.tracks={int(t['id']):t for t in data['tracks']};self.next_id=data['next_id']
        self.startup_ids=set(self.tracks);self.startup_frames_left=3
    def update(self,detections,stamp_ns):
        current=[d for d in detections if d.get('valid') and d.get('complete_center_target') and 'base_surface_center_m' in d]
        active=[t for t in self.tracks.values() if 0<=(stamp_ns-t['last_ns'])*1e-9 and ((stamp_ns-t['last_ns'])*1e-9<60 or (self.startup_frames_left>0 and t['id'] in self.startup_ids))]
        costs=np.full((len(active),len(current)),1000.)
        for i,t in enumerate(active):
            dt=(stamp_ns-t['last_ns'])*1e-9
            resuming=dt>=60 and t['id'] in self.startup_ids
            predicted=np.array(t['position_m']) if resuming else np.array(t['position_m'])+np.array(t['velocity_m_s'])*min(dt,.3)
            gate=.005 if resuming else min(.085,.018+.20*dt)
            for j,d in enumerate(current):
                p=np.array(d['base_surface_center_m']);distance=float(np.linalg.norm(p-predicted))
                if distance>gate or abs(p[2]-t['position_m'][2])>.025:continue
                old=max(t['class_votes'],key=t['class_votes'].get);new=d['class_name']
                camp_penalty=.02 if old.split('_')[0]!=new.split('_')[0] else 0.
                costs[i,j]=distance+camp_penalty
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
        for t,d in pairs:
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
            normal=np.array(d['base_surface_normal']);normal/=np.linalg.norm(normal)
            history=t.get('normal_history',[])+[normal.tolist()];t['normal_history']=history[-12:]
            normals=np.array(t['normal_history']);mean_normal=normals.mean(0);mean_normal/=np.linalg.norm(mean_normal)
            spread=float(np.sqrt(np.mean(np.degrees(np.arccos(np.clip(normals@mean_normal,-1,1)))**2)))
            t['normal']=(.7*np.array(t['normal'])+.3*normal).tolist();n=np.array(t['normal']);n/=np.linalg.norm(n)
            tilt=math.degrees(math.acos(float(np.clip(n[2],-1,1))))
            d.update(track_id=t['id'],track_confirmed=t['hits']>=3,track_hits=t['hits'],track_class_name=label,track_class_vote_fraction=fraction,position_stamp_ns=stamp_ns,orientation_observed='experimental_depth_surface_normal_only',surface_normal_base=n.tolist(),surface_tilt_deg=tilt,normal_spread_deg=spread,normal_temporally_stable=len(history)>=8 and spread<1.5,orientation_valid_for_motion=False,yaw_observed=False,yaw_rad=None)
        for d in detections:
            d.setdefault('track_id',None);d.setdefault('track_confirmed',False)
        if current:self.startup_frames_left=max(0,self.startup_frames_left-1)
        if self.path:
            data=dict(schema='xiangqi_tracks/v1',next_id=self.next_id,tracks=list(self.tracks.values()))
            tmp=self.path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2,allow_nan=False));tmp.replace(self.path)
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
