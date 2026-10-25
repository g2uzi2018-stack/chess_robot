"""Independent localhost-only Dobot V4 test fixture; never connects to hardware.

Produces simulated feedback and uses URDF FK / numerical IK for protocol tests.
Run with the project's urdf_tools_env Python after sourcing ROS when used with
an rclpy integration client. Test data must stay on an isolated ROS domain.
"""
import json
import math
import pathlib
import re
import socket
import struct
import threading
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class Model:
    def __init__(self, project):
        import sys
        sys.path.insert(0,str(project/'tools/description'))
        from rigid_geometry import origin, rotation
        self.rotation=rotation
        xml=ET.parse(project/'src/chess_robot_description/urdf/nova5.urdf').getroot()
        joints=[xml.find(f"joint[@name='joint{i}']") for i in range(1,7)]
        self.origins=[np.array(origin(j.find('origin'))) for j in joints]
        self.axes=[list(map(float,j.find('axis').get('xyz').split())) for j in joints]
        self.bounds=np.array([[float(j.find('limit').get(k)) for j in joints] for k in ['lower','upper']])

    def transform(self,q):
        t=np.eye(4)
        for a,o,axis in zip(q,self.origins,self.axes):
            t=t@o@np.array(self.rotation(axis,float(a)))
        return t

    def pose(self,q):
        t=self.transform(q)
        return list(t[:3,3]*1000)+list(Rotation.from_matrix(t[:3,:3]).as_euler('xyz',degrees=True))

    def inverse(self,pose,seed):
        target_r=Rotation.from_euler('xyz',pose[3:],degrees=True).as_matrix()
        target_p=np.array(pose[:3])/1000
        def error(q):
            t=self.transform(q)
            return np.r_[(t[:3,3]-target_p)/.85,Rotation.from_matrix(target_r.T@t[:3,:3]).as_rotvec()]
        seed=np.clip(seed,self.bounds[0]+1e-7,self.bounds[1]-1e-7)
        result=least_squares(error,seed,bounds=(self.bounds[0],self.bounds[1]),
                             xtol=1e-10,ftol=1e-10,gtol=1e-10,max_nfev=150)
        if np.linalg.norm(error(result.x))>1e-5:raise ValueError('unreachable fixture IK')
        return result.x


class ControllerFixture:
    def __init__(self,project,out):
        self.project=pathlib.Path(project);self.out=pathlib.Path(out)
        self.out.mkdir(parents=True,exist_ok=True)
        self.model=Model(self.project)
        home=json.loads((self.project/'config/arm/home_reference.json').read_text())
        self.q=np.array(home['joint_positions_rad']);self.start_q=self.q.copy();self.target_q=self.q.copy()
        self.start_time=time.monotonic();self.duration=0
        self.command_id=0;self.stop_count=0;self.actuation=[];self.commands=[];self.forbidden=[]
        self.lock=threading.RLock();self.closed=threading.Event();self.threads=[]
        self.controls={'feedback_enabled':True,'duplicate_every':0,'hold_duplicates':False,
                       'wrong_echo':'','notice':'','reject':'','mode':None}
        self.dashboard=self.listener();self.feedback=self.listener()
        self.info={'host':'127.0.0.1','dashboard_port':self.dashboard.getsockname()[1],
                   'feedback_port':self.feedback.getsockname()[1],'simulation':True}
        (self.out/'endpoints.json').write_text(json.dumps(self.info,indent=2)+'\n')
        self._start(self.accept_dashboard);self._start(self.accept_feedback)

    @staticmethod
    def listener():
        listener=socket.socket();listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        listener.bind(('127.0.0.1',0));listener.listen();listener.settimeout(.2)
        return listener

    def _start(self,target,*args):
        thread=threading.Thread(target=target,args=args,daemon=True)
        self.threads.append(thread);thread.start()

    def state(self):
        elapsed=time.monotonic()-self.start_time
        if self.duration>0 and elapsed<self.duration:
            fraction=max(0,min(1,elapsed/self.duration))
            q=self.start_q+(self.target_q-self.start_q)*fraction
            velocity=(self.target_q-self.start_q)/self.duration
            mode=7
        else:q=self.target_q.copy();velocity=np.zeros(6);mode=5
        self.q=q
        if self.controls['mode'] is not None:mode=self.controls['mode']
        return q.copy(),velocity,mode

    def accept_dashboard(self):
        while not self.closed.is_set():
            try:peer,_=self.dashboard.accept()
            except socket.timeout:continue
            except OSError:return
            self._start(self.serve_dashboard,peer)

    def serve_dashboard(self,peer):
        peer.settimeout(.2);buffer=''
        try:
            while not self.closed.is_set():
                try:chunk=peer.recv(4096)
                except socket.timeout:continue
                if not chunk:return
                buffer+=chunk.decode('ascii')
                if len(buffer)>32768:return
                while ')' in buffer:
                    end=buffer.index(')')+1;command=buffer[:end].strip();buffer=buffer[end:].lstrip(';\r\n ')
                    response=self.command(command)
                    peer.sendall(response.encode('ascii'))
        except (OSError,UnicodeError):pass
        finally:peer.close()

    @staticmethod
    def first_six(command):
        head=command.split('(',1)[1]
        if head.startswith('joint='):head=head[7:]
        values=re.findall(r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?',head)
        return [float(v) for v in values[:6]]

    def command(self,command):
        name=command.split('(',1)[0].lower();payload='';error=0
        with self.lock:
            self.commands.append(command)
            q,velocity,mode=self.state()
            if self.controls['notice'].lower()==name:return 'Control Mode Is Not Tcp\t'
            if self.controls['reject'].lower()==name:error=-10000
            else:
                try:
                    if name=='robotmode':payload=str(mode)
                    elif name=='getangle':payload=self.numbers(np.degrees(q))
                    elif name=='getpose':payload=self.numbers(self.model.pose(q))
                    elif name=='geterrorid':payload='[]'
                    elif name=='getcurrentcommandid':payload=str(self.command_id)
                    elif name=='positivekin':payload=self.numbers(self.model.pose(np.radians(self.first_six(command))))
                    elif name=='inversekin':
                        near=re.search(r'jointnear\s*=\s*\{([^}]+)\}',command,re.I)
                        seed=np.radians([float(x) for x in near[1].split(',')]) if near else q
                        payload=self.numbers(np.degrees(self.model.inverse(self.first_six(command),seed)))
                    elif name=='servoj':
                        target=np.radians(self.first_six(command));assert len(target)==6
                        duration=re.search(r'(?:,|\()\s*t\s*=\s*([-+0-9.eE]+)',command)
                        self.start_q=q;self.target_q=target;self.start_time=time.monotonic()
                        self.duration=float(duration[1]) if duration else .1
                        self.command_id+=1;payload=str(self.command_id);self.actuation.append(command)
                    elif name=='stop':
                        self.start_q=q;self.target_q=q;self.duration=0;self.stop_count+=1
                        self.actuation.append(command)
                    else:
                        self.forbidden.append(command);error=-10000
                except (ValueError,AssertionError,IndexError):error=-1
            echo='GetAngle()' if self.controls['wrong_echo'].lower()==name else command
            return f'{error},{{{payload}}},{echo};'

    @staticmethod
    def numbers(values):return ','.join(f'{float(x):.10f}' for x in values)

    def accept_feedback(self):
        while not self.closed.is_set():
            try:peer,_=self.feedback.accept()
            except socket.timeout:continue
            except OSError:return
            self._start(self.serve_feedback,peer)

    def serve_feedback(self,peer):
        last=None;count=0;stamp=int(time.time()*1000)
        try:
            while not self.closed.wait(.008):
                with self.lock:
                    control=self.controls.copy()
                    if not control['feedback_enabled']:continue
                    q,velocity,mode=self.state()
                    if last and (control['hold_duplicates'] or (control['duplicate_every'] and count%control['duplicate_every']==0)):
                        packet=last
                    else:
                        stamp+=8;frame=bytearray(1440)
                        struct.pack_into('<H',frame,0,1440);struct.pack_into('<Q',frame,24,mode)
                        struct.pack_into('<Q',frame,32,stamp);struct.pack_into('<Q',frame,48,0x0123456789ABCDEF)
                        struct.pack_into('<d',frame,64,.01)
                        struct.pack_into('<6d',frame,192,*np.degrees(self.target_q))
                        struct.pack_into('<6d',frame,432,*np.degrees(q))
                        struct.pack_into('<6d',frame,480,*np.degrees(velocity))
                        struct.pack_into('<6d',frame,624,*self.model.pose(q))
                        struct.pack_into('<Q',frame,1112,self.command_id)
                        packet=bytes(frame);last=packet
                peer.sendall(packet);count+=1
        except OSError:pass
        finally:peer.close()

    def close(self):
        self.closed.set();self.dashboard.close();self.feedback.close()
        for thread in self.threads:thread.join(timeout=2)
        report={'simulation':True,'commands':self.commands,'actuation':self.actuation,
                'stop_count':self.stop_count,'forbidden':self.forbidden}
        (self.out/'fixture_traffic.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('project',type=pathlib.Path);parser.add_argument('output',type=pathlib.Path)
    parser.add_argument('--seconds',type=float,default=30);args=parser.parse_args()
    fixture=ControllerFixture(args.project,args.output)
    print(json.dumps(fixture.info),flush=True)
    try:time.sleep(args.seconds)
    finally:fixture.close()
