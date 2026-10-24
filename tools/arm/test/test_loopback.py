"""Independent localhost V4 fixture. Never connects to a physical controller."""
import json
import math
import pathlib
import re
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import yaml


class Fixture:
    def __init__(self, home):
        self.lock=threading.RLock();self.closed=threading.Event();self.threads=[]
        self.q=list(home);self.start=list(home);self.target=list(home)
        self.duration=0.;self.began=time.monotonic();self.tcp=False;self.mode=4
        self.id=0;self.commands=[];self.writes=[];self.duplicates=False
        self.dashboard=self.listener();self.feedback=self.listener()
        self.spawn(self.accept,self.dashboard,self.command_peer)
        self.spawn(self.accept,self.feedback,self.feedback_peer)
    @staticmethod
    def listener():
        s=socket.socket();s.bind(('127.0.0.1',0));s.listen();s.settimeout(.1);return s
    def spawn(self,f,*args):
        t=threading.Thread(target=f,args=args,daemon=True);self.threads.append(t);t.start()
    def accept(self,s,f):
        while not self.closed.is_set():
            try:c,_=s.accept()
            except socket.timeout:continue
            except OSError:return
            self.spawn(f,c)
    def state(self):
        fraction=(time.monotonic()-self.began)/self.duration if self.duration else 1
        if fraction<1:
            q=[a+(b-a)*fraction for a,b in zip(self.start,self.target)]
            return q,[(b-a)/self.duration for a,b in zip(self.start,self.target)],7
        self.q=list(self.target);return list(self.q),[0.]*6,self.mode
    def command_peer(self,c):
        c.settimeout(1)
        try:
            raw=''
            while ')' not in raw:raw+=c.recv(4096).decode()
            command=raw[:raw.index(')')+1];name=command.split('(',1)[0];payload=''
            with self.lock:
                self.commands.append(command)
                if name=='RequestControl':self.tcp=True;self.writes.append(command)
                elif not self.tcp:c.sendall(b'Control Mode Is Not Tcp\t');return
                elif name=='RobotMode':payload=str(self.state()[2])
                elif name=='ClearError':self.mode=4;self.writes.append(command)
                elif name=='EnableRobot':self.mode=5;self.writes.append(command)
                elif name=='MovJ':
                    self.start=self.state()[0];self.target=[math.radians(float(x)) for x in re.search(r'joint=\{([^}]+)',command)[1].split(',')]
                    self.began=time.monotonic();self.duration=.25;self.id+=1;payload=str(self.id);self.writes.append(command)
                elif name=='GetCurrentCommandID':payload=str(self.id)
                elif name=='Stop':
                    self.target=self.state()[0];self.duration=0;self.mode=5;self.writes.append(command)
                else:raise AssertionError('unexpected command '+command)
                c.sendall(f'0,{{{payload}}},{command};'.encode())
        except (OSError,UnicodeError):pass
        finally:c.close()
    def feedback_peer(self,c):
        stamp=1;last=None
        try:
            while not self.closed.wait(.008):
                with self.lock:
                    if self.duplicates and last:packet=last
                    else:
                        q,v,mode=self.state();stamp+=1;packet=bytearray(1440)
                        struct.pack_into('<H',packet,0,1440);struct.pack_into('<Q',packet,24,mode)
                        struct.pack_into('<Q',packet,32,stamp);struct.pack_into('<Q',packet,48,0x0123456789ABCDEF)
                        struct.pack_into('<d',packet,64,.01)
                        struct.pack_into('<6d',packet,432,*[math.degrees(x) for x in q])
                        struct.pack_into('<6d',packet,480,*[math.degrees(x) for x in v])
                        last=packet
                c.sendall(packet)
        except OSError:pass
        finally:c.close()
    def close(self):
        self.closed.set();self.dashboard.close();self.feedback.close()
        for t in self.threads:t.join(timeout=1)


def main():
    binary,config_file,home_file=map(pathlib.Path,sys.argv[1:])
    home=yaml.safe_load(home_file.read_text())['chess_robot_arm']['ros__parameters']['home']['joint_positions_rad']
    fixture=Fixture(home);events=[]
    try:
        with tempfile.TemporaryDirectory(prefix='arm_tool_protocol_') as directory:
            directory=pathlib.Path(directory);canonical=directory/'home.yaml';canonical.write_bytes(home_file.read_bytes())
            config=yaml.safe_load(config_file.read_text());config['controller'].update(ip='127.0.0.1',dashboard_port=fixture.dashboard.getsockname()[1],feedback_port=fixture.feedback.getsockname()[1])
            config['capture'].update(duration_s=.15,min_frames=5,max_gap_s=.1)
            config['home'].update(arrival_dwell_s=.03,timeout_s=5.,stop_timeout_s=2.)
            config_path=directory/'config.yaml';config_path.write_text(yaml.safe_dump(config))
            def command(operation,*args):return [str(binary),operation,'--config',str(config_path),'--home-file',str(canonical),*map(str,args)]
            def run(operation,*args,success=True):
                p=subprocess.run(command(operation,*args),capture_output=True,text=True,timeout=10)
                assert (p.returncode==0)==success,(p.returncode,p.stdout,p.stderr)
                return p
            run('prepare');assert not fixture.writes
            run('prepare','--request-tcp','--apply');assert fixture.mode==5
            assert [x.split('(')[0] for x in fixture.writes]==['RequestControl','ClearError','EnableRobot']
            events.append('preview zero writes; disabled non-TCP -> RequestControl -> ClearError -> EnableRobot -> measured idle')
            fixture.tcp=False;before=list(fixture.writes);run('prepare','--request-tcp','--apply',success=False);assert fixture.writes==before
            fixture.tcp=True;events.append('enabled non-TCP refuses automatic mode switch/disable')
            before=list(fixture.writes);run('home','--record-feedback',directory/'preview.csv');assert fixture.writes==before
            assert len((directory/'preview.csv').read_text().splitlines())>=2
            with fixture.lock:fixture.target[0]+=.05
            p=run('home','--execute','--record-feedback',directory/'motion.csv');assert 'HOME_REACHED' in p.stdout
            assert len((directory/'motion.csv').read_text().splitlines())>5
            events.append('direct MovJ(joint) Home verifies queue, measured target, speed and dwell without ROS')
            target=yaml.safe_load(canonical.read_text());target['chess_robot_arm']['ros__parameters']['home']['joint_positions_rad'][0]+=.05;canonical.write_text(yaml.safe_dump(target))
            before=len(fixture.writes);p=subprocess.Popen(command('home','--execute'),stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            deadline=time.monotonic()+3
            while len(fixture.writes)==before and time.monotonic()<deadline:time.sleep(.005)
            assert len(fixture.writes)>before
            p.send_signal(signal.SIGINT);stdout,stderr=p.communicate(timeout=5)
            assert p.returncode!=0 and 'STOP_VERIFIED' in stderr,(stdout,stderr)
            assert fixture.writes[-1]=='Stop()';events.append('Ctrl+C after motion requests Stop and confirms stopping')
            before=list(fixture.writes);saved=canonical.read_bytes()
            run('capture-home','--output-dir',directory/'candidate');assert canonical.read_bytes()==saved
            run('capture-home','--output-dir',directory/'applied','--apply')
            assert (directory/'applied/previous_home.yaml').read_bytes()==saved
            assert (directory/'home_reference.json').exists() and fixture.writes==before
            events.append('capture candidate preserves canonical; apply updates only temporary files and backs up old Home; zero robot writes')
            fixture.duplicates=True;saved=canonical.read_bytes()
            run('capture-home','--output-dir',directory/'duplicates','--apply',success=False)
            assert canonical.read_bytes()==saved;events.append('duplicate feedback capture fails without modifying Home')
            print(json.dumps({'status':'PASS','simulation':True,'physical_robot_commands':0,'events':events},indent=2))
    finally:fixture.close()


if __name__=='__main__':main()
