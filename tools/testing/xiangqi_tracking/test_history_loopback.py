"""Bounded ROS loopback regressions. Never connects outside 127.0.0.1."""
import json,signal,socket,struct,subprocess,threading,time,os,argparse
from pathlib import Path
import yaml,rclpy
from chess_robot_interfaces.msg import ArmState
from rosidl_runtime_py.convert import message_to_ordereddict as convert
p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--label',default='history-loopback');p.add_argument('--cases',nargs='*');a=p.parse_args()
r=Path(os.environ.get('CHESS_ROBOT_ROOT','/home/kuang/workspace/chess_robot'));w=Path(a.run).resolve();out=w/a.label;out.mkdir(exist_ok=False)
assert os.environ.get('ROS_DOMAIN_ID')=='118' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
rclpy.init();node=rclpy.create_node('isolated_checkpoint_lifecycle_regression');latest=[]
sub=node.create_subscription(ArmState,'/simulation/arm/state',lambda m:latest.append(m),50);results=[]
def run_case(name):
 folder=out/name;folder.mkdir();stop=threading.Event();pause=threading.Event();calls=[];mode=[5 if name=='pure-mode5' else 4];errors=[]
 ds=socket.socket();ds.bind(('127.0.0.1',0));ds.listen();fs=socket.socket();fs.bind(('127.0.0.1',0));fs.listen();dp,fp=ds.getsockname()[1],fs.getsockname()[1]
 fixture=json.loads(subprocess.check_output([str(w/'build/chess_robot_arm/checkpoint_fixture'),str(r/'src/chess_robot_description/urdf/nova5_with_ag95.urdf'),str(folder/'state.save'),str(dp),str(fp),'dirty' if name=='dirty' else 'clean-nonzero'],text=True))
 original=(folder/'state.save').read_bytes();(folder/'original.save').write_bytes(original)
 if name=='checksum':
  with (folder/'state.save').open('ab') as f:f.write(b'bad checksum')
 q=list(fixture['q_deg'])
 if name=='endpoint':q[0]+=.006*180/3.141592653589793
 def feedback_server():
  fs.settimeout(.1)
  while not stop.is_set():
   try:c,_=fs.accept()
   except socket.timeout:continue
   try:
    with c:
     while not stop.is_set():
      if pause.is_set():stop.wait(.002);continue
      b=bytearray(1440);struct.pack_into('<H',b,0,1440);struct.pack_into('<Q',b,24,mode[0]);struct.pack_into('<Q',b,32,time.time_ns()//1000000);struct.pack_into('<Q',b,48,0x0123456789ABCDEF);struct.pack_into('<6d',b,432,*q);c.sendall(b);stop.wait(.008)
   except (BrokenPipeError,ConnectionResetError):continue
   except Exception as e:errors.append(str(e));return
 def dashboard_server():
  try:
   ds.settimeout(.1)
   while not stop.is_set():
    try:c,_=ds.accept();break
    except socket.timeout:continue
   else:return
   with c:
    c.settimeout(.1);buffer=b''
    while not stop.is_set():
     try:data=c.recv(4096)
     except socket.timeout:continue
     if not data:break
     buffer+=data
     while b')' in buffer:
      raw,buffer=buffer.split(b')',1);cmd=(raw+b')').decode();calls.append(cmd)
      assert cmd in ('GetErrorID()','EnableRobot()'),('unexpected mock command',cmd)
      if cmd=='GetErrorID()':
       if name=='reset-alarm':
        c.setsockopt(socket.SOL_SOCKET,socket.SO_LINGER,struct.pack('ii',1,0));return
       if name=='loss-alarm':pause.set();stop.wait(.35)
       c.sendall(b'0,{[]},GetErrorID();');pause.clear()
      else:
       if name=='reset-enable':
        c.setsockopt(socket.SOL_SOCKET,socket.SO_LINGER,struct.pack('ii',1,0));return
       if name=='timeout-enable':stop.wait(.3);return
       if name!='ack-no-mode5':mode[0]=5
       if name=='loss-ack':pause.set();stop.wait(.35)
       c.sendall(b'0,{},EnableRobot();');pause.clear()
  except (BrokenPipeError,ConnectionResetError):pass
  except Exception as e:errors.append(str(e))
  finally:pause.clear()
 threads=[threading.Thread(target=feedback_server),threading.Thread(target=dashboard_server)]
 for t in threads:t.start()
 auto_enable=name not in ('pure-mode4','pure-mode5','dirty','checksum','identity','endpoint')
 params={'chess_robot_arm':{'ros__parameters':{'backend':'hardware','test_fixture':True,'motion.paused':False,'urdf_file':str(r/'src/chess_robot_description/urdf/nova5_with_ag95.urdf'),'controller.ip':'127.0.0.1','controller.dashboard_port':dp,'controller.feedback_port':fp,'controller.timeout_ms':100,'controller.protocol_family':'v4','controller.firmware_version':'wrong-identity' if name=='identity' else 'checkpoint-loopback-fixture','tcp.translation_m':[0.,0.,.2],'tcp.rpy_rad':[1.5707963267948966,-1.5707963267948966,0.],'checkpoint.file':str(folder/'state.save'),'checkpoint.auto_restore':True,'checkpoint.no_unobserved_motion':True,'checkpoint.enable_after_restore':auto_enable,'checkpoint.period_s':.25,'checkpoint.stopped_dwell_s':.04,'tolerances.dwell_s':.02,'feedback.freshness_s':.2,'hardware.enable':True,'hardware.tcp_verified':True,'hardware.limits_verified':True,'hardware.execution_profile_verified':True,'hardware.servoj_certified':True}}}
 (folder/'params.yaml').write_text(yaml.safe_dump(params));latest.clear();log=(folder/'node.log').open('w');child=subprocess.Popen([str(w/'install/chess_robot_arm/lib/chess_robot_arm/arm_node'),'--ros-args','--params-file',str(folder/'params.yaml')],stdout=log,stderr=subprocess.STDOUT)
 try:
  end=time.monotonic()+9;passed=False
  while time.monotonic()<end:
   rclpy.spin_once(node,timeout_sec=.02);assert child.poll() is None,'node exited'
   if not latest:continue
   state=latest[-1];d=state.diagnostic
   if name in ('pure-mode4','pure-mode5','healthy'):passed=state.reference_valid and 'durable stopped save ready' in d
   elif name in ('reset-alarm','reset-enable','timeout-enable'):passed=state.reference_valid and 'history retained by independent fresh stopped endpoint' in d and 'durable stopped save ready' in d
   elif name=='ack-no-mode5':passed=state.reference_valid and 'enable confirmation timed out' in d
   elif name in ('loss-alarm','loss-ack'):passed=not state.reference_valid and state.feedback_fresh and 'subsequent feedback confirmation invalidated' in d
   elif name=='dirty':passed=not state.reference_valid and 'checkpoint is dirty' in d
   elif name=='checksum':passed=not state.reference_valid and 'checksum mismatch' in d
   elif name=='identity':passed=not state.reference_valid and 'schema/model/controller mismatch' in d
   elif name=='endpoint':passed=not state.reference_valid and 'live joints do not match' in d
   if passed:break
  assert passed,(name,convert(latest[-1]) if latest else 'no state')
  # Observe an extra interval: no repeated actuator calls after success/failure.
  end=time.monotonic()+.35
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
  state=latest[-1]
  expected=[] if not auto_enable else ['GetErrorID()'] if name in ('reset-alarm','loss-alarm') else ['GetErrorID()','EnableRobot()']
  assert calls==expected,(name,calls);assert not errors,errors
  if name not in ('dirty','checksum','identity','endpoint','loss-alarm','loss-ack'):
   assert state.reference_valid and state.total_twist_rad>.1,(name,'nonzero winding lost',convert(state))
  restarted=False
  if name in ('pure-mode4','pure-mode5'):
   before_total=state.total_twist_rad;before_signed=state.signed_twist_rad
   child.send_signal(signal.SIGINT);child.wait(timeout=8);latest.clear()
   child=subprocess.Popen([str(w/'install/chess_robot_arm/lib/chess_robot_arm/arm_node'),'--ros-args','--params-file',str(folder/'params.yaml')],stdout=log,stderr=subprocess.STDOUT)
   deadline=time.monotonic()+5
   while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=.02);assert child.poll() is None
    if latest and latest[-1].reference_valid and 'durable stopped save ready' in latest[-1].diagnostic:break
   state=latest[-1];assert state.reference_valid and abs(state.total_twist_rad-before_total)<1e-12 and abs(state.signed_twist_rad-before_signed)<1e-12
   assert calls==[];restarted=True
  results.append(dict(case=name,passed=True,calls=list(calls),same_checkpoint_node_restart_tested=restarted,final_state=convert(state),physical_commands=0))
  print(json.dumps(dict(case=name,passed=True,calls=calls,reference=state.reference_valid,mode=state.controller_mode)),flush=True)
 finally:
  if child.poll() is None:child.send_signal(signal.SIGINT);child.wait(timeout=8)
  log.close();stop.set();pause.clear()
  for t in threads:t.join(timeout=2)
  ds.close();fs.close()
  for _ in range(5):rclpy.spin_once(node,timeout_sec=.01)
try:
 cases=('pure-mode4','pure-mode5','healthy','reset-alarm','reset-enable','timeout-enable','ack-no-mode5','loss-alarm','loss-ack','dirty','checksum','identity','endpoint')
 selected=a.cases or cases
 assert all(name in cases for name in selected)
 for name in selected:run_case(name)
 (out/'results.json').write_text(json.dumps(dict(passed=True,cases=results,physical_controller_connections=0),indent=2))
finally:node.destroy_node();rclpy.shutdown()
