"""HTTP -> C++ panel -> ROS action, isolated domain 95, no hardware drivers."""
import os,time,json,threading,subprocess,signal,socket,uuid,urllib.request,urllib.error,math,copy
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='95';os.environ['ROS_LOCALHOST_ONLY']='1'
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action.server import GoalResponse
from chess_robot_interfaces.action import MoveJoints,MoveGripper
from chess_robot_interfaces.msg import ArmState,GripperState
from sensor_msgs.msg import JointState
RUN=Path(os.environ.get('CHESS_ROBOT_TEST_RUN','/tmp/chess-panel-arm-integration'))
RUN.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('panel_arm_fixture')
q=[.4,.2,.6,-.2,-1.57,.1];goals=[];checks=[]
state=ArmState();state.feedback_fresh=state.reference_valid=state.simulated=True;state.controller_mode=5
grip=GripperState();grip.connected=grip.feedback_valid=grip.commands_enabled=grip.simulated=True
grip.initialization=grip.grip_status=1;grip.position_raw=500
pubs=[n.create_publisher(t,p,10) for t,p in [(ArmState,'/arm/state'),(JointState,'/arm/joint_states'),(GripperState,'/gripper/state')]]
mode={'publish':True,'stamp_age':0,'mismatch':False,'reject':False,'delay':.3,'code':1,'executed':True,'omit_joint':False,'invalid':False}
def publish():
    if not mode['publish']:return
    stamp=n.get_clock().now().to_msg();stamp.sec-=mode['stamp_age']
    state.header.stamp=stamp;state.header.frame_id='base_link'
    j=JointState();j.header=copy.deepcopy(state.header);j.name=['joint'+str(i) for i in range(1,7)];j.position=list(q);j.velocity=[0.]*6
    if mode['mismatch']:j.header.stamp.nanosec=(j.header.stamp.nanosec+123)%1000000000
    if mode['invalid']:j.position[2]=float('nan')
    pubs[0].publish(state)
    if not mode['omit_joint']:pubs[1].publish(j)
    grip.header.stamp=n.get_clock().now().to_msg();pubs[2].publish(grip)
timer=n.create_timer(.04,publish)
def execute(h):
    goals.append(h.request);state.busy=True;time.sleep(mode['delay']);state.busy=False
    r=MoveJoints.Result();r.code=mode['code'];r.executed=mode['executed'];r.detail='simulated measured result'
    if r.code==1:q[:]=h.request.positions;h.succeed()
    else:h.abort()
    return r
server=ActionServer(n,MoveJoints,'/arm/move_joints',execute,goal_callback=lambda g:GoalResponse.REJECT if mode['reject'] else GoalResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
gripper_goals=[]
def gripper_exec(h):
    gripper_goals.append(h.request);r=MoveGripper.Result();r.code=1;r.command_attempted=True;r.simulated=True;h.succeed();return r
gs=ActionServer(n,MoveGripper,'/gripper/command',gripper_exec,callback_group=ReentrantCallbackGroup())
executor=MultiThreadedExecutor(num_threads=4);executor.add_node(n);thread=threading.Thread(target=executor.spin);thread.start()
with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
log=(RUN/'panel-arm-test.log').open('w')
def launch(port):
    return subprocess.Popen([os.environ.get('PANEL_EXECUTABLE',str(RUN/'install/chess_robot_panel/lib/chess_robot_panel/panel_node')),'--ros-args','-p',f'port:={port}','-p','enable_control:=true','-p','enable_arm_control:=true','-p','feedback_max_age_sec:=0.3','-p','arm_timeout_sec:=0.5'],stdout=log,stderr=log)
proc=launch(port)
base=f'http://127.0.0.1:{port}'
def request(path='/api/state',body=None,extra=None,raw=None):
    headers={'Content-Type':'application/json','X-Panel-Request':'2'};headers.update(extra or {})
    req=urllib.request.Request(base+path,data=raw if raw is not None else None if body is None else json.dumps(body).encode(),headers=headers)
    try:
        with urllib.request.urlopen(req,timeout=2) as r:return r.status,json.loads(r.read()) if path.startswith('/api') else r.read().decode()
    except urllib.error.HTTPError as e:return e.code,json.loads(e.read())
def snapshot():return request()[1]
def arm():return snapshot()['arm']
def can():return arm()['can_move']
def wait(pred,timeout=8):
    until=time.monotonic()+timeout
    while time.monotonic()<until:
        try:
            if pred():return
        except (OSError,KeyError):pass
        time.sleep(.05)
    raise AssertionError('condition timeout; last panel state='+json.dumps(snapshot()))
def payload(index=2,target=None):return dict(request_id=uuid.uuid4().hex,server_id=snapshot()['server_id'],joint_index=index,target_deg=math.degrees(q[index])+.5 if target is None else target)
try:
    wait(can);assert not goals
    request('/');snapshot();assert not goals;checks.append('startup and all GETs publish zero actions')
    start=list(q);body=payload();assert request('/api/arm/joint',body)[0]==202
    assert request('/api/arm/joint',body)[0]==200
    assert request('/api/arm/joint',dict(body,target_deg=body['target_deg']+1))[0]==409
    assert request('/api/arm/joint',dict(body,target_deg=body['target_deg']+.00001))[0]==409
    assert request('/api/gripper/move',dict(request_id=uuid.uuid4().hex,server_id=body['server_id'],position_raw=450,force_percent=20))[0]==409
    wait(lambda:len(goals)==1);wait(can)
    g=goals[0];assert list(g.expected_start)==start and g.header.frame_id=='base_link' and not g.plan_only
    assert abs(g.positions[2]-math.radians(body['target_deg']))<1e-12
    assert all(g.positions[i]==start[i] for i in range(6) if i!=2)
    checks.append('one selected joint converted degrees->radians; exact baseline/stamp; duplicate suppression and arm/gripper mutual exclusion')
    count=len(goals)
    for bad in [dict(payload(),joint_index=6),dict(payload(),target_deg=''),dict(payload(),target_deg='NaN'),dict(payload(),target_deg=math.degrees(q[2])+11),dict(payload(),unknown=1)]:
        assert request('/api/arm/joint',bad)[0]==400,bad
    b=payload();raw=(json.dumps(b)[:-1]+',"joint_index":2}').encode();assert request('/api/arm/joint',raw=raw)[0]==400
    assert len(goals)==count;checks.append('bad index, empty, nonfinite, >10deg, duplicate/unknown fields rejected without action')
    assert request('/api/arm/joint',payload(),{'Origin':'http://evil.invalid'})[0]==403
    assert request('/api/arm/joint',payload(),{'X-Panel-Request':'1'})[0]==403
    assert request('/api/arm/joint',dict(payload(),server_id='old-service'))[0]==409
    checks.append('same-origin, protocol version and server epoch checks enforced')
    for key,value in [('stamp_age',5),('mismatch',True),('invalid',True),('omit_joint',True)]:
        mode[key]=value;time.sleep(1.2 if key=='omit_joint' else .5)
        wait(lambda:not can(),timeout=2)
        assert request('/api/arm/joint',payload())[0]==409
        mode[key]=0 if key=='stamp_age' else False;wait(can)
    state.reference_valid=False;wait(lambda:not can());assert request('/api/arm/joint',payload())[0]==409;state.reference_valid=True;wait(can)
    checks.append('old, mismatched, invalid, missing joint feedback and invalid reference fail closed')
    mode['reject']=True;assert request('/api/arm/joint',payload())[0]==202;wait(lambda:snapshot()['arm_command']['phase']=='rejected');mode['reject']=False;wait(can)
    checks.append('action rejection is displayed and never retried')
    mode['code']=10;mode['executed']=False;assert request('/api/arm/joint',payload())[0]==202
    wait(lambda:snapshot()['arm_command']['phase']=='failed');wait(can)
    checks.append('definitive non-executed invalid goal is retryable after manual correction')
    mode['executed']=True
    mode['code']=16;assert request('/api/arm/joint',payload())[0]==202
    wait(lambda:snapshot()['arm_command']['phase']=='unknown');assert not can()
    assert request('/api/arm/joint',payload())[0]==409
    checks.append('ambiguous executed failure stays locked with zero automatic resends')
    proc.send_signal(signal.SIGINT);proc.wait(timeout=6)
    with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
    base=f'http://127.0.0.1:{port}';mode['code']=1;mode['delay']=4.2;proc=launch(port)
    wait(can);assert request('/api/arm/joint',payload())[0]==202
    wait(lambda:snapshot()['arm_command']['phase']=='unknown',timeout=6)
    time.sleep(1.2);assert snapshot()['arm_command']['phase']=='unknown' and not can()
    assert request('/api/arm/joint',payload())[0]==409
    checks.append('late success after deadline cannot unlock the unknown-result latch')
    assert not gripper_goals
    (RUN/'panel-arm-result.json').write_text(json.dumps({'passed':True,'checks':checks,'real_hardware_commands':0,'joint_actions':len(goals)},indent=2));print(json.dumps(checks))
finally:
    proc.send_signal(signal.SIGINT)
    try:proc.wait(timeout=6)
    except subprocess.TimeoutExpired:proc.kill();proc.wait()
    log.close();executor.shutdown();thread.join();n.destroy_node();rclpy.shutdown()
