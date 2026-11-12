#!/usr/bin/env python3
"""Installed demo startup: trusted arm reference -> gripper initialization -> Home."""
import argparse
import json
import signal
import sys
import time
from pathlib import Path

import rclpy
import yaml
from chess_robot_interfaces.srv import RestoreCheckpoint
from ros_io import RealIO,GetParameters,MoveGripper,convert,position,stamp,write


def stage(io, name, **details):
    io.report['startup_stage']=name
    print(json.dumps(dict(stage=name,**details),ensure_ascii=False),flush=True)


def wait_stationary_for_recovery(io):
    if not io.params.wait_for_service(timeout_sec=15):
        raise RuntimeError('机械臂参数服务未启动')
    request=GetParameters.Request()
    request.names=['checkpoint.stopped_dwell_s','tolerances.stationary_rad_s']
    values=io.wait(io.params.call_async(request),5).values
    dwell,speed=[value.double_value for value in values]
    deadline=time.monotonic()+15
    since=previous=anchor=None
    while time.monotonic()<deadline:
        io.spin()
        if not (io.arm.keys() & io.joint.keys()):
            continue
        state,joint=io.pair()
        tick=stamp(state)
        if previous is not None and tick<=previous[0]:
            continue
        good=(state.feedback_fresh and not state.simulated and not state.busy
              and state.controller_mode in (4,5) and 0<=(io.now()-tick)*1e-9<=.2
              and max(map(abs,joint.velocity))<=speed)
        if previous is not None:
            dt=(tick-previous[0])*1e-9
            good=good and 0<dt<=.2 and max(abs(a-b) for a,b in zip(joint.position,previous[1]))/dt<=speed
        if anchor is not None:
            good=good and max(abs(a-b) for a,b in zip(joint.position,anchor))<=.001
        if not good:
            since=anchor=None
        elif since is None:
            since=tick;anchor=list(joint.position)
        previous=tick,list(joint.position)
        if since is not None and (tick-since)*1e-9>=dwell+.2:
            return state
    raise RuntimeError('机械臂未取得新鲜连续停稳反馈；未恢复、未初始化')


def restore_arm_reference(io, attested):
    stage(io,'WAIT_ARM_STATIONARY')
    state=wait_stationary_for_recovery(io)
    if state.reference_valid and state.controller_mode==5:
        stage(io,'REFERENCE_ALREADY_VALID')
        return
    stage(io,'RESTORE_ARM_REFERENCE')
    if not attested:
        raise RuntimeError('启动恢复需要确认停机期间没有手动拖动、碰撞或其他未观测运动；确认后在启动命令末尾加 --no-unobserved-motion')
    client=io.n.create_client(RestoreCheckpoint,'/arm/restore_checkpoint')
    try:
        if not client.wait_for_service(timeout_sec=15):
            raise RuntimeError('机械臂检查点恢复服务未启动')
        request=RestoreCheckpoint.Request();request.no_unobserved_motion=True
        reply=io.wait(client.call_async(request),15)
        io.report['reference_restore']=dict(success=reply.success,detail=reply.detail)
        if not reply.success:
            raise RuntimeError('可信参考恢复失败：'+reply.detail+'。未初始化夹爪、未启动业务；需要完整历史恢复或正式现场参考重建，重复启动不能修复此记录')
        enabled=wait_stationary_for_recovery(io)
        if enabled.controller_mode!=5:
            raise RuntimeError('可信参考已恢复，但机械臂仍禁用；使能结果：'+reply.detail)
        stage(io,'REFERENCE_RESTORED',detail=reply.detail)
    finally:
        io.n.destroy_client(client)


def fresh_gripper(io):
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        io.spin()
        g=io.gripper
        if (g is not None and not g.simulated and g.connected and g.feedback_valid
                and g.commands_enabled and not g.busy
                and 0<=g.feedback_age_sec and 0<=(io.now()-stamp(g))*1e-9<=.2
                and g.feedback_age_sec+(io.now()-stamp(g))*1e-9<=io.gripper_freshness):
            return g
    raise RuntimeError('夹爪没有新鲜、有效、空闲反馈；未发送初始化命令')


def initialize_gripper(io):
    stage(io,'CHECK_GRIPPER')
    g=fresh_gripper(io)
    if g.object_detected or g.object_dropped:
        raise RuntimeError('夹爪持物或报告掉落，需要检查现场；不会自动张爪或重新初始化')
    if g.initialization==1:
        stage(io,'GRIPPER_ALREADY_INITIALIZED')
        return
    if g.initialization!=0:
        raise RuntimeError('夹爪初始化状态不允许新请求：'+str(g.initialization))
    # 用户启动声明包含夹爪无物体、手已移开。只发送一次初始化，不自动重试。
    stage(io,'INITIALIZE_GRIPPER')
    if not io.grip_client.wait_for_server(timeout_sec=5):
        raise RuntimeError('夹爪 Action 服务未启动')
    goal=MoveGripper.Goal()
    goal.operation=goal.INITIALIZE;goal.position_raw=1000
    goal.force_percent=io.demo['force_percent'];goal.require_contact=False;goal.timeout_sec=15.
    io.grip_pending=io.grip_client.send_goal_async(goal)
    io.grip_goal=io.wait(io.grip_pending,8);io.grip_pending=None
    if not io.grip_goal.accepted:
        raise RuntimeError('夹爪拒绝初始化请求；未重试')
    io.grip_terminal=io.grip_goal.get_result_async()
    reply=io.wait(io.grip_terminal,20,monitor=True)
    io.report['gripper_actions'].append(dict(operation='INITIALIZE',status=reply.status,result=convert(reply.result)))
    result=reply.result;g=result.final_state
    if not (reply.status==4 and result.code==0 and not result.simulated
            and not g.simulated and g.connected and g.feedback_valid and g.commands_enabled
            and g.initialization==1 and not g.busy
            and not g.object_detected and not g.object_dropped):
        raise RuntimeError('夹爪初始化未得到有效完成反馈：'+result.detail+'；不会自动重试')
    io.receive_gripper(g)
    io.grip_goal=io.grip_terminal=None
    stage(io,'GRIPPER_INITIALIZED')


def prepare_devices(io, attested):
    restore_arm_reference(io,attested)
    stage(io,'VERIFY_ARM_READY')
    io.stopped()
    io.check_parameters()
    io.read_gripper_policy()
    initialize_gripper(io)
    io.startup_precheck(io.demo)
    if io.gripper_state(idle=True).object_detected:
        raise RuntimeError('夹爪仍检测到物体；不执行新业务')
    if io.gripper.position_raw<995:
        stage(io,'OPEN_EMPTY_GRIPPER')
        io.open_empty()
    _,joint,_=io.stopped()
    if max(abs(a-b) for a,b in zip(joint.position,io.home_joints))>io.demo['home_joint_tolerance_rad']:
        stage(io,'RETURN_HOME')
        io.home()
    stage(io,'DEVICES_READY_HOME_EMPTY')


def verify_speeds(io):
    config=yaml.safe_load(Path('config/arm/demo-speed.yaml').read_text())['chess_robot_arm']['ros__parameters']
    profiles=config['profiles'];servo=config['servo']
    expected={'profiles.roundtrip_low.speed_rad_s':profiles['roundtrip_low']['speed_rad_s'],
              'profiles.table_contact.speed_rad_s':profiles['table_contact']['speed_rad_s'],
              'profiles.roundtrip_low.acceleration_rad_s2':profiles['roundtrip_low']['acceleration_rad_s2'],
              'profiles.table_contact.acceleration_rad_s2':profiles['table_contact']['acceleration_rad_s2'],
              'servo.ack_timeout_s':servo['ack_timeout_s'],'servo.max_pending':servo['max_pending']}
    request=GetParameters.Request();request.names=list(expected)
    values=io.wait(io.params.call_async(request),5).values
    actual={name:(value.integer_value if name=='servo.max_pending' else value.double_value)
            for name,value in zip(request.names,values)}
    if any(abs(actual[name]-limit)>1e-9 for name,limit in expected.items()):
        raise RuntimeError('演示速度或 ServoJ 回执配置与活节点不同；运行 activate 加载配置后再启动')
    io.report['profiles']={k:v for k,v in actual.items() if k.startswith('profiles.')}
    io.report['servo']={k:v for k,v in actual.items() if k.startswith('servo.')}
    return io.report['profiles']


def wait_checkpoint_saved(io):
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        io.spin()
        state,_=io.pair()
        if (state.reference_valid and state.feedback_fresh and not state.busy
                and 'durable stopped save ready' in state.diagnostic):
            return
    raise RuntimeError('尚未确认持久化停稳存档；取消重启，保留当前有效参考')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=('prepare','before-restart','vertical'))
    parser.add_argument('--no-unobserved-motion',action='store_true',
                        help='确认停机或断电期间没有手动拖动、碰撞等未观测运动')
    args=parser.parse_args()
    config=json.loads(Path('tools/pick_stack/demo.row.live.json').read_text())
    out=Path('runs')/('demo_startup_'+str(time.time_ns()));out.mkdir()
    rclpy.init();io=RealIO(config,out,'User launched automatic demo preparation; empty gripper and hands clear')
    signal.signal(signal.SIGINT,lambda *_:setattr(io,'abort',True))
    signal.signal(signal.SIGTERM,lambda *_:setattr(io,'abort',True))
    code=0
    try:
        prepare_devices(io,args.no_unobserved_motion)
        if args.mode=='before-restart':
            wait_checkpoint_saved(io)
            stage(io,'READY_FOR_CONTROLLED_RESTART')
        else:
            profiles=verify_speeds(io)
            print(json.dumps(dict(status='READY_HOME_EMPTY',profiles=profiles,servo=io.report['servo']),ensure_ascii=False),flush=True)
        if args.mode=='vertical':
            state,_=io.pair();start=position(state).tolist();target=[start[0],start[1],.14]
            for operation,point in (('DESCEND',target),('RETREAT',start)):
                begin=time.monotonic()
                io.arm_request(operation,point,True);io.arm_request(operation,point,False)
                print(operation+' seconds: '+str(round(time.monotonic()-begin,3)),flush=True)
            io.home()
        io.report['status']='READY' if args.mode!='vertical' else 'EMPTY_VERTICAL_RETURNED_HOME'
    except Exception as error:
        code=2
        io.report.update(status='STARTUP_BLOCKED',error=str(error))
        if any(value is not None for value in (io.goal,io.pending,io.grip_goal,io.grip_pending)):
            io.halt('unknown')
        print(json.dumps(dict(status='STARTUP_BLOCKED',stage=io.report.get('startup_stage'),
                              reason=str(error),report=str(out/'startup-result.json')),ensure_ascii=False),flush=True)
    finally:
        io.report.update(arm_latest=convert(io.arm[max(io.arm)]) if io.arm else None,
                         gripper_latest=convert(io.gripper) if io.gripper else None,
                         no_unobserved_motion_attested=args.no_unobserved_motion)
        write(out/'startup-result.json',io.report)
        io.close();rclpy.shutdown()
    return code


if __name__=='__main__':
    raise SystemExit(main())
