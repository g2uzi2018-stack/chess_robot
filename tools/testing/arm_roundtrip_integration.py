#!/usr/bin/env python3
"""Business-node round-trip regression, IPv4 loopback + isolated ROS domain 98."""
import argparse
import json
import os
import pathlib
import time
from types import SimpleNamespace
from arm_controller_fixture import ControllerFixture
from arm_roundtrip import RosSession,Runner,ExecutionFault,load_home,manifest,write_json
from arm_roundtrip_report import render


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root',type=pathlib.Path,default=pathlib.Path(__file__).resolve().parents[2])
    parser.add_argument('--out',type=pathlib.Path,required=True)
    parser.add_argument("--command-policy",type=pathlib.Path)
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='98' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    root=args.project_root.resolve();out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    home=load_home(root);data=manifest(home)
    position=home['tcp_m']
    # Targets are exact TCP endpoints, not grasp coordinates. Both have nonzero
    # displacement, and one deliberately violates the configured workspace.
    data['coarse_points']=[{'id':'right_15mm','tcp_m':[position[0]+.015,position[1],position[2]],'kind':'coarse'},
                          {'id':'workspace_reject','tcp_m':[3.,position[1],position[2]],'kind':'coarse'},
                          {'id':'up_10mm','tcp_m':[position[0],position[1],position[2]+.010],'kind':'coarse'}]
    data['rays']={};data['max_refinements_per_ray']=0;data['maximum_targets']=3
    fixture=ControllerFixture(root,out/'fixture')
    p=SimpleNamespace(fixture=True,execute=True,controller_ip='127.0.0.1',
       fixture_endpoints=out/'fixture/endpoints.json',history_audit=[],history_attestation=None,
       speed_evidence=None,goal_timeout=45,margin_evaluator=None,recover_home=True,command_policy=args.command_policy)
    session=RosSession.__new__(RosSession)
    events=[]
    try:
        run=out/'normal';run.mkdir()
        session.__init__(root,run,data,p)
        from chess_robot_interfaces.srv import InitializeReference
        initialize=session.node.create_client(InitializeReference,'/simulation/arm/initialize_reference')
        session.wait(lambda:initialize.service_is_ready(),5)
        until=time.monotonic()+.4;session.wait(lambda:time.monotonic()>=until,1)
        response=session.future(initialize.call_async(InitializeReference.Request()),5)
        assert response.success,response.detail
        session.wait(lambda:session.snapshot()['reference_valid'],3)
        # Explicitly simulated off-Home starting condition after the fixture's
        # single reference initialization. Recovery must preserve that history.
        with fixture.lock:
            fixture.target_q=fixture.target_q.copy();fixture.target_q[0]+=.03
            fixture.start_q=fixture.target_q.copy();fixture.duration=0
        session.wait(lambda:session.snapshot()['joints_rad'][0]>home['joints_rad'][0]+.02,3)
        session.needs_qualification=True # simulated qualification uses same real business-node action chain
        result=Runner(data,session,run,execute=True).run()
        result['simulation']=True
        write_json(run/'results.json',result)
        assert [x['status'] for x in result['targets']]==['reused_success','planning_rejected','roundtrip_success']
        assert all(x['outbound']['position_error_m']<.003 and x['return_home']['position_error_m']<.003
                   for x in result['targets'] if x['status'] in ['roundtrip_success','reused_success'])
        evidence=json.loads((run/'speed_qualification.json').read_text())
        assert evidence['simulation'] is True and evidence['validated_profiles']==['low','high']
        assert len(result['qualification_trials'])==2
        assert result['recovery_home']['status']=='MEASURED_HOME_REACHED'
        events += ['two nonzero exact-TCP targets each complete outbound + formal Home via existing action',
                   'workspace planning rejection is recorded and next target continues',
                   'predicted target-to-Home precheck marker required before all outbound actuation',
                   'independent fresh endpoint position/orientation and Home joint checks pass',
                   'fixed orientation, zero clearance, no gripper/descend/retreat API requests']
        events.append('low/high qualification uses same action chain; evidence binds context and remains simulated=true')
        events.append('explicit off-Home recovery is planned low HOME and measured before qualification; no reference reset')
        # A successful ACK with frozen encoders must fail. No Home is sent after
        # the outbound failure; stop verification only observes fresh idle data.
        failure=out/'frozen';failure.mkdir()
        frozen=dict(data,coarse_points=[{'id':'frozen_20mm','tcp_m':[position[0],position[1],position[2]+.020],'kind':'coarse'}],maximum_targets=1)
        fixture.controls['ignore_servo']=True
        before=len(session.frames)
        runner=Runner(frozen,session,failure,execute=True)
        try:
            runner.run()
            raise AssertionError('frozen encoders incorrectly reported success')
        except ExecutionFault:
            pass
        assert runner.report['status']=='EXECUTION_FAULT_STOPPED',runner.report
        assert runner.report['targets'][0]['return_home'] is None
        assert runner.report['targets'][0]['outbound']['code']!=1
        assert not any(x['plan_only'] is False and x.get('final_tcp_m')==home['tcp_m']
                       and x['code']==1 for x in session.frames[before:])
        runner.report['simulation']=True
        write_json(failure/'results.json',runner.report)
        events.append('frozen-encoder successful ACKs fail outbound; Stop verified; no forced Home return/replay')
        render(run);render(failure)
        summary={'status':'PASS','simulation':True,'physical_motion_commands':0,'events':events,
                 'roundtrip_successes':2,'planning_rejections':1,'execution_failures':1}
        write_json(out/'result.json',summary)
        print(json.dumps(summary,indent=2))
    finally:
        session.close();fixture.close()


if __name__=='__main__':main()
