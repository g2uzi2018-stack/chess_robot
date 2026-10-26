#!/usr/bin/env python3
"""Isolated simulated power-cycle history restore -> node Enable -> Home test.

No real controller connection, no InitializeReference/ClearError service, no
direct motion transport. All movement remains the business-node action chain.
"""
import argparse
import json
import math
import os
import pathlib
import struct
import time
from types import SimpleNamespace
from arm_controller_fixture import ControllerFixture
from arm_roundtrip import RosSession,Runner,load_home,manifest,sha,write_json


class PowerCycleFixture(ControllerFixture):
    def __init__(self,root,out,endpoint):
        super().__init__(root,out)
        self.enable_count=0
        with self.lock:
            self.target_q=self.q.copy();self.target_q[5]+=.0024
            self.start_q=self.target_q.copy();self.q=self.target_q.copy();self.duration=0
            self.controls['mode']=4

    def command(self,command):
        if command.lower()=='enablerobot()':
            with self.lock:
                self.commands.append(command);self.enable_count+=1
                if self.controls['mode']!=4:return '-1,{},'+command+';'
            # Keep mode4 feedback flowing while the enable ACK exceeds the
            # ordinary 500ms FK/query budget. Node has a separate enable budget.
            self.closed.wait(.7)
            with self.lock:self.controls['mode']=5
            return '0,{},'+command+';'
        return super().command(command)


def history(path,home):
    import numpy as np
    base_host=time.time_ns()-5_000_000_000
    old_clock=int(time.time()*1000)+3_600_000 # deliberately later than the new live epoch
    with path.open('wb') as file:
        for i in range(12):
            q=list(home['joints_rad'])
            q[5]+=.0002*i if i<10 else .0022+.0002*(i-10)
            frame=bytearray(1440)
            struct.pack_into('<H',frame,0,1440)
            struct.pack_into('<Q',frame,24,5 if i<10 else 9)
            struct.pack_into('<Q',frame,32,old_clock+8*i)
            struct.pack_into('<Q',frame,48,0x0123456789ABCDEF)
            struct.pack_into('<d',frame,64,1.)
            struct.pack_into('<6d',frame,192,*[x*180/math.pi for x in q])
            struct.pack_into('<6d',frame,432,*[x*180/math.pi for x in q])
            file.write(struct.pack('<q',base_host+8_000_000*i));file.write(frame)
    return old_clock+88


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root',type=pathlib.Path,default=pathlib.Path(__file__).resolve().parents[2])
    parser.add_argument('--out',type=pathlib.Path,required=True)
    parser.add_argument('--command-policy',type=pathlib.Path)
    args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='98' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    root=args.project_root.resolve();out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    home=load_home(root);data=manifest(home)
    point={'id':'up_10mm','kind':'coarse','tcp_m':[home['tcp_m'][0],home['tcp_m'][1],home['tcp_m'][2]+.010]}
    data.update(coarse_points=[point],rays={},max_refinements_per_ray=0,maximum_targets=1)
    raw=out/'old_epoch_feedback.bin';old_last_clock=history(raw,home)
    prior=out/'prior_simulated_fault.json'
    write_json(prior,{'status':'EXECUTION_FAULT_STOP_UNCONFIRMED','simulation':True,'fixture_only':True})
    power=out/'power_fixture_evidence.json'
    write_json(power,{'human_statement':'SIMULATED FIXTURE: power cycle only; no manual joint/cable motion',
       'only_power_cycle_no_manual_joint_or_cable_motion':True,
       'prior_fault_result':{'path':str(prior),'sha256':sha(prior)},'simulation':True})
    fixture=PowerCycleFixture(root,out/'fixture',home['joints_rad'])
    common=dict(fixture=True,execute=True,controller_ip='127.0.0.1',
        fixture_endpoints=out/'fixture/endpoints.json',history_audit=[raw],history_attestation=None,
        speed_evidence=None,goal_timeout=45,margin_evaluator=None,recover_home=True,
        command_policy=args.command_policy,continue_results=None,reuse_results=None,controller_limits=None)
    events=[]
    disabled=RosSession.__new__(RosSession)
    try:
        run=out/'default_disabled';run.mkdir()
        try:
            disabled.__init__(root,run,data,SimpleNamespace(**common,power_cycle_evidence=None))
            raise AssertionError('default path incorrectly accepted a mode9 fault-tail handoff')
        except ValueError as error:
            assert 'restore rejected' in str(error),str(error)
        assert fixture.enable_count==0
        events.append('default path rejects terminal mode9 history and sends no EnableRobot')
    finally:disabled.close()
    session=RosSession.__new__(RosSession)
    try:
        run=out/'explicit_power';run.mkdir()
        session.__init__(root,run,data,SimpleNamespace(**common,power_cycle_evidence=power))
        restored=session.snapshot()
        write_json(run/'restored_before_any_motion.json',restored)
        assert restored['mode']==5 and restored['reference_valid'] and restored['simulated']
        assert abs(restored['net_twist_rad'])>1e-5 and restored['total_twist_rad']>1e-5
        assert session.restore_count==1 and fixture.enable_count==1
        assert not any(c.lower().startswith('servoj(') for c in fixture.commands)
        # Existing raw historical controller clock is never rewritten to match
        # the lower new epoch. It is only explicitly attested by the parent node.
        assert struct.unpack_from('<Q',raw.read_bytes(),11*1448+8+32)[0]==old_last_clock
        events.append('explicit power evidence restores nonzero winding at disabled q without zero/reference init')
        events.append('only business-node empty GetErrorID check then one EnableRobot changes mode4 to mode5')
        session.needs_qualification=True
        runner=Runner(data,session,run,True)
        runner.report['simulation']=True
        report=runner.run()
        assert report['recovery_home']['status']=='MEASURED_HOME_REACHED'
        assert len(report['qualification_trials'])==2
        assert report['reference_initializations']==0 and report['reference_resets']==0
        assert report['targets'][0]['status']=='reused_success'
        assert not any('clearerror' in c.lower() for c in fixture.commands)
        first_enable=next(i for i,c in enumerate(fixture.commands) if c.lower()=='enablerobot()')
        first_servo=next(i for i,c in enumerate(fixture.commands) if c.lower().startswith('servoj('))
        assert any(c.lower()=='geterrorid()' for c in fixture.commands[:first_enable]) and first_enable<first_servo
        events.append('fresh mode5 precedes low Home action and new-epoch low/high qualification; no ClearError')
        write_json(out/'result.json',{'status':'PASS','simulation':True,'physical_motion_commands':0,
            'enable_commands':fixture.enable_count,'events':events,
            'old_epoch_last_controller_stamp_preserved':old_last_clock,
            'restored_nonzero_net_twist_rad':restored['net_twist_rad'],
            'restored_nonzero_total_twist_rad':restored['total_twist_rad']})
        print((out/'result.json').read_text())
    finally:
        session.close();fixture.close()


if __name__=='__main__':main()
