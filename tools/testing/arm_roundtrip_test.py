#!/usr/bin/env python3
"""Offline orchestration regressions; fake session never opens a socket."""
import json
import pathlib
import tempfile
import unittest
from arm_roundtrip import Runner,RosSession,ExecutionFault,manifest,validate_manifest,return_precheck,boundary_bracket,apply_controller_limits


HOME={'tcp_m':[0.,-.5,.3],'joints_rad':[0.]*6,'quaternion_xyzw':[0.,0.,0.,1.],'home_yaml_sha256':'fixture'}


class Session:
    def __init__(self,reject=None,fault=None,precheck=True):
        self.reject,self.fault,self.precheck=reject,fault,precheck
        self.position=HOME['tcp_m'];self.calls=[];self.arrivals=[]
    def require_home(self):
        if self.position!=HOME['tcp_m']:
            raise AssertionError('not at formal Home')
    def action(self,point,profile,plan_only):
        self.calls.append((point,profile,plan_only))
        if plan_only:
            rejected=self.reject and self.reject(point)
            return {'code':14 if rejected else 0,'detail':'constraint rejected' if rejected else
                    ('home_return_checked=true home_return_duration_s=1.5' if self.precheck else 'outbound only'),
                    'executed':False,'planned_duration_s':1.,'profile':profile}
        if self.fault=='outbound' and point is not None:
            return {'code':16,'detail':'tracking failure','executed':True,'profile':profile}
        if self.fault=='return' and point is None:
            return {'code':19,'detail':'Stop failure','executed':True,'profile':profile}
        self.position=point['tcp_m'] if point else HOME['tcp_m']
        return {'code':1,'detail':'measured','executed':True,'profile':profile,'final_tcp_m':self.position}
    def choose_profile(self,point,plan):
        return ('roundtrip_low' if point.get('kind')=='refinement' else 'roundtrip_high',['fixture'],None)
    def require_arrival(self,result,xyz,quaternion):
        self.arrivals.append(xyz)
        if result['code']!=1 or result['final_tcp_m']!=xyz:
            raise ExecutionFault('not measured arrived')
    def confirm_stopped(self):
        return {'confirmed':self.fault!='return'}
    def make_speed_evidence(self,trials):
        return {'status':'PASS','simulation':True,'trial_count':len(trials)}


class Tests(unittest.TestCase):
    def point(self,name='one',distance=.2):
        return {'id':name,'tcp_m':[distance,-.5,.3],'kind':'coarse','ray':'right','distance_m':distance}
    def data(self,points=None):
        d=manifest(HOME)
        d['coarse_points']=points if points is not None else [self.point()]
        d['rays']={'right':[1,0,0]}
        d['max_refinements_per_ray']=0
        return d
    def runner(self,session,execute=True,data=None):
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        return Runner(data or self.data(),session,pathlib.Path(directory.name),execute)
    def test_exact_endpoint_two_legs_and_formal_home(self):
        s=Session();r=self.runner(s);result=r.run()
        self.assertEqual(result['targets'][0]['status'],'roundtrip_success')
        actual=[p for p,profile,plan in s.calls if not plan]
        self.assertEqual(actual,[self.point(),None])
        self.assertEqual(s.arrivals,[self.point()['tcp_m'],HOME['tcp_m']])
        self.assertEqual(result['reference_initializations'],0)
    def test_outbound_fault_never_forces_home_or_replays(self):
        s=Session(fault='outbound');r=self.runner(s)
        with self.assertRaises(ExecutionFault):r.run()
        self.assertEqual(len([x for x in s.calls if not x[2]]),1)
        self.assertEqual(r.report['status'],'EXECUTION_FAULT_STOPPED')
        self.assertIsNone(r.report['targets'][0]['return_home'])
    def test_return_fault_not_roundtrip_success(self):
        r=self.runner(Session(fault='return'))
        with self.assertRaises(ExecutionFault):r.run()
        self.assertEqual(r.report['targets'][0]['status'],'execution_failed')
        self.assertEqual(r.report['status'],'EXECUTION_FAULT_STOP_UNCONFIRMED')
    def test_both_paths_required_before_first_motion(self):
        s=Session(precheck=False);r=self.runner(s)
        with self.assertRaises(ValueError):r.run()
        self.assertFalse(any(not x[2] for x in s.calls))
    def test_planning_reject_continues_then_finite_boundary(self):
        s=Session(reject=lambda p:p['distance_m']>.25)
        data=self.data([self.point('near',.15),self.point('far',.6)]);data['max_refinements_per_ray']=2
        r=self.runner(s,data=data);result=r.run()
        self.assertLessEqual(len(result['targets']),4)
        self.assertEqual(result['targets'][1]['status'],'planning_rejected')
        self.assertTrue(all(x['profile']=='roundtrip_low' for x in result['targets'] if x.get('kind')=='refinement' and x['status']=='roundtrip_success'))
        low,high=boundary_bracket(result['targets'],'right')
        self.assertLess(low,high)
    def test_plan_pass_never_becomes_measured_success(self):
        s=Session();result=self.runner(s,execute=False).run()
        self.assertEqual(result['targets'][0]['status'],'planning_passed')
        self.assertFalse(any(not x[2] for x in s.calls))
        self.assertEqual(result['boundaries']['right']['farthest_measured_roundtrip_m'],0.)
    def test_manifest_is_finite_and_does_not_redefine_home(self):
        d=manifest(HOME);validate_manifest(d,HOME)
        self.assertEqual(len(d['coarse_points']),26)
        self.assertEqual(d['maximum_targets'],38)
        self.assertEqual(d['quaternion_xyzw'],HOME['quaternion_xyzw'])
        bad=json.loads(json.dumps(d));bad['home']['joints_rad'][0]=1
        with self.assertRaises(ValueError):validate_manifest(bad,HOME)
    def test_fake_current_home_to_home_is_not_return_precheck(self):
        with self.assertRaises(ValueError):return_precheck('PLAN_READY duration=0.03')
    def test_faulted_batch_cannot_be_auto_resumed(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):Runner(self.data(),Session(),directory,True,{'status':'EXECUTION_FAULT_STOPPED'})
    def test_qualification_is_two_fresh_pairs_in_same_tool_then_reused(self):
        s=Session();s.needs_qualification=True
        r=self.runner(s);result=r.run()
        self.assertEqual(len(result['qualification_trials']),2)
        self.assertEqual([x['profile'] for x in result['qualification_trials']],['roundtrip_low','roundtrip_high'])
        self.assertEqual(result['targets'][0]['status'],'reused_success')
        self.assertEqual(len([x for x in s.calls if not x[2]]),4)
    def test_failed_qualification_retains_both_leg_evidence_without_retry(self):
        for fault in ['outbound','return']:
            with self.subTest(fault=fault):
                s=Session(fault=fault);s.needs_qualification=True
                r=self.runner(s)
                with self.assertRaises(ExecutionFault):r.run()
                self.assertEqual(r.report['targets'],[])
                self.assertEqual(len(r.report['qualification_trials']),1)
                failed=r.report['qualification_trials'][0]
                self.assertEqual(failed['status'],'execution_failed')
                self.assertEqual(failed['id'],'qualification_roundtrip_low')
                self.assertEqual(failed['outbound']['executed'],True)
                self.assertEqual(failed['return_home'] is None,fault=='outbound')
                self.assertEqual(len([x for x in s.calls if not x[2]]),1 if fault=='outbound' else 2)
                self.assertTrue(r.report['status'].startswith('EXECUTION_FAULT'))
                saved=json.loads((r.out/'results.json').read_text())
                self.assertEqual(saved['qualification_trials'],r.report['qualification_trials'])
                self.assertFalse((r.out/'speed_qualification.json').exists())
    def test_source_backed_limits_are_applied_to_node_not_just_labelled(self):
        p={'controller':{'firmware_version':'4.6.2.4'},'limits':{'velocity_rad_s':[2.36]*6,'acceleration_rad_s2':[4.72]*6},
           'profiles':{'roundtrip_low':{'speed_rad_s':.708,'acceleration_rad_s2':1.416},
                       'roundtrip_high':{'speed_rad_s':2.36,'acceleration_rad_s2':4.72}}}
        limits={'model':'Nova5','firmware_version':'4.6.2.4','source':{'kind':'unit_fixture','reference':'not real evidence'},
                'units':{'velocity':'rad/s','acceleration':'rad/s2'},'velocity_rad_s':[1.5]*6,'acceleration_rad_s2':[3.]*6}
        effective=apply_controller_limits(p,limits)
        self.assertEqual(p['profiles']['roundtrip_high']['speed_rad_s'],1.5)
        self.assertEqual(p['profiles']['roundtrip_high']['acceleration_rad_s2'],3.)
        self.assertEqual(effective['high']['per_axis_velocity_rad_s'],[1.5]*6)
        limits['units']['velocity']='percent'
        with self.assertRaises(ValueError):apply_controller_limits(p,limits)
    def test_explicit_recovery_is_one_low_home_not_reference_reset(self):
        from types import SimpleNamespace
        class Recoverable(Session):
            def require_home(self):
                if self.position!=HOME['tcp_m']:raise ValueError('not Home')
            def require_reference_idle(self):pass
        s=Recoverable();s.position=[.01,-.5,.3];s.args=SimpleNamespace(recover_home=True)
        r=self.runner(s);result=r.run()
        self.assertEqual(result['recovery_home']['status'],'MEASURED_HOME_REACHED')
        calls=[x for x in s.calls if not x[2]]
        self.assertIsNone(calls[0][0]);self.assertEqual(calls[0][1],'roundtrip_low')
        self.assertEqual(result['reference_resets'],0)
    def test_execution_replans_and_does_not_use_cached_high_margin(self):
        class ChangedMargin(Session):
            calls_to_classifier=0
            def choose_profile(self,point,plan):
                self.calls_to_classifier+=1
                return ('roundtrip_high' if self.calls_to_classifier==1 else 'roundtrip_low',['new path'],None)
        s=ChangedMargin();result=self.runner(s).run()
        actual=[profile for point,profile,plan in s.calls if not plan]
        self.assertEqual(actual,['roundtrip_low','roundtrip_low'])
        self.assertEqual(result['targets'][0]['preflight_planning']['selected']['profile'],'roundtrip_high')
        self.assertEqual(result['targets'][0]['planning']['selected']['profile'],'roundtrip_low')
    def test_handle_timeout_shutdowns_owned_business_node_without_resend(self):
        import signal
        from types import SimpleNamespace
        class Process:
            def __init__(self):self.signals=[];self.done=False
            def poll(self):return 0 if self.done else None
            def send_signal(self,s):self.signals.append(s)
            def wait(self,timeout):self.done=True;return 0
        class Client:
            sends=0
            def send_goal_async(self,goal):self.sends+=1;return object()
        with tempfile.TemporaryDirectory() as directory:
            s=RosSession.__new__(RosSession);s.out=pathlib.Path(directory);s.frames=[]
            s.p={'stop_timeout_s':4};s.process=Process();s.client=Client()
            def timeout(future,budget):raise TimeoutError('handle response lost after acceptance')
            s.future=timeout
            with self.assertRaises(ExecutionFault):s.goal_handle(object(),False,123)
            self.assertEqual(s.client.sends,1)
            self.assertEqual(s.process.signals,[signal.SIGINT])
            self.assertFalse(s.confirm_stopped()['confirmed'])
            evidence=json.loads((s.out/'ambiguous_goal_shutdown.json').read_text())
            self.assertIsNone(evidence['executed'])
            self.assertFalse(evidence['stop_confirmation']['confirmed'])


    def test_stale_client_snapshot_must_refresh_before_any_action_submission(self):
        from types import SimpleNamespace
        class Goal:
            HOME=4;PREGRASP=0
        s=RosSession.__new__(RosSession);s.MoveArm=SimpleNamespace(Goal=Goal)
        sent=[]
        s.goal_handle=lambda *args:sent.append(args)
        def unavailable(predicate,budget):raise TimeoutError('no genuinely fresh matched snapshot')
        s.wait=unavailable
        with self.assertRaises(TimeoutError):s.action(None,'roundtrip_low',True)
        self.assertEqual(sent,[])

    def test_terminal_feedback_fault_is_saved_even_when_snapshot_disappears(self):
        from types import SimpleNamespace
        class Goal:
            HOME=4;PREGRASP=0
        with tempfile.TemporaryDirectory() as directory:
            s=RosSession.__new__(RosSession);s.out=pathlib.Path(directory);s.frames=[]
            (s.out/'node.log').write_text('')
            s.MoveArm=SimpleNamespace(Goal=Goal);s.args=SimpleNamespace(goal_timeout=3)
            s.node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=123)))
            calls=[]
            def wait(predicate,budget):
                calls.append(1)
                if len(calls)>1:raise TimeoutError('feedback disappeared after action failed')
            s.wait=wait;s.snapshot=lambda *args:{'stamp_ns':100}
            handle=SimpleNamespace(accepted=True,get_result_async=lambda:object())
            s.goal_handle=lambda *args:handle
            terminal=SimpleNamespace(code=13,detail='feedback lost during task',executed=True,
                final_tcp_valid=False,trajectory=SimpleNamespace(points=[]))
            s.future=lambda *args:SimpleNamespace(result=terminal,status=6)
            record=s.action(None,'roundtrip_low',False)
            self.assertEqual(record['code'],13);self.assertTrue(record['executed'])
            self.assertIsNone(record['after']);self.assertIn('feedback disappeared',record['after_feedback_error'])
            self.assertEqual(json.loads((s.out/'actions.json').read_text())[0]['code'],13)


    def test_user_command_cap_is_applied_without_hardware_certification(self):
        from arm_roundtrip import apply_command_policy
        p={'limits':{'velocity_rad_s':[2.36]*6,'acceleration_rad_s2':[4.72]*6},
           'profiles':{'roundtrip_low':{'speed_rad_s':.708,'acceleration_rad_s2':1.416},
                       'roundtrip_high':{'speed_rad_s':2.36,'acceleration_rad_s2':4.72}}}
        policy={'schema':'chess_robot.command_policy/v1','user_statement':'先按照1rad每秒走',
                'hardware_limits_verified':False,'velocity_cap_rad_s':1.,'acceleration_cap_rad_s2':1.,
                'profiles':{'low':{'speed_rad_s':.3,'acceleration_rad_s2':.3},
                            'high':{'speed_rad_s':1.,'acceleration_rad_s2':1.}}}
        effective=apply_command_policy(p,policy)
        self.assertEqual(effective['high']['per_axis_velocity_rad_s'],[1.]*6)
        self.assertEqual(p['profiles']['roundtrip_low']['speed_rad_s'],.3)
        p['limits']['velocity_rad_s'][0]=.2
        self.assertEqual(apply_command_policy(p,policy)['high']['per_axis_velocity_rad_s'][0],.2)
        policy['hardware_limits_verified']=True
        with self.assertRaises(ValueError):apply_command_policy(p,policy)



class PowerCycleTests(unittest.TestCase):
    def context(self):
        return {'home_sha256':'fixture','geometry_sha256':'g','urdf_sha256':{'urdf':'u'},
            'sdk_manifest_sha256':'sdk','sdk_patch_sha256':'patch','tcp':{'translation':[0,0,.2]},
            'profiles':{'low':.3,'high':1.},'limits':{'v':[1.]*6},'exploration':{'radius':1.1},
            'firmware_version':'4.6.2.4','controller_ip':'127.0.0.1','controller_limit_sha256':None,
            'command_policy_sha256':'policy','motion_parameters':{'feedback':{'freshness_s':.1},
                'reference':{'max_joint_step_rad':.1}},'binary_sha256':'old',
            'source_sha256':{'driver':'old-driver'}}
    def prior(self):
        data=manifest(HOME);point=data['coarse_points'][0]
        leg={'code':1,'executed':True,'final_tcp_valid':True,'after':{'simulated':True},
             'final_stamp_ns':111,'command_ros_ns':100}
        record=dict(point,status='roundtrip_success',profile='roundtrip_high',
                    outbound=dict(leg),return_home=dict(leg))
        return {'status':'EXECUTION_FAULT_STOP_UNCONFIRMED','simulation':True,'home':data['home'],
                'manifest':data,'qualification_context':self.context(),'targets':[record]},data
    def files(self,directory):
        from arm_roundtrip import sha
        previous,data=self.prior();base=pathlib.Path(directory)
        prior=base/'prior.json';prior.write_text(json.dumps(previous))
        statement=base/'statement.json';statement.write_text(json.dumps({'human_statement':'fixture explicit continue remaining points'}))
        evidence=base/'power.json';evidence.write_text(json.dumps({'human_statement':'fixture power only no manual motion',
            'only_power_cycle_no_manual_joint_or_cable_motion':True,
            'prior_fault_result':{'path':str(prior),'sha256':sha(prior)}}))
        return previous,data,prior,statement,evidence
    def test_power_requires_actual_confirmation_and_prior_hash(self):
        from arm_roundtrip import load_power_cycle_evidence
        with tempfile.TemporaryDirectory() as directory:
            previous,data,prior,statement,evidence=self.files(directory)
            accepted=load_power_cycle_evidence(evidence)
            self.assertTrue(accepted['epoch_handoff_explicit'])
            raw=json.loads(evidence.read_text());raw['only_power_cycle_no_manual_joint_or_cable_motion']=False
            evidence.write_text(json.dumps(raw))
            with self.assertRaises(ValueError):load_power_cycle_evidence(evidence)
            raw['only_power_cycle_no_manual_joint_or_cable_motion']=True;raw['prior_fault_result']['sha256']='wrong'
            evidence.write_text(json.dumps(raw))
            with self.assertRaises(ValueError):load_power_cycle_evidence(evidence)
    def test_readonly_or_missing_power_evidence_never_enables_node(self):
        from arm_roundtrip import configure_power_cycle_handoff
        p={'reference':{},'controller':{}}
        self.assertFalse(configure_power_cycle_handoff(p,{'human':'fixture'},False))
        self.assertFalse(p['controller']['enable_after_history_restore'])
        self.assertFalse(configure_power_cycle_handoff(p,None,True))
        self.assertTrue(configure_power_cycle_handoff(p,{'human':'fixture'},True))
        self.assertTrue(p['reference']['power_cycle_handoff'])
        self.assertTrue(p['controller']['enable_after_history_restore'])
    def test_explicit_partial_continuation_preserves_old_context_and_time(self):
        from arm_roundtrip import load_continuation
        with tempfile.TemporaryDirectory() as directory:
            previous,data,prior,statement,evidence=self.files(directory)
            current=self.context();current['binary_sha256']='new';current['source_sha256']={'driver':'fixed'}
            current['motion_parameters']['reference']['power_cycle_handoff']=True
            current['restore_parameters']={'enable_after_history_restore':True}
            continued=load_continuation(prior,statement,current,data,True)
            self.assertEqual(continued['historical_success_count'],1)
            self.assertEqual(continued['previous_qualification_context']['binary_sha256'],'old')
            point=next(iter(continued['successes'].values()))
            self.assertEqual(point['outbound']['final_stamp_ns'],111)
            self.assertIn('binary_sha256',continued['current_context_delta'])
            self.assertEqual(json.loads(prior.read_text()),previous)
    def test_continuation_still_rejects_safety_or_feedback_change(self):
        from arm_roundtrip import load_continuation
        with tempfile.TemporaryDirectory() as directory:
            previous,data,prior,statement,evidence=self.files(directory)
            current=self.context();current['motion_parameters']['feedback']['freshness_s']=.25
            with self.assertRaises(ValueError):load_continuation(prior,statement,current,data,True)
            current=self.context();current['command_policy_sha256']='different'
            with self.assertRaises(ValueError):load_continuation(prior,statement,current,data,True)
            with self.assertRaises(ValueError):load_continuation(prior,None,self.context(),data,True)
            with self.assertRaises(ValueError):load_continuation(prior,statement,self.context(),data,False)
    def test_feedback_window_explicit_change_only_preserves_original_context(self):
        from arm_roundtrip import load_continuation,load_feedback_window_evidence,sha
        with tempfile.TemporaryDirectory() as directory:
            previous,data,prior,statement,evidence=self.files(directory)
            window=pathlib.Path(directory)/'window.json'
            raw={'human_statement':'fixture human explicitly says expand freshness from 100 to 200ms',
                 'previous_freshness_s':.1,'current_freshness_s':.2,
                 'prior_results':{'path':str(prior),'sha256':sha(prior)},
                 'latest_fault_result':{'path':str(prior),'sha256':sha(prior)}}
            window.write_text(json.dumps(raw))
            accepted=load_feedback_window_evidence(window,.2)
            current=self.context();current['motion_parameters']['feedback']['freshness_s']=.2
            current['motion_parameters']['reference']['power_cycle_join_file']='fixture-declared-epoch.bin'
            continued=load_continuation(prior,statement,current,data,True,accepted)
            self.assertFalse(continued['safety_invariants_equal'])
            self.assertEqual(continued['authorized_feedback_window_change']['current_s'],.2)
            self.assertEqual(continued['previous_qualification_context']['motion_parameters']['feedback']['freshness_s'],.1)
            self.assertEqual(json.loads(prior.read_text()),previous)
            current['limits']['v'][0]=.9
            with self.assertRaises(ValueError):load_continuation(prior,statement,current,data,True,accepted)
    def test_feedback_window_rejects_missing_changed_or_unbound_evidence(self):
        from arm_roundtrip import load_feedback_window_evidence,load_continuation,sha
        self.assertIsNone(load_feedback_window_evidence(None,.1))
        with self.assertRaises(ValueError):load_feedback_window_evidence(None,.2)
        with self.assertRaises(ValueError):load_feedback_window_evidence(None,.25)
        with tempfile.TemporaryDirectory() as directory:
            previous,data,prior,statement,evidence=self.files(directory)
            path=pathlib.Path(directory)/'window.json'
            raw={'human_statement':'fixture expand 200ms','previous_freshness_s':.1,'current_freshness_s':.2,
                'prior_results':{'path':str(prior),'sha256':sha(prior)},
                'latest_fault_result':{'path':str(prior),'sha256':'changed'}}
            path.write_text(json.dumps(raw))
            with self.assertRaises(ValueError):load_feedback_window_evidence(path,.2)
            raw['latest_fault_result']['sha256']=sha(prior);raw['human_statement']=''
            path.write_text(json.dumps(raw))
            with self.assertRaises(ValueError):load_feedback_window_evidence(path,.2)
    def test_historical_success_not_reexecuted_or_overwritten_by_qualification(self):
        from arm_roundtrip import load_continuation
        with tempfile.TemporaryDirectory() as directory:
            previous,data,prior,statement,evidence=self.files(directory)
            data['coarse_points']=data['coarse_points'][:1];data['rays']={};data['max_refinements_per_ray']=0
            continued=load_continuation(prior,statement,self.context(),data,True)
            session=Session();session.needs_qualification=True
            runner=Runner(data,session,pathlib.Path(directory),True,continuation=continued)
            result=runner.run()
            self.assertEqual(len([c for c in session.calls if not c[2]]),4)
            self.assertEqual(result['targets'][0]['reuse_origin'],'historical_continuation')
            self.assertEqual(result['targets'][0]['outbound']['final_stamp_ns'],111)
            self.assertEqual(len(result['qualification_trials']),2)
            self.assertEqual(result['preflight_estimate_s']['new_preflight_count'],0)
    def test_every_preflight_result_persisted_before_next_point(self):
        class CheckSaved(Session):
            def action(self,point,profile,plan_only):
                if plan_only and point['id']=='second':
                    saved=json.loads((self.output/'results.json').read_text())
                    assert saved['coarse_preflight']['first']['status']=='planning_passed'
                return super().action(point,profile,plan_only)
        with tempfile.TemporaryDirectory() as directory:
            data=manifest(HOME);data['coarse_points']=[{'id':'first','kind':'coarse','tcp_m':[.1,-.5,.3]},
                {'id':'second','kind':'coarse','tcp_m':[.2,-.5,.3]}];data['rays']={};data['max_refinements_per_ray']=0
            session=CheckSaved();session.output=pathlib.Path(directory)
            Runner(data,session,directory,False).run()
    def test_pre_send_failure_classified_without_submitting_motion(self):
        from arm_roundtrip import PreSendFeedbackFault
        class MissingFeedback(Session):
            def action(self,point,profile,plan_only):
                if not plan_only:raise PreSendFeedbackFault('mode9 before send',{'controller_mode':9,'diagnostic':'controller fault'})
                return super().action(point,profile,plan_only)
        with tempfile.TemporaryDirectory() as directory:
            data=manifest(HOME);data['coarse_points']=data['coarse_points'][:1];data['rays']={};data['max_refinements_per_ray']=0
            session=MissingFeedback();runner=Runner(data,session,directory,True)
            with self.assertRaises(ExecutionFault):runner.run()
            record=runner.report['targets'][0]
            self.assertEqual(record['status'],'pre_send_feedback_failed')
            self.assertEqual(record['fault_stage'],'before_action_submission')
            self.assertEqual(record['feedback_diagnostic']['controller_mode'],9)
            self.assertFalse(any(not c[2] for c in session.calls))

if __name__=='__main__':unittest.main(verbosity=2)
