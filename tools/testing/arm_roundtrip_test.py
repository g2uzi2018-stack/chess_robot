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


if __name__=='__main__':unittest.main(verbosity=2)
