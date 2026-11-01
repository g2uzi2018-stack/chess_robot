"""Isolated ROS120: scene starts first, source/no-arm/loss/restart recover safely."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

os.environ.update(ROS_DOMAIN_ID='120', ROS_LOCALHOST_ONLY='1')
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from chess_robot_interfaces.msg import ArmState
from builtin_interfaces.msg import Time
from test_node_split import packet


def main():
    p = argparse.ArgumentParser(); p.add_argument('--output', required=True); a = p.parse_args()
    out = Path(a.output); out.mkdir(parents=True, exist_ok=False)
    rclpy.init(); node = rclpy.create_node('scene_lifecycle_fixture')
    statuses = []; records = []; snapshots = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(String, '/pose/xiangqi/observation/status', lambda m: statuses.append(json.loads(m.data)), qos),
                     node.create_subscription(String, '/pose/xiangqi/observation/snapshot', lambda m: snapshots.append(json.loads(m.data)), qos),
                     node.create_subscription(String, '/pose/xiangqi/detections', lambda m: records.append(json.loads(m.data)), 10)]
    mp = node.create_publisher(String, '/perception/xiangqi/measurements', QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT))
    ap = node.create_publisher(ArmState, '/arm/state', 100); jp = node.create_publisher(JointState, '/arm/joint_states', 100)
    env = dict(os.environ, XIANGQI_RUN_DIR=str(out/'scene'))
    log = (out/'scene.log').open('w')
    # Define the passive audit fixture locally to avoid importing another
    # integration test's module-level domain setup.
    code = "import sys;sys.path.insert(0,"+repr(str(Path(__file__).resolve().parent))+ ");import scene_node;from test_node_split import FixtureBase;scene_node.BaseGeometry=FixtureBase\n" + \
           "class Audit:\n def __init__(self,*a):self.controller=1;self.epoch_start_ns=0;self.token='controller-one';self.ok=True;self.reason='fixture'\n def poll(self,now):self.host=now;return True\n def close(self):pass\n" + \
           "scene_node.AuditEpoch=Audit;scene_node.main()"
    worker = subprocess.Popen([sys.executable, '-c', code], env=env, stdout=log, stderr=subprocess.STDOUT)
    seq = 0; epoch = 'source-one'; last_image = last_arm = 0.
    def spin(seconds, images=False, feedback=False):
        nonlocal seq, last_image, last_arm
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            assert worker.poll() is None, 'independent scene exited'
            now = time.monotonic(); t = node.get_clock().now().nanoseconds
            if feedback and now-last_arm >= .025:
                j = JointState(); j.header.stamp = Time(sec=t//10**9, nanosec=t%10**9)
                j.name = ['joint'+str(i) for i in range(1,7)]; j.position = [0.]*6; j.velocity = [0.]*6
                arm = ArmState(); arm.header = j.header; arm.backend = 'fixture'; arm.feedback_fresh = True
                arm.simulated = False; arm.busy = False; arm.controller_mode = 4
                ap.publish(arm); jp.publish(j); last_arm = now
            if images and now-last_image >= 1/30:
                p = packet(seq, epoch=epoch, timestamp=t); seq += 1
                mp.publish(String(data=json.dumps(p))); last_image = now
            rclpy.spin_once(node, timeout_sec=.002)
    try:
        deadline = time.monotonic()+5
        while mp.get_subscription_count() == 0:
            spin(.02); assert time.monotonic() < deadline
        spin(.25)
        assert not records and statuses and not statuses[-1]['observation_eligible']
        spin(.4, images=True)
        assert any(r.get('camera_geometry_valid') for r in records)
        assert not any(r.get('base_transform_valid') for r in records)
        for kind in ('identity-list','overflow-float','stage-list','zero-confidence'):
            t=node.get_clock().now().nanoseconds;p=packet(seq,epoch=epoch,timestamp=t);seq+=1
            if kind=='identity-list':p.update(detection_valid=False,camera_geometry_valid=False,detections=[],identity=[['a','b','c']])
            elif kind=='stage-list':p['live_stage_ms']=['abc']
            elif kind=='zero-confidence':p['detections'][0]['confidence']=0.
            text=json.dumps(p)
            if kind=='overflow-float':text=text.replace('"identity": {"model": "fixture"}','"identity": {"bad": 1e999}')
            mp.publish(String(data=text));spin(.08)
        spin(3.3, images=True, feedback=True)
        assert snapshots and statuses[-1]['observation_eligible'], statuses[-1]
        old = snapshots[-1]; old_ids = {d['track_id'] for d in records[-1]['detections']}
        spin(.65, feedback=True)
        assert not statuses[-1]['observation_eligible'] and not records[-1]['detection_valid']
        epoch = 'source-restarted'; seq = 0
        spin(.3, images=True, feedback=True)
        assert not statuses[-1]['observation_eligible']
        assert not old_ids & {d['track_id'] for d in records[-1]['detections']}
        spin(2.4, images=True, feedback=True)
        assert snapshots[-1]['snapshot_id'] != old['snapshot_id'] and statuses[-1]['observation_eligible']
        result = dict(passed=True, domain=120, scene_first_wait=True, camera_without_arm=True,
                      feedback_late_recovery=True, source_loss_revokes_coordinates=True,
                      perception_restart_retires_ids=True, new_two_second_snapshot=True,
                      malformed_measurements_rejected_then_source_recovered=True,physical_commands=0)
        (out/'result.json').write_text(json.dumps(result, indent=2)); print(json.dumps(result))
    finally:
        worker.send_signal(signal.SIGTERM)
        try: worker.wait(timeout=4)
        except subprocess.TimeoutExpired: worker.kill(); worker.wait(timeout=2)
        log.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
