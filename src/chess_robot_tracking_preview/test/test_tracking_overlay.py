"""Isolated-domain integration: fresh tracks draw; stale/moving observations do not."""
import argparse,json,os,subprocess,time
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='119';os.environ['ROS_LOCALHOST_ONLY']='1'
import cv2,numpy as np,rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import String

p=argparse.ArgumentParser();p.add_argument('--binary',required=True);p.add_argument('--output',required=True);a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
log=(out/'renderer.log').open('w');child=subprocess.Popen([a.binary],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
rclpy.init();node=rclpy.create_node('isolated_tracking_overlay_fixture');bridge=CvBridge();received=[]
image_pub=node.create_publisher(Image,'/vision/color/image_rect',qos_profile_sensor_data)
joint_pub=node.create_publisher(JointState,'/arm/joint_states',10)
data_pub=node.create_publisher(String,'/pose/xiangqi/detections',10)
metrics=[]
metric_sub=node.create_subscription(String,'/pose/xiangqi/preview_metrics',lambda m:metrics.append(json.loads(m.data)),10)
sub=node.create_subscription(Image,'/pose/debug/image',received.append,qos_profile_sensor_data)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
def payload(stamp,camera_only=False):
    if camera_only:return dict(stamp_ns=stamp,base_transform_valid=False,detections=[dict(detection_valid=True,camera_geometry_valid=True,valid=False,class_name="red_pawn",confidence=.9,center_pixel=[800,400],camera_surface_center_m=[.1,-.2,.2],mask_outline_xy=[[780,380],[820,380],[820,420],[780,420]])])
    return dict(stamp_ns=stamp,source_joint_position_rad=[0.]*6,base_transform_valid=True,detections=[dict(valid=True,track_confirmed=True,complete_center_target=True,track_id=1,track_class_name='red_pawn',track_class_vote_fraction=1.,surface_tilt_deg=0.,center_pixel=[800,400],base_surface_center_m=[.1,-.2,.01],mask_outline_xy=[[780,380],[820,380],[820,420],[780,420]])])
def observe(seconds,moving=False,publish_data=True,camera_only=False,publish_joint=True):
    begin=node.get_clock().now().nanoseconds;end=time.monotonic()+seconds
    while time.monotonic()<end:
        t=node.get_clock().now();j=JointState();j.header.stamp=t.to_msg();j.name=['joint'+str(i) for i in range(1,7)];j.position=[.02 if moving else 0.]+[0.]*5;j.velocity=[.01 if moving else 0.]+[0.]*5;
        if publish_joint:joint_pub.publish(j)
        if publish_data:data_pub.publish(String(data=json.dumps(payload(t.nanoseconds,camera_only))))
        im=bridge.cv2_to_imgmsg(np.zeros((720,1280,3),np.uint8),'bgr8');im.header.stamp=t.to_msg();im.header.frame_id='d405_color_optical_frame';image_pub.publish(im);spin(.035)
    candidates=[m for m in received if m.header.stamp.sec*10**9+m.header.stamp.nanosec>begin]
    assert len(candidates)>2,'Fresh camera frames did not continue'
    result=bridge.imgmsg_to_cv2(candidates[-1],'bgr8');return result,len(candidates)
try:
    deadline=time.monotonic()+8
    while image_pub.get_subscription_count()==0 or data_pub.get_subscription_count()==0:
        assert time.monotonic()<deadline and child.poll() is None;spin(.05)
    camera,n0=observe(.8,camera_only=True,publish_joint=False);assert camera[400,800,0]>200,'Camera-only prediction absent without arm'
    fresh,n1=observe(.8);assert fresh[400,800,1]>200,'Fresh ID marker absent'
    moving,n2=observe(.7,moving=True);assert moving[400,800,0]>200 and moving[400,800,1]<=200,'Moving-arm base ID was not replaced by prediction'
    fresh,_=observe(.4);assert fresh[400,800,1]>200
    stale,n3=observe(.9,publish_data=False);assert not stale[400,800].any(),'Stale position overlay was not suppressed'
    cv2.imwrite(str(out/'camera-only.png'),camera);cv2.imwrite(str(out/'fresh.png'),fresh);cv2.imwrite(str(out/'stale.png'),stale)
    report=dict(passed=True,domain=119,fresh_track_marker=True,moving_arm_base_id_suppressed=True,camera_only_without_arm=True,stale_xyz_suppressed=True,video_frames_while_fresh=n1,video_frames_while_moving=n2,video_frames_while_stale=n3,real_robot_commands=0)
    (out/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
finally:
    node.destroy_node();rclpy.shutdown();child.terminate()
    try:child.wait(timeout=4)
    except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
    log.close()
