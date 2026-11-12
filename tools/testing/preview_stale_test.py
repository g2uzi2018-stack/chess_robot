#!/usr/bin/env python3
"""Isolated source-loss/configuration test. Does not stop the real camera."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request
import yaml
import rclpy
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

root=Path('/home/kuang/workspace/chess_robot')
out=root/'runs/preview-20261025'
config=yaml.safe_load((root/'src/chess_robot_preview/config/channels.yaml').read_text())
config['preview']['bind_address']='127.0.0.1'
config['preview']['port']=8099
title='测试 "引号"\n第二行'
config['channels']=[dict(id='synthetic',title=title,description='临时验证通道',
                         topic='/preview_test/image',type='image',mode='color')]
file=out/'synthetic-config.yaml';file.write_text(yaml.safe_dump(config,allow_unicode=True))
log=(out/'synthetic-node.log').open('w')
proc=subprocess.Popen(['ros2','run','chess_robot_preview','preview_node','--ros-args',
    '-r','__node:=preview_test_node','-p','config_file:='+str(file)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
def request(path):
    with opener.open('http://127.0.0.1:8099'+path,timeout=3) as response:
        return response.status,response.read()
rclpy.init()
node=rclpy.create_node('preview_test_source')
pub=None
image=Image();image.width=100;image.height=60;image.encoding='bgr8';image.step=300;image.data=bytes([40,120,200])*6000
checks=[]
try:
    deadline=time.monotonic()+8
    while True:
        try:
            assert json.loads(request('/api/channels')[1])['channels'][0]['title']==title
            break
        except Exception:
            if time.monotonic()>deadline:raise
            time.sleep(.1)
    checks.append('adding a YAML channel requires no frontend edits; JSON escaping supports quotes/newlines')
    assert request('/api/frame/synthetic?client=stale')[0]==202
    status,data=request('/api/frame/synthetic?client=stale')
    assert status==202 and json.loads(data)['status']=='no_data'
    assert not json.loads(request('/api/channels')[1])['channels'][0]['has_publisher']
    checks.append('missing publisher: explicit no_data and no cached image')
    pub=node.create_publisher(Image,'/preview_test/image',qos_profile_sensor_data)
    deadline=time.monotonic()+6
    while True:
        image.header.stamp=node.get_clock().now().to_msg();pub.publish(image)
        rclpy.spin_once(node,timeout_sec=.1)
        if request('/api/frame/synthetic?client=stale')[0]==200:break
        assert time.monotonic()<deadline
    checks.append('new channel receives and encodes a live ROS image')
    time.sleep(1.2)
    assert request('/api/frame/synthetic?client=stale')[0]==202
    checks.append('source stops: old cached JPEG is not returned after freshness timeout')
    # Old header stamps must not turn an old source into a fresh preview.
    for _ in range(6):
        pub.publish(image);rclpy.spin_once(node,timeout_sec=.1)
    assert request('/api/frame/synthetic?client=stale')[0]==202
    checks.append('replayed stale image timestamps rejected')
    deadline=time.monotonic()+4
    while True:
        image.header.stamp=node.get_clock().now().to_msg();pub.publish(image)
        rclpy.spin_once(node,timeout_sec=.1)
        if request('/api/frame/synthetic?client=stale')[0]==200:break
        assert time.monotonic()<deadline
    checks.append('source resumes: live preview recovers')
    node.destroy_publisher(pub);pub=None
    time.sleep(1.3)
    status,data=request('/api/frame/synthetic?client=stale')
    assert status==202 and json.loads(data)['status']=='no_data'
    assert not json.loads(request('/api/channels')[1])['channels'][0]['has_publisher']
    checks.append('publisher exits: old display becomes no_data again')
    time.sleep(2.5)
    assert not json.loads(request('/api/channels')[1])['channels'][0]['subscribed']
    checks.append('expired lease releases dynamically added channel')
    result={'passed':True,'checks':checks}
    (out/'source-loss.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False,indent=2))
finally:
    node.destroy_node();rclpy.shutdown()
    if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=6)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    log.close()
