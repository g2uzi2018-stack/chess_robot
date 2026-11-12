#!/usr/bin/env python3
import json
from pathlib import Path
import subprocess
import time
import urllib.request
import cv2
import numpy as np
import os

base=os.environ.get('PREVIEW_TEST_BASE_URL','http://192.168.30.26:8088')
output=Path(os.environ.get('PREVIEW_TEST_OUTPUT','/home/kuang/workspace/chess_robot/runs/preview-20261025'))
output.mkdir(parents=True,exist_ok=True)
checks=[]
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))

def request(path,post=False):
    req=urllib.request.Request(base+path,method='POST' if post else 'GET')
    with opener.open(req,timeout=4) as response:
        return response.status,response.read(),dict(response.headers)

def metadata():
    status,data,_=request('/api/channels')
    assert status==200
    return json.loads(data)

def subscribed():
    return {c['id'] for c in metadata()['channels'] if c['subscribed']}

def release(token):
    assert request('/api/release?client='+token,post=True)[0]==200

def image(channel,client='acceptance',after=''):
    deadline=time.monotonic()+6
    while time.monotonic()<deadline:
        status,data,headers=request('/api/frame/'+channel+'?client='+client+'&after='+after)
        if status==200:
            decoded=cv2.imdecode(np.frombuffer(data,dtype=np.uint8),cv2.IMREAD_COLOR)
            assert decoded is not None and 0<decoded.shape[1]<=960
            assert headers.get('Cache-Control')=='no-store'
            return data,headers,decoded
        assert status in (202,204),status
        time.sleep(.1)
    raise AssertionError('no fresh image: '+channel)

def pids():
    result={}
    for name,package in [('preview','chess_robot_preview'),('camera','chess_robot_vision')]:
        executable='preview_node' if name=='preview' else 'vision_node'
        pattern='^/home/kuang/workspace/chess_robot/install_vision/'+package+'/lib/'+package+'/'+executable
        values=subprocess.check_output(['pgrep','-f',pattern],text=True).split()
        assert len(values)==1,values
        result[name]=int(values[0])
    return result

def process_sample(pid):
    fields=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
    cpu=(int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK')
    rss=int(next(s.split()[1] for s in Path(f'/proc/{pid}/status').read_text().splitlines() if s.startswith('VmRSS:')))
    return cpu,rss/1024

def resource_sample(duration,channel=None):
    interval=1.0/metadata()['fps'] if channel else .2
    proc=pids();before={k:process_sample(v) for k,v in proc.items()};start=time.monotonic();received=0;frames=0;sequence=''
    while time.monotonic()-start<duration:
        tick=time.monotonic()
        if channel:
            status,data,headers=request('/api/frame/'+channel+'?client=resource&after='+sequence)
            if status==200:
                received+=len(data);frames+=1;sequence=headers['X-Frame-Sequence']
            else:
                assert status in (202,204)
        time.sleep(max(0,interval-(time.monotonic()-tick)))
    elapsed=time.monotonic()-start
    result={'duration_s':elapsed,'jpeg_frames':frames,'jpeg_kib_per_s':received/elapsed/1024}
    for name,pid in proc.items():
        after=process_sample(pid)
        result[name]={'cpu_one_core_percent':100*(after[0]-before[name][0])/elapsed,'rss_mib':after[1]}
    if channel: release('resource')
    return result

release('acceptance');release('other');release('resource')
time.sleep(2.3)
assert subscribed()==set()
before=metadata()
request('/')
time.sleep(1)
after=metadata()
assert [c['encoded_frames'] for c in before['channels']]==[c['encoded_frames'] for c in after['channels']]
checks.append('initial page/metadata: zero image subscriptions and zero additional JPEG encodes')

for channel in ('color','depth'):
    data,headers,decoded=image(channel)
    assert subscribed()=={channel},subscribed()
    (output/(channel+'.jpg')).write_bytes(data)
    checks.append('selection '+channel+': valid JPEG, only selected channel subscribed')

status,data,_=request('/api/frame/detection?client=acceptance')
assert status==202 and json.loads(data)['status']=='no_data'
assert subscribed()=={'detection'}
det=next(c for c in metadata()['channels'] if c['id']=='detection')
assert not det['has_publisher'] and det['encoded_frames']==0
checks.append('detection placeholder: missing topic returns no_data, no invented JPEG')
image('color','other')
assert subscribed()=={'color','detection'}
release('acceptance')
assert subscribed()=={'color'}
image('color','acceptance')
assert subscribed()=={'color'}
checks.append('multiple clients: independent selection and shared subscription for same channel')
release('acceptance');release('other')
assert subscribed()==set()
checks.append('explicit pause/release immediately removes subscriptions')

image('color')
time.sleep(2.5)
assert subscribed()==set()
checks.append('abandoned browser lease expires and unsubscribes')

idle=None if os.environ.get('PREVIEW_TEST_SKIP_RESOURCE') else resource_sample(6)
viewing=None if os.environ.get('PREVIEW_TEST_SKIP_RESOURCE') else resource_sample(8,'color')
time.sleep(2.3)
assert subscribed()==set()

summary={'passed':True,'checks':checks,'idle':idle,'color_view':viewing,'final_state':metadata()}
(output/'integration.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(summary,ensure_ascii=False,indent=2))
