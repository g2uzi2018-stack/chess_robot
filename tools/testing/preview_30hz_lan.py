"""Measure unique JPEG delivery over the Windows-to-AGX LAN HTTP connection."""
import argparse
import json
from pathlib import Path
import time
import urllib.request
import subprocess

parser=argparse.ArgumentParser()
parser.add_argument('--output',required=True)
parser.add_argument('--seconds',type=float,default=8)
parser.add_argument('--ssh-config',default=str(Path(__file__).with_name('ssh_preview_config')))
parser.add_argument('--channels',nargs='+',default=['color','depth'])
args=parser.parse_args()
opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
base='http://192.168.30.26:8088'
client='lan30_'+str(time.time_ns())
def request(path,method='GET'):
    with opener.open(urllib.request.Request(base+path,method=method),timeout=4) as r:
        return r.status,r.read(),dict(r.headers)
def metadata():return json.loads(request('/api/channels')[1])
def resource_snapshot():
    return json.loads(subprocess.check_output(['ssh','-F',args.ssh_config,
        'agx-preview','python3','/home/kuang/workspace/chess_robot/tools/testing/preview_resource_snapshot.py'],text=True))
results=[]
try:
    assert metadata()['fps']==30
    for channel in args.channels:
        sequence='';warm=time.perf_counter()+1.5
        while time.perf_counter()<warm:
            status,data,headers=request('/api/frame/'+channel+'?client='+client+'&after='+sequence)
            if status==200:sequence=headers['X-Frame-Sequence']
            else:assert status in (202,204)
            time.sleep(.01)
        cpu_before=resource_snapshot();before=metadata();start=time.perf_counter();times=[];sequences=[];sizes=[];rtts=[];waits=0
        while time.perf_counter()-start<args.seconds:
            tick=time.perf_counter()
            status,data,headers=request('/api/frame/'+channel+'?client='+client+'&after='+sequence)
            if status==200:
                new_sequence=headers['X-Frame-Sequence']
                assert new_sequence!=sequence
                assert data.startswith(b'\xff\xd8') and data.endswith(b'\xff\xd9')
                sequence=new_sequence;sequences.append(int(sequence));times.append(time.perf_counter());sizes.append(len(data));rtts.append(time.perf_counter()-tick)
            else:assert status in (202,204);waits+=1
            time.sleep(max(0,1/30-(time.perf_counter()-tick)))
        elapsed=time.perf_counter()-start;after=metadata();cpu_after=resource_snapshot()
        delivered=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
        a=next(c for c in before['channels'] if c['id']==channel)
        b=next(c for c in after['channels'] if c['id']==channel)
        result=dict(channel=channel,duration_s=elapsed,jpeg_frames=len(times),unique_jpeg_fps=delivered,
            jpeg_mib_per_s=sum(sizes)/elapsed/1024**2,response_time_ms_mean=1000*sum(rtts)/len(rtts) if rtts else None,
            encoding_fps=(b['encoded_frames']-a['encoded_frames'])/elapsed,waiting_responses=waits,
            received_ros_fps=(b.get('received_frames',0)-a.get('received_frames',0))/elapsed,
            viewers_start=before['viewers'],viewers_end=after['viewers'])
        for name in ('preview','camera'):
            assert cpu_before[name]['pid']==cpu_after[name]['pid']
            result[name]={'cpu_one_core_percent':100*(cpu_after[name]['cpu_s']-cpu_before[name]['cpu_s'])/(cpu_after['monotonic_s']-cpu_before['monotonic_s']),
                          'rss_mib':cpu_after[name]['rss_mib']}
        results.append(result)
finally:
    request('/api/release?client='+client,'POST')
summary={'passed':all(r['unique_jpeg_fps']>=28.5 for r in results) and len(results)==len(args.channels),
    'target_fps':30,'method':'real Windows client over LAN, unique JPEG sequence, one request in flight',
    'clock':time.get_clock_info('perf_counter').implementation,
    'clock_resolution_s':time.get_clock_info('perf_counter').resolution,
    'results':results}
Path(args.output).write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False,indent=2))
raise SystemExit(0 if summary['passed'] else 1)
