"""READ ONLY: GetAngle queries, feedback, TCP_INFO and host timer jitter."""
import argparse,csv,ipaddress,json,pathlib,socket,struct,threading,time
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--out',type=pathlib.Path,required=True)
p.add_argument('--controller-ip',type=lambda s:str(ipaddress.IPv4Address(s)),required=True)
p.add_argument('--queries',type=int,default=2000);args=p.parse_args()
assert 1<=args.queries<=10000
out=args.out;out.mkdir(parents=True,exist_ok=False)
ip=args.controller_ip;stop=threading.Event();jitter=[];frames=[];errors=[]
def timer():
    due=time.monotonic()+.002
    while not stop.is_set():
        time.sleep(max(0,due-time.monotonic()));now=time.monotonic()
        jitter.append(max(0,now-due));due=now+.002
def feedback():
    try:
        with socket.create_connection((ip,30004),timeout=2) as s:
            s.settimeout(2)
            while not stop.is_set():
                data=b''
                while len(data)<1440:
                    chunk=s.recv(1440-len(data))
                    if not chunk:raise RuntimeError('feedback connection closed')
                    data+=chunk
                assert struct.unpack_from('<H',data)[0]==1440
                frames.append((time.time_ns(),struct.unpack_from('<Q',data,32)[0],struct.unpack_from('<Q',data,24)[0],*struct.unpack_from('<6d',data,432),*struct.unpack_from('<6d',data,480)))
    except Exception as e:errors.append(repr(e))
threads=[threading.Thread(target=timer),threading.Thread(target=feedback)]
for t in threads:t.start()
queries=[]
try:
    with socket.create_connection((ip,29999),timeout=2) as s:
        s.settimeout(.5)
        # Single persistent command stream, with kernel defaults matching C++.
        for i in range(args.queries):
            due=time.monotonic();stamp=time.time_ns();s.sendall(b'GetAngle()');sent=time.monotonic()
            data=s.recv(4096);first=time.monotonic()
            while not data.endswith(b';'):
                chunk=s.recv(4096)
                if not chunk:raise RuntimeError('command connection closed')
                data+=chunk
            end=time.monotonic();assert data.startswith(b'0,'),data
            info=s.getsockopt(socket.IPPROTO_TCP,socket.TCP_INFO,104)
            rtt,rttvar=struct.unpack_from('<II',info,68);retrans=struct.unpack_from('<I',info,100)[0]
            queries.append([stamp,(sent-due)*1000,(first-sent)*1000,(end-first)*1000,(end-due)*1000,rtt,rttvar,retrans,len(data)])
            if i%500==499:print(json.dumps({'queries':i+1,'max_query_ms':max(q[4] for q in queries),'timer_max_ms':max(jitter)*1000}),flush=True)
            time.sleep(max(0,.03-(time.monotonic()-due)))
except Exception as e:
    errors.append(repr(e));raise
finally:
    stop.set()
    for t in threads:t.join()
    for name,rows,header in [('queries',queries,['host_ns','send_ms','first_byte_ms','remaining_reply_ms','total_ms','tcp_rtt_us','tcp_rttvar_us','total_retrans','bytes']),('feedback',frames,['host_ns','controller_stamp','mode',*[f'q{i}_deg' for i in range(6)],*[f'dq{i}_deg_s' for i in range(6)]])]:
        with (out/(name+'.csv')).open('w') as f:w=csv.writer(f);w.writerow(header);w.writerows(rows)
    elapsed=np.array([q[4] for q in queries]);gaps=np.diff([f[0] for f in frames])/1e6
    result={'read_only':True,'motion_commands':0,'queries':len(queries),'errors':errors,
            'query_latency_ms_percentiles':np.quantile(elapsed,[0,.5,.95,.99,1]).tolist() if len(elapsed) else [],
            'query_above_30ms':int(sum(elapsed>30)),'query_above_60ms':int(sum(elapsed>60)),
            'slow_queries':[q for q in queries if q[4]>30],
            'timer_max_jitter_ms':max(jitter,default=0)*1000,'timer_p99_jitter_ms':float(np.quantile(jitter,.99)*1000) if jitter else None,
            'feedback_frames':len(frames),'feedback_max_gap_ms':float(max(gaps)) if len(gaps) else None,
            'modes':sorted(set(f[2] for f in frames)),
            'peak_joint_speed_deg_s':max((abs(v) for f in frames for v in f[9:15]),default=0),
            'tcp_total_retrans':queries[-1][7] if queries else None}
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
