"""Offline evidence summary/plots for one or more controlled exploration batches."""
import argparse,csv,html,json,pathlib,re,subprocess
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('--run',type=pathlib.Path,action='append',required=True)
p.add_argument('--out',type=pathlib.Path,required=True)
p.add_argument('--project-root',type=pathlib.Path,default=pathlib.Path(__file__).resolve().parents[2])
p.add_argument('--evaluator',type=pathlib.Path);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
geometry=yaml.safe_load((a.project_root/'src/chess_robot_arm/config/nova5_ag95_geometry.yaml').read_text())
origins=np.array(geometry['joint_origins']).reshape(6,4,4);tcp=np.array(geometry['tcp']).reshape(4,4)
all_goals=[];checklist={};boundaries=[];scans=[];traces=[];times=[];nets=[];totals=[];all_xyz=[];acks=[];gaps=[];pending=[];restores=[];margins=[];speeds=[]
reports=[];raw_speed=[];raw_times=[];stop_acks=[];twist_breaks=[];raw_breaks=[]
for run in a.run:
    report=json.loads((run/'results.json').read_text());reports.append(report)
    if (run/'controlled_checkpoint.json').exists():
        report['status']='CONTROLLED_CHECKPOINT'
    for g in report['goals']:g=dict(g,run=run.name);all_goals.append(g)
    for c in report['checklist']:
        if c['id'] not in checklist or c['status']=='reached' or checklist[c['id']]['status']!='reached':checklist[c['id']]=dict(c,source_report=str(run/'results.json'))
    boundaries.extend(dict(b,run=run.name) for b in report.get('boundaries',[]));scans.extend(dict(s,run=run.name) for s in report.get('planning_scan',[]));restores.append(report.get('restored'))
    rows=list(csv.DictReader((run/'joint_feedback.csv').open()))
    rows=[r for r in rows if r.get('total_twist_rad')]
    # ArmState publishes zero metric fields while the reference is invalid.
    # Those placeholders must never appear as cable zeroing in the time line.
    states=[json.loads(line) for line in (run/'state_history.jsonl').read_text().splitlines()]
    intervals=[];start=None
    for event in states:
        valid=event['reference'] and event['fresh']
        if valid and start is None:start=event['host_utc_ns']
        if not valid and start is not None:intervals.append((start,event['host_utc_ns']));start=None
    if start is not None:intervals.append((start,2**63-1))
    rows=[r for r in rows if any(begin<=int(r['host_utc_ns'])<end for begin,end in intervals)]
    if rows:
        q=np.array([[float(r[f'q{i}_rad']) for i in range(1,7)] for r in rows])
        t=np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
        for i in range(6):
            axis=np.array(geometry['axes'][i]);x,y,z=axis
            k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]);rot=np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
            rot[:,:3,:3]=np.eye(3)+np.sin(q[:,i,None,None])*k+(1-np.cos(q[:,i,None,None]))*(k@k)
            t=t@origins[i]@rot
        xyz=(t@tcp)[:,:3,3];all_xyz.append(xyz)
        if times:twist_breaks.append(len(times))
        times.extend(int(r['host_utc_ns']) for r in rows);nets.extend(float(r['net_twist_rad']) for r in rows);totals.extend(float(r['total_twist_rad']) for r in rows)
        speeds.append(max(abs(float(r[f'dq{i}_rad_s'])) for r in rows for i in range(1,7)))
        ids=np.unique(np.linspace(0,len(xyz)-1,min(2200,len(xyz))).astype(int));line=xyz[ids]
        traces.append(dict(type='scatter3d',mode='lines',name='真实反馈轨迹 · '+run.name,x=line[:,0].tolist(),y=line[:,1].tolist(),z=line[:,2].tolist(),line=dict(color='#25874b',width=2),hoverinfo='skip'))
        if a.evaluator:
            q.astype('<f8').tofile(a.out/'measured_q.bin')
            subprocess.run([str(a.evaluator),str(a.project_root/'src/chess_robot_arm/config/nova5_ag95_geometry.yaml'),str(a.out/'measured_q.bin'),str(a.out/'measured_metrics.bin')],check=True,stdout=subprocess.DEVNULL)
            m=np.fromfile(a.out/'measured_metrics.bin',dtype='<f8').reshape(-1,12)
            margins.append({'run':run.name,'samples':len(m),'table_clearance_lower_bound_m':float(m[:,4].min()+.01),'joint_margin_rad':float(m[:,5].min()),'sigma_min':float(m[:,6].min()),'condition_max':float(m[:,7].max()),'self_clearance_lower_bound_m':float(m[:,11].min()),'table_pass':bool(np.all(m[:,9]==1)),'self_pass':bool(np.all(m[:,10]==1))})
            assert margins[-1]['table_pass'] and margins[-1]['self_pass']
            (a.out/'measured_q.bin').unlink();(a.out/'measured_metrics.bin').unlink()
    cp=run/'audit/commands.csv'
    if cp.exists():
        commands=list(csv.DictReader(cp.open()));acks.extend(float(r['latency_s']) for r in commands if r.get('command','').startswith('ServoJ') and r.get('latency_s') and r.get('reply_or_error','').startswith('0,'))
        stop_acks.extend({'run':run.name,'latency_s':float(r['latency_s']),'host_utc_ns':int(r['host_utc_ns'])} for r in commands if r.get('command')=='Stop()' and r.get('reply_or_error','').startswith('0,'))
    raw_path=run/'audit/feedback.bin'
    if raw_path.exists():
        count=raw_path.stat().st_size//1448
        raw=np.memmap(raw_path,mode='r',dtype=np.dtype([('host','<i8'),('frame','u1',(1440,))]),shape=(count,))
        velocity=raw['frame'][:,480:528].copy().view('<f8').reshape(-1,6)*np.pi/180
        if raw_times:raw_breaks.append(len(raw_times))
        raw_times.extend(raw['host'].tolist());raw_speed.extend(np.max(abs(velocity),axis=1).tolist())
    sp=run/'audit/servo_send.csv'
    if sp.exists():
        sends=[r for r in csv.DictReader(sp.open()) if r.get('pending_count')]
        pending.extend(int(r['pending_count']) for r in sends)
        intervals=np.diff([int(r['host_utc_ns']) for r in sends])/1e9;gaps.extend(intervals[(intervals>0)&(intervals<.1)].tolist())

def add(name,points,labels,color,symbol,size=5):
    if not points:return
    xyz=np.array(points);traces.append(dict(type='scatter3d',mode='markers',name=name,x=xyz[:,0].tolist(),y=xyz[:,1].tolist(),z=xyz[:,2].tolist(),text=labels,marker=dict(color=color,symbol=symbol,size=size),hovertemplate='%{text}<br>X %{x:.4f} m<br>Y %{y:.4f} m<br>Z %{z:.4f} m<extra>%{fullData.name}</extra>'))
latest_scans={g['id']:g for g in scans}
for passed,color,symbol in [(True,'#3478bf','circle'),(False,'#c37015','x')]:
    group=[g for g in latest_scans.values() if (g['code']==0)==passed]
    for b in boundaries:group.extend(dict(g,run=b['run']) for g in b['probes'] if (g['code']==0)==passed)
    add('规划通过' if passed else '规划拒绝',[g['target_m'] for g in group],[html.escape(g.get('id','边界探针')+' | '+g.get('run','readonly boundary')+' | '+g['detail']) for g in group],color,symbol,4)
for code,color,symbol,label in [(1,'#25874b','square','实机反馈到位'),(17,'#925bc1','diamond-open','取消后确认停止')]:
    group=[g for g in all_goals if g['executed'] and g['code']==code and g.get('final_tcp_m')]
    add(label,[g['final_tcp_m'] for g in group],[html.escape(g['run']+' / '+g['label']+' | '+g['detail']) for g in group],color,symbol)
failed=[g for g in all_goals if g['executed'] and g['code'] not in [1,17]]
valid_failed=[g for g in failed if g.get('final_tcp_m')]
add('执行失败终态位置',[g['final_tcp_m'] for g in valid_failed],[html.escape(g['label']+' | '+g['detail']) for g in valid_failed],'#d13438','x',7)
xyz=np.concatenate(all_xyz) if all_xyz else np.empty((0,3));motion=[g for g in all_goals if g['executed']]
selection=[]
for g in all_goals:
    candidates=[dict(seed=int(i),total_rad=float(total),net_rad=float(net),seconds=float(seconds)) for i,total,net,seconds in re.findall(r'seed=(\d+) total=([\d.eE+-]+) net=([\d.eE+-]+) seconds=([\d.eE+-]+)',g['detail'])]
    chosen=re.search(r'selected=(\d+)',g['detail'])
    if candidates and chosen:selection.append(dict(run=g['run'],label=g['label'],executed=g['executed'],selected=int(chosen[1]),candidates=candidates))
for c in checklist.values():
    original=c
    while 'previous_result' in original and not original.get('measured_pregrasp_m'):original=original['previous_result']
    for key in ['achieved_stage','measured_pregrasp_m']:
        if key not in c and key in original:c[key]=original[key]
summary={'statuses':[r['status'] for r in reports],'checklist_total':len(checklist),'checklist_by_group':{},'checklist':list(checklist.values()),
         'motion_tasks':len(motion),'arrived_tasks':sum(g['code']==1 for g in motion),'verified_stops':sum(g['code']==17 for g in motion),'execution_failures':len(failed),
         'reference_initializations':sum(r['reference_initializations'] for r in reports),'reference_restores':sum(r['reference_restores'] for r in reports),'restored_states':restores,
         'feedback_frames':len(times),'raw_feedback_frames':len(raw_times),'measured_tcp_min_m':xyz.min(0).tolist() if len(xyz) else [],'measured_tcp_max_m':xyz.max(0).tolist() if len(xyz) else [],
         'peak_feedback_speed_rad_s':max(speeds,default=0),'peak_abs_net_twist_rad':max(map(abs,nets),default=0),'peak_total_twist_rad':max(totals,default=0),
         'servo_ack_count':len(acks),'servo_ack_latency_ms_percentiles':(np.quantile(acks,[.5,.95,.99,1])*1000).tolist() if acks else [],'servo_ack_above_60ms':sum(v>.06 for v in acks),
         'successful_stop_acknowledgements':stop_acks,'raw_peak_speed_rad_s':max(raw_speed,default=0),
         'send_gap_ms_percentiles':(np.quantile(gaps,[0,.5,.99,1])*1000).tolist() if gaps else [],'max_unacknowledged':max(pending,default=0),'independent_feedback_margins':margins,'boundaries':boundaries,
         'scope':'Discrete sampled paths and nominal-URDF TCP from real encoders. Rejection is conditional on current history, IK seeds and checked path; no continuous-volume or physical TCP certification.'}
summary['candidate_selection']={'recorded_plans':len(selection),'two_candidates':sum(len(r['candidates'])==2 for r in selection),
    'distinct_twist_candidates':sum(len(r['candidates'])==2 and abs(r['candidates'][0]['total_rad']-r['candidates'][1]['total_rad'])>1e-5 for r in selection),
    'selected_home_seed':sum(r['selected']==1 for r in selection)}
summary['verified_fault_stops']=sum(bool(g['code']==16 and g.get('after',{}).get('fresh') and g.get('after',{}).get('mode')==5) for g in failed)
summary['batch_modes']=[dict(run=run.name,status=r['status'],execute=r['execute']) for run,r in zip(a.run,reports)]
summary['latest_grid_scan']=list(latest_scans.values())
for parent in sorted({run.parent for run in a.run}):
    if (parent/'task-status.json').exists():summary['task_status']=json.loads((parent/'task-status.json').read_text())
summary['post_fault_stop_confirmations']=[]
for parent in sorted({run.parent for run in a.run}):
    for path in sorted(parent.glob('stop_recovery*/summary.json')):
        record=json.loads(path.read_text())
        if record.get('modes')==[5] and record.get('peak_speed_rad_s',1)<=.002 and record.get('frames',0)>=50:
            for stop in record.get('stop',[]):
                if stop.get('reply')=='0,{},Stop();':
                    summary['post_fault_stop_confirmations'].append(dict(source=str(path),stop_ack_ms=(stop['end_utc_ns']-stop['start_utc_ns'])/1e6,stopped_feedback=record))
for c in checklist.values():
    group=summary['checklist_by_group'].setdefault(c['group'],{});group[c['status']]=group.get(c['status'],0)+1
(a.out/'exploration-summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
with (a.out/'coverage-checklist.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['id','group','status','x_m','y_m','grasp_z_m','stage','yaw_deg','reason'])
    for c in checklist.values():w.writerow([c['id'],c['group'],c['status'],*c['target_m'],c.get('achieved_stage',''),c.get('yaw_degrees',''),c.get('last_reason','')])
with (a.out/'selection-evidence.csv').open('w',newline='') as f:
    w=csv.writer(f);w.writerow(['run','label','executed','seed','selected','total_twist_rad','net_twist_rad','duration_s'])
    for r in selection:
        for c in r['candidates']:w.writerow([r['run'],r['label'],r['executed'],c['seed'],c['seed']==r['selected'],c['total_rad'],c['net_rad'],c['seconds']])
layout=dict(scene=dict(xaxis=dict(title='X / m'),yaxis=dict(title='Y / m'),zaxis=dict(title='Z / m'),aspectmode='data'),legend=dict(orientation='h'),margin=dict(l=0,r=0,b=0,t=10),uirevision='workspace')
payload=json.dumps(dict(data=traces,layout=layout),ensure_ascii=False).replace('</','<\/')
def broken_series(x,y,ids,breaks):
    xs=[];ys=[];previous=-1
    for i in ids:
        if any(previous<b<=i for b in breaks):xs.append(None);ys.append(None)
        xs.append(float(x[i]));ys.append(float(y[i]));previous=i
    return xs,ys
origin_ns=raw_times[0] if raw_times else times[0] if times else 0
if times:
    ids=np.unique(np.linspace(0,len(times)-1,min(4000,len(times))).astype(int));minutes=(np.array(times,dtype=np.float64)-origin_ns)/60e9
    nx,ny=broken_series(minutes,np.degrees(nets),ids,twist_breaks);tx,ty=broken_series(minutes,np.degrees(totals),ids,twist_breaks)
    twist={'data':[dict(type='scatter',mode='lines',name='净扭转',x=nx,y=ny),dict(type='scatter',mode='lines',name='总扭转',x=tx,y=ty)],
           'layout':dict(xaxis=dict(title='本轮时间 / min'),yaxis=dict(title='扭转 / °'),legend=dict(orientation='h'),margin=dict(l=65,r=20,t=10,b=50))}
else:twist={'data':[],'layout':{}}
speed_plot={'data':[],'layout':{}}
if raw_times:
    raw_minutes=(np.asarray(raw_times,dtype=np.float64)-origin_ns)/60e9
    edges=np.linspace(0,len(raw_speed),min(4000,len(raw_speed))+1).astype(int)
    chosen=[begin+int(np.argmax(raw_speed[begin:end])) for begin,end in zip(edges[:-1],edges[1:]) if end>begin]
    sx,sy=broken_series(raw_minutes,raw_speed,chosen,raw_breaks)
    speed_plot={'data':[dict(type='scatter',mode='lines',name='原始反馈速度分段峰值',x=sx,y=sy)],
                'layout':dict(xaxis=dict(title='本轮时间 / min'),yaxis=dict(title='最大关节速度 / rad/s'),margin=dict(l=70,r=20,t=10,b=50),shapes=[dict(type='line',xref='paper',x0=0,x1=1,y0=.032,y1=.032,line=dict(color='#d13438',dash='dash'))],annotations=[dict(xref='paper',x=.98,y=.032,text='停止监测门限 0.032',showarrow=False,yshift=12)])}
rows=''.join('<tr><td>'+html.escape(c['id'])+'</td><td>'+html.escape(c['status'])+'</td><td>'+html.escape(c.get('last_reason',''))+'</td></tr>' for c in checklist.values())
page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>实际可达范围与连续扭转历史</title>
<style>body{font-family:system-ui;color:#253548;background:#f7f9fb;margin:20px}h1{font-size:22px}p{line-height:1.6;max-width:1100px}#range{height:70vh;min-height:500px;background:white}#twist{height:320px;background:white}button{padding:9px 16px;margin:5px}table{width:100%;border-collapse:collapse}td,th{text-align:left;padding:8px;border-bottom:1px solid #ccd4dc;overflow-wrap:anywhere}@media(max-width:600px){body{margin:10px}td{font-size:12px}}</style>
<h1>实际可达范围与连续扭转历史</h1><p>COUNTS</p><p>蓝圆点：规划通过；橙叉：规划拒绝；绿方点：反馈到位；紫色空心点：确认停止；红叉：执行失败终态。每点可查看原因和来源；点和反馈轨迹不代表整个包围盒已验证。</p>
<div id="range" aria-label="真实业务节点规划与实际反馈三维范围"></div><h2>同一参考的扭转历史</h2><div id="twist" aria-label="实际反馈的净扭转与总扭转时间线"></div><h2>原始反馈速度与故障门限</h2><div id="speed" style="height:300px;background:white" aria-label="包含失败试验的原始关节速度峰值"></div>
<p>所有坐标由真实编码器及标称 URDF 重建；没有独立 TCP 标定。边界是当前硬约束、参考历史、两组厂家 IK seed 及采样路径下的结果。初始历史重放保持非零扭转，未逐点回 Home，未自动清零。</p>
<details><summary>覆盖计划清单</summary><table><thead><tr><th>点位</th><th>结果</th><th>拒绝原因</th></tr></thead><tbody>ROWS</tbody></table></details>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script><script>const range=RANGE;Plotly.newPlot('range',range.data,range.layout,{responsive:true,displaylogo:false});const twist=TWIST;Plotly.newPlot('twist',twist.data,twist.layout,{responsive:true,displaylogo:false});const speed=SPEED_DATA;Plotly.newPlot('speed',speed.data,speed.layout,{responsive:true,displaylogo:false});</script></html>'''
counts=f"清单 {len(checklist)} 项；实机到位 {summary['arrived_tasks']} 个动作；取消后确认停止 {summary['verified_stops']}；执行失败 {summary['execution_failures']}，其中批内确认停止 {summary['verified_fault_stops']}、批外独立 Stop 确认 {len(summary['post_fault_stop_confirmations'])}。原始停止失败不会改写为成功。"
if summary.get('task_status',{}).get('status')=='BLOCKED':counts='整体任务：实机探索阻塞；其余规划通过项仍未实机到位。 '+counts
page=page.replace('COUNTS',html.escape(counts)).replace('ROWS',rows).replace('RANGE',payload).replace('TWIST',json.dumps(twist,ensure_ascii=False)).replace('SPEED_DATA',json.dumps(speed_plot,ensure_ascii=False))
(a.out/'actual-workspace.html').write_text(page)
fig=plt.figure(figsize=(12,7));ax=fig.add_subplot(111,projection='3d')
for trace in traces:
    if trace['mode']=='lines':ax.plot(trace['x'],trace['y'],trace['z'],lw=.4,c='#25874b',alpha=.35);continue
    color=trace['marker']['color'];marker='x' if trace['marker']['symbol']=='x' else 's' if color=='#25874b' else 'o'
    label={'#3478bf':'Plan pass','#c37015':'Plan reject','#25874b':'Measured arrival','#925bc1':'Verified stop','#d13438':'Execution failure'}[color]
    ax.scatter(trace['x'],trace['y'],trace['z'],s=13,c=color,marker=marker,label=label,depthshade=False)
ax.set(xlabel='X / m',ylabel='Y / m',zlabel='Z / m',title='Current business-node plans and measured execution');ax.view_init(elev=27,azim=-55)
ax.legend(loc='upper left');fig.text(.5,.02,'Discrete evidence only. Nominal-URDF TCP; no independent physical TCP metrology.',ha='center');fig.savefig(a.out/'actual-workspace.png',dpi=170);plt.close(fig)
if times:
    fig,ax=plt.subplots(figsize=(12,4));ax.plot(nx,ny,label='Signed net');ax.plot(tx,ty,label='Total absolute');ax.set(xlabel='Elapsed time / min',ylabel='Twist / degrees');ax.grid(alpha=.2);ax.legend();fig.tight_layout();fig.savefig(a.out/'continuous-twist.png',dpi=170);plt.close(fig)
print(json.dumps({k:v for k,v in summary.items() if k not in ['checklist','boundaries']},ensure_ascii=False,indent=2))
