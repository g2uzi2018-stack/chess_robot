"""Plot recorded business Action evidence; no robot connection or interpolation."""
import argparse,csv,html,json,pathlib
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser()
p.add_argument('--workspace',type=pathlib.Path,required=True)
p.add_argument('--local-planning',type=pathlib.Path,required=True)
p.add_argument('--physical',type=pathlib.Path,required=True)
p.add_argument('--out',type=pathlib.Path,required=True)
p.add_argument('--project-root',type=pathlib.Path,default=pathlib.Path(__file__).resolve().parents[2])
a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
traces=[]
def add(name,points,labels,color,symbol='circle',size=5):
    if not points:return
    xyz=np.array(points)
    traces.append(dict(type='scatter3d',mode='markers',name=name,x=xyz[:,0].tolist(),y=xyz[:,1].tolist(),z=xyz[:,2].tolist(),text=labels,
                       marker=dict(color=color,symbol=symbol,size=size),hovertemplate='%{text}<br>X %{x:.4f} m<br>Y %{y:.4f} m<br>Z %{z:.4f} m<extra>%{fullData.name}</extra>'))
def plans(path,prefix,clearance,grid_only=False):
    data=json.loads((path/'results.json').read_text())
    for passed,color,symbol in [(True,'#3478bf','circle'),(False,'#c37015','x')]:
        selected=[g for g in data['goals'] if g.get('target_m') is not None and g['plan_only'] and (not grid_only or g['label'].startswith(('grid_','edge_'))) and (g['code']==0)==passed]
        pts=[];labels=[]
        for g in selected:
            xyz=list(g['target_m'])
            if g['operation']==0:xyz[2]+=clearance
            pts.append(xyz);labels.append(html.escape(g['label']+' | '+g['detail']))
        add(prefix+('规划通过' if passed else '规划拒绝'),pts,labels,color,symbol)
    return data
workspace=plans(a.workspace,'历史全范围 · ',.25,True)
local=plans(a.local_planning,'局部 · ',.02)
physical=json.loads((a.physical/'results.json').read_text())
for code,color,symbol,label in [(1,'#25874b','square','实机到位'),(17,'#925bc1','diamond-open','取消后确认停止')]:
    group=[g for g in physical['goals'] if g['executed'] and g['code']==code and g.get('final_tcp_m') is not None]
    add(label,[g['final_tcp_m'] for g in group],[g['label']+' | '+g['detail'] for g in group],color,symbol,5)
failed=[g for g in physical['goals'] if g['executed'] and g['code'] not in (1,17) and g.get('final_tcp_m') is not None]
add('实机失败（终态反馈位置）',[g['final_tcp_m'] for g in failed],[g['label']+' | '+g['detail'] for g in failed],'#d13438','x',7)
rows=list(csv.DictReader((a.physical/'joint_feedback.csv').open()))
q=np.array([[float(r[f'q{i}_rad']) for i in range(1,7)] for r in rows])
geometry=yaml.safe_load((a.project_root/'src/chess_robot_arm/config/nova5_ag95_geometry.yaml').read_text())
origins=np.array(geometry['joint_origins']).reshape(6,4,4);tcp=np.array(geometry['tcp']).reshape(4,4)
t=np.broadcast_to(np.eye(4),(len(q),4,4)).copy()
for i in range(6):
    rot=np.broadcast_to(np.eye(4),(len(q),4,4)).copy();c=np.cos(q[:,i]);s=np.sin(q[:,i]);rot[:,0,0]=c;rot[:,0,1]=-s;rot[:,1,0]=s;rot[:,1,1]=c;t=t@origins[i]@rot
xyz=(t@tcp)[:,:3,3]
indices=np.unique(np.linspace(0,len(xyz)-1,min(2500,len(xyz))).astype(int));line=xyz[indices]
traces.append(dict(type='scatter3d',mode='lines',name='真实关节反馈重建轨迹',x=line[:,0].tolist(),y=line[:,1].tolist(),z=line[:,2].tolist(),line=dict(color='#25874b',width=3),hoverinfo='skip'))
origin=np.array(physical['local_origin_tcp_m'])
axis=lambda name:dict(title=name+' / m')
layout=dict(scene=dict(xaxis=axis('X'),yaxis=axis('Y'),zaxis=axis('Z'),aspectmode='data'),margin=dict(l=0,r=0,b=0,t=15),legend=dict(orientation='h'),uirevision='scope')
payload=json.dumps(dict(data=traces,layout=layout),ensure_ascii=False).replace('</','<\/')
summary={'workspace_passed':sum(g['code']==0 for g in workspace['goals'] if g['label'].startswith(('grid_','edge_'))),
         'local_planning_passed':sum(g['code']==0 for g in local['goals']),
         'physical_arrived':sum(g['executed'] and g['code']==1 for g in physical['goals']),
         'physical_stopped':sum(g['executed'] and g['code']==17 for g in physical['goals']),
         'physical_failed':len(failed),'feedback_frames':len(rows),'tcp_min_m':xyz.min(0).tolist(),'tcp_max_m':xyz.max(0).tolist(),
         'scope':'Planning pass is not execution. TCP from real encoders and nominal URDF; no independent TCP metrology or continuous-volume certification.'}
(a.out/'range-summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>业务节点 · 规划与实机证据</title><style>body{font-family:system-ui;margin:20px;color:#243448;background:#f6f8fa}h1{font-size:22px}p{max-width:1000px;line-height:1.6}button{padding:10px 16px;margin:4px}#chart{height:76vh;min-height:480px;background:white}@media(max-width:500px){body{margin:10px}#chart{height:70vh}}</style>
<h1>业务节点 · 规划与实机证据</h1>
<p>蓝色圆点：仅规划通过；橙色叉号：规划拒绝；绿色方点：真实反馈确认到位；紫色空心点：取消后确认停止。<span id="counts"></span></p>
<button id="global">全范围</button><button id="local">实机邻域</button><div id="chart" aria-label="可旋转的三维规划和实机点位证据"></div>
<p>全范围是 0b15c6d 的历史扫描；局部点和实机轨迹是本轮记录。点代表离散采样，空白处未测试；曲线为真实编码器经标称 URDF 重建的 TCP 轨迹。未作独立 TCP 标定，也不表示整个包围盒均可运动。桌面 z=-0.010 m；实机通道预抓高度 20 mm，默认业务 250 mm 通道的扫描仅规划。</p>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script><script>
const figure=PAYLOAD;Plotly.newPlot('chart',figure.data,figure.layout,{responsive:true,displaylogo:false});
const origin=ORIGIN;document.getElementById('counts').textContent=COUNTS;
document.getElementById('local').onclick=()=>Plotly.relayout('chart',{'scene.xaxis.range':[origin[0]-.05,origin[0]+.05],'scene.yaxis.range':[origin[1]-.05,origin[1]+.05],'scene.zaxis.range':[origin[2]-.05,origin[2]+.05]});
document.getElementById('global').onclick=()=>Plotly.relayout('chart',{'scene.xaxis.autorange':true,'scene.yaxis.autorange':true,'scene.zaxis.autorange':true});
</script></html>'''.replace('PAYLOAD',payload).replace('ORIGIN',json.dumps(origin.tolist())).replace('COUNTS',json.dumps(f"本轮到位 {summary['physical_arrived']}；停止 {summary['physical_stopped']}；实机失败 {summary['physical_failed']}。",ensure_ascii=False))
(a.out/'business-range.html').write_text(page)
plt.rcParams.update({'font.size':9})
fig=plt.figure(figsize=(13,6));axes=[fig.add_subplot(121,projection='3d'),fig.add_subplot(122,projection='3d')]
english=['Workspace plan pass','Workspace plan reject','Local plan pass','Local plan reject','Measured arrival','Verified stop','Physical failure','Measured path']
for number,ax in enumerate(axes):
    for trace in traces:
        coordinates=np.array([trace['x'],trace['y'],trace['z']]).T
        mask=np.all(abs(coordinates-origin)<.05,axis=1) if number else np.ones(len(coordinates),dtype=bool)
        coordinates=coordinates[mask]
        if not len(coordinates):continue
        if trace['mode']=='lines':ax.plot(*coordinates.T,color='#25874b',alpha=.7,lw=.7);continue
        name=trace['name'];label=('Local' if name.startswith('局部') else 'Workspace')+(' plan pass' if '规划通过' in name else ' plan reject') if '规划' in name else ('Measured arrival' if name=='实机到位' else 'Verified stop' if '停止' in name else 'Physical failure')
        marker='x' if trace['marker']['symbol']=='x' else 's' if name=='实机到位' else 'o'
        ax.scatter(*coordinates.T,s=22 if marker!='x' else 32,c=trace['marker']['color'],marker=marker,label=label,depthshade=False)
    ax.set(xlabel='X / m',ylabel='Y / m',zlabel='Z / m');ax.view_init(elev=25,azim=-55)
axes[0].set_title('Business-node planning samples / actual local execution')
axes[1].set(xlim=(origin[0]-.05,origin[0]+.05),ylim=(origin[1]-.05,origin[1]+.05),zlim=(origin[2]-.05,origin[2]+.05),title='Measured Home neighborhood (nominal TCP)')
handles,labels=axes[0].get_legend_handles_labels()
fig.legend(handles,labels,loc='lower center',bbox_to_anchor=(.5,.055),ncol=4,fontsize=8)
fig.text(.5,.015,'Discrete evidence only; plan pass is not arrival. No independent TCP calibration. Physical approach channel: 20 mm.',ha='center')
fig.subplots_adjust(bottom=.20,wspace=.16);fig.savefig(a.out/'business-range.png',dpi=180);plt.close(fig)
print(json.dumps(summary,indent=2))
