"""Analyze generated model states. No robot feedback or control."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser()
p.add_argument('run',type=Path)
a=p.parse_args()
with (a.run/'synthetic_motion.csv').open() as f:
    rows=list(csv.DictReader(f))
t=np.array([float(r['synthetic_time_s']) for r in rows])
clock=np.array([int(r['record_ros_time_ns']) for r in rows],dtype=np.int64)
wall=(clock-clock[0])*1e-9
q=np.array([[float(r[f'q_deg{j}']) for j in range(1,7)] for r in rows])
qd=np.array([[float(r[f'qd_deg_s{j}']) for j in range(1,7)] for r in rows])
xyz=np.array([[float(r[f'flange_{c}_m']) for c in 'xyz'] for r in rows])
sv=np.array([float(r['jacobian_min_sv_scaled']) for r in rows])
cond=np.array([float(r['jacobian_condition_scaled']) for r in rows])
moving=(t>0)&(t<20)
gradient=np.gradient(q[moving],t[moving],axis=0,edge_order=2)
dt=np.diff(clock)*1e-6
summary={
    'scope':'SYNTHETIC MODEL STATES ONLY; no actual robot feedback',
    'frames':len(rows),'record_duration_s':float(wall[-1]),
    'dt_median_ms':float(np.median(dt)),'dt_p95_ms':float(np.percentile(dt,95)),
    'dt_max_ms':float(np.max(dt)),'sample_rate_from_median_hz':float(1000/np.median(dt)),
    'q_min_deg':q.min(axis=0).tolist(),'q_max_deg':q.max(axis=0).tolist(),
    'analytic_peak_abs_qd_deg_s':np.max(np.abs(qd),axis=0).tolist(),
    'difference_q_derivative_vs_analytic_qd_peak_deg_s':float(np.max(np.abs(gradient-qd[moving]))),
    'scaled_jacobian_min_sv':float(sv.min()),'scaled_jacobian_max_condition':float(cond.max()),
    'closed_curve_start_end_position_error_m':float(np.linalg.norm(xyz[-1]-xyz[0])),
    'closed_curve_start_end_joint_error_deg':float(np.max(np.abs(q[-1]-q[0]))),
    'actual_robot_frames':0,'controller_commands_sent':0,
    'limits_note':'URDF position bounds and Jacobian screening only; zero velocity/effort placeholders were not converted into physical limits.',
    'timing_note':'record_ros_time_ns is generated state record time, not controller feedback reception time; video is 10 fps and RViz display was about 5-7 fps.',
}
(a.run/'simulation_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
fig=plt.figure(figsize=(13,9),constrained_layout=True)
ax=fig.add_subplot(221)
for j in range(6):ax.plot(wall,q[:,j],label=f'joint{j+1}')
ax.set(title='Generated joint positions',xlabel='Record elapsed (s)',ylabel='Position (deg)');ax.grid(alpha=.25);ax.legend(ncol=3,fontsize=8)
ax=fig.add_subplot(222)
for j in range(6):ax.plot(wall,qd[:,j],label=f'joint{j+1}')
ax.set(title='Analytic velocities of generated curve',xlabel='Record elapsed (s)',ylabel='Velocity (deg/s)');ax.grid(alpha=.25);ax.legend(ncol=3,fontsize=8)
ax=fig.add_subplot(223,projection='3d')
ax.plot(xyz[moving,0],xyz[moving,1],xyz[moving,2],color='teal')
ax.scatter(*xyz[0],color='crimson',label='same start/end')
ax.set(title='URDF Link6 curve; not measured TCP / not MovL',xlabel='Base X (m)',ylabel='Base Y (m)',zlabel='Base Z (m)');ax.legend(fontsize=8)
ax=fig.add_subplot(224)
ax.plot(wall[1:],dt,color='slateblue',linewidth=.6)
ax.axhline(20,color='crimson',ls='--',label='nominal 50 Hz = 20 ms')
ax.set(title='Generated-state record intervals',xlabel='Record elapsed (s)',ylabel='ROS clock interval (ms)');ax.grid(alpha=.25);ax.legend(fontsize=8)
fig.suptitle('RViz / URDF simulation only - no hardware, dynamics or execution accuracy validation',fontsize=13)
fig.savefig(a.run/'rviz_simulation_data.png',dpi=160)
print(json.dumps(summary,indent=2))
