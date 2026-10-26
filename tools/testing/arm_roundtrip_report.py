#!/usr/bin/env python3
"""Offline report/3D plots for finite Home round-trip acceptance results."""
import argparse
import base64
import csv
import json
import pathlib


def render(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out = pathlib.Path(out)
    report = json.loads((out/'results.json').read_text())
    targets = report.get('targets',[])
    with (out/'targets.csv').open('w',newline='') as file:
        writer = csv.DictWriter(file,fieldnames=['id','x_m','y_m','z_m','status','profile',
           'plan_code','plan_detail','outbound_code','outbound_position_error_m',
           'outbound_rotation_error_rad','outbound_elapsed_s','return_code',
           'return_position_error_m','return_rotation_error_rad','return_elapsed_s',
           'pair_elapsed_s','failure_reason','fault_stage','feedback_diagnostic'])
        writer.writeheader()
        for point in targets:
            xyz = point['tcp_m']
            planning = point.get('planning',{})
            plan = planning.get('selected',planning.get('initial',{}))
            outbound,back = point.get('outbound') or {},point.get('return_home') or {}
            writer.writerow({'id':point['id'],'x_m':xyz[0],'y_m':xyz[1],'z_m':xyz[2],
               'status':point['status'],'profile':point.get('profile'),
               'plan_code':plan.get('code'),'plan_detail':plan.get('detail'),
               'outbound_code':outbound.get('code'),'outbound_position_error_m':outbound.get('position_error_m'),
               'outbound_rotation_error_rad':outbound.get('orientation_error_rad'),'outbound_elapsed_s':outbound.get('elapsed_s'),
               'return_code':back.get('code'),'return_position_error_m':back.get('position_error_m'),
               'return_rotation_error_rad':back.get('orientation_error_rad'),'return_elapsed_s':back.get('elapsed_s'),
               'pair_elapsed_s':point.get('pair_elapsed_s'),'failure_reason':point.get('failure_reason'),
               'fault_stage':point.get('fault_stage'),
               'feedback_diagnostic':json.dumps(point.get('feedback_diagnostic'),ensure_ascii=False) if point.get('feedback_diagnostic') else None})
    counts = {name:sum(p['status']==name for p in targets) for name in
              ['planning_passed','roundtrip_success','reused_success','planning_rejected','execution_failed','pre_send_feedback_failed','pending']}
    simulation = bool(report.get('simulation'))
    caption = 'SIMULATION ONLY — no physical range claim' if simulation else report.get('status','UNKNOWN')
    figure = plt.figure(figsize=(14,6),layout='constrained')
    categories = {'planning_passed':('tab:blue','o','Planning passed only'),
                  'roundtrip_success':('tab:green','o','Measured outbound + Home success'),
                  'reused_success':('seagreen','s','Reused prior measured success'),
                  'planning_rejected':('darkorange','x','Planning rejected'),
                  'execution_failed':('crimson','X','Execution failed'),
                  'pre_send_feedback_failed':('purple','D','Feedback fault before new motion submission'),
                  'pending':('gray','.','Pending / not executed')}
    home = report.get('home',{}).get('tcp_m')
    for number,(elevation,azimuth) in enumerate([(23,-57),(75,-90)],1):
        ax = figure.add_subplot(1,2,number,projection='3d' if number==1 else None)
        for status,(color,marker,label) in categories.items():
            points = [p['tcp_m'] for p in targets if p['status']==status]
            if points:
                coords = points if number==1 else [p[:2] for p in points]
                ax.scatter(*zip(*coords),c=color,marker=marker,s=45,label=f'{label} ({len(points)})',
                           **({'depthshade':False} if number==1 else {}))
        if home:
            ax.scatter(*(home if number==1 else home[:2]),c='black',marker='*',s=130,label='Formal Home nominal TCP')
        if not targets:
            pending = report.get('manifest',{}).get('coarse_points',[])
            if pending:
                coords=[p['tcp_m'] if number==1 else p['tcp_m'][:2] for p in pending]
                ax.scatter(*zip(*coords),c='gray',marker='.',label='Manifest only — not planned/executed')
        ax.set(xlabel='base X / m',ylabel='base Y / m')
        if number==1:
            ax.set_zlabel('TCP Z / m')
            ax.view_init(elevation,azimuth)
            ax.set_box_aspect((1,1,1))
        else:
            ax.set_title('Top view (XY)')
            ax.set_aspect('equal',adjustable='box')
            ax.grid(True,alpha=.3)
        if number==1:
            ax.legend(fontsize=7,loc='upper left')
    figure.suptitle('Home → exact TCP → Home | '+caption)
    figure.savefig(out/'range.png',dpi=180)
    plt.close(figure)
    lines = ['# Home → TCP target → Home acceptance','',f'Status: **{report.get("status")}**','',
             'Simulation only; no physical range claim.' if simulation else
             'Only records with two measured successful legs count as a target success.',
             '',f'Counts: {json.dumps(counts)}','',
             '![3D tested target status](range.png)','',
             'Coordinates are exact TCP endpoints in base_link. Formal Home is the existing YAML; '
             'its nominal TCP marker is not evidence that the live robot is currently at Home. '
             'Planning-only and unexecuted points do not establish actual reachability.','',
             '| Direction | Farthest measured round trip / m | Farther rejection / m | Remaining bracket / m | Status |',
             '|---|---:|---:|---:|---|']
    for name,value in report.get('boundaries',{}).items():
        lines.append(f'| {name} | {value["farthest_measured_roundtrip_m"]} | '
                     f'{value["nearest_farther_planning_rejection_m"]} | {value["remaining_bracket_width_m"]} | {value["status"]} |')
    lines += ['', 'No global/continuous workspace guarantee is inferred from these sparse samples. '
              'All-pass directions remain explicitly unbracketed.','',
              'Full coordinates, both leg outcomes, measured errors, durations and failure reasons: '
              '`targets.csv`, `results.json`, `actions.json`, per-action trajectory files and `audit/feedback.bin`.']
    if report.get('error'):
        lines += ['', 'Blocking evidence: '+report['error']]
    if report.get('status','').startswith('EXECUTION_FAULT'):
        lines += ['', 'The batch stopped at the first execution fault. No automatic Home return or replay was attempted.']
    if report.get('status','').startswith('PRE_SEND_FEEDBACK_FAULT'):
        lines += ['', 'A fresh-feedback precondition failed before the new action was submitted. '
                  'No new motion was sent; the reported stop confirmation remains explicit. No forced Home or replay.']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    pixels=base64.b64encode((out/'range.png').read_bytes()).decode('ascii')
    (out/'range.html').write_text('<!doctype html><meta charset="utf-8"><title>Home round-trip 3D status</title>'
                                '<h1>Home round-trip 3D status</h1><p>'+caption+'</p>'
                                '<img src="data:image/png;base64,'+pixels+'" style="max-width:100%">'
                                '<p>See REPORT.md and targets.csv for evidence.</p>')


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=pathlib.Path)
    render(parser.parse_args().run)
