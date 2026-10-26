#!/usr/bin/env python3
"""Offline-only receipt-gap analysis. No ROS import, socket, controller or motion API.

Binary audit: little-endian signed host UTC ns + original 1440-byte V4 frame.
Optional timing CSV is copied as evidence and summarized per *_ns duration;
timestamps remain separate clock domains. No controller-to-ROS conversion.
"""
import argparse
import collections
import bisect
import csv
import decimal
import hashlib
import ipaddress
import json
import math
import pathlib
import re
import statistics
import struct
import subprocess

FRAME_SIZE=1440
RECORD_SIZE=1448
MAGIC=0x0123456789ABCDEF


def audit_rows(path):
    with pathlib.Path(path).open('rb') as stream:
        index=0
        while True:
            raw=stream.read(RECORD_SIZE)
            if not raw:return
            if len(raw)!=RECORD_SIZE:raise ValueError(f'{path}: truncated record {index}: {len(raw)} bytes')
            host=struct.unpack_from('<q',raw)[0]
            frame=memoryview(raw)[8:]
            if struct.unpack_from('<H',frame)[0]!=FRAME_SIZE or struct.unpack_from('<Q',frame,48)[0]!=MAGIC:
                raise ValueError(f'{path}: invalid V4 framing at record {index}')
            q=struct.unpack_from('<6d',frame,432)
            qd=struct.unpack_from('<6d',frame,480)
            if not all(math.isfinite(x) for x in q+qd):raise ValueError('nonfinite joint feedback')
            yield {'frame_index':index,'host_utc_ns':host,'controller_stamp_raw':struct.unpack_from('<Q',frame,32)[0],
                   'mode':struct.unpack_from('<Q',frame,24)[0],
                   'q_rad':[math.radians(x) for x in q],'qd_rad_s':[math.radians(x) for x in qd]}
            index+=1


def analyze(path,threshold):
    previous=None; intervals=[]; gaps=[]; modes=collections.Counter();duplicates=regressions=count=0
    for row in audit_rows(path):
        count+=1;modes[row['mode']]+=1
        if previous:
            delta=row['host_utc_ns']-previous['host_utc_ns']; intervals.append(delta)
            controller_delta=row['controller_stamp_raw']-previous['controller_stamp_raw']
            duplicates+=controller_delta==0;regressions+=controller_delta<0
            if delta>threshold*1e9 or delta<0:
                gaps.append({'before_frame':previous['frame_index'],'after_frame':row['frame_index'],
                    'before_host_utc_ns':previous['host_utc_ns'],'after_host_utc_ns':row['host_utc_ns'],
                    'receipt_gap_s':delta/1e9,'controller_delta_raw':controller_delta,
                    'before_mode':previous['mode'],'after_mode':row['mode'],
                    'max_q_delta_rad':max(abs(a-b) for a,b in zip(row['q_rad'],previous['q_rad'])),
                    'after_max_qd_rad_s':max(map(abs,row['qd_rad_s']))})
        previous=row
    return {'path':str(pathlib.Path(path).resolve()),'sha256':hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest(),
        'frames':count,'mode_counts':dict(modes),'threshold_s':threshold,
        'max_host_receipt_gap_s':max(intervals,default=0)/1e9,
        'median_host_receipt_gap_s':statistics.median(intervals)/1e9 if intervals else None,
        'duplicate_controller_stamps':duplicates,'regressing_controller_stamps':regressions,
        'gap_count':len(gaps),'gaps':gaps,
        'interpretation':'Host audit UTC timestamps denote complete-frame consumption near receive time; not kernel packet arrival or controller production. Raw controller deltas are not ROS timestamps.'}


def trace_summary(path):
    rows=list(csv.DictReader(pathlib.Path(path).open()))
    result={'path':str(pathlib.Path(path).resolve()),'sha256':hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest(),
        'rows':len(rows),'columns':list(rows[0]) if rows else [],'duration_summary':{}}
    pairs={'read':('read_begin_steady_ns','complete_steady_ns'),
        'audit':('audit_begin_steady_ns','audit_end_steady_ns'),
        'decode':('audit_end_steady_ns','decode_end_steady_ns'),
        'dispatch':('dispatch_begin_steady_ns','dispatch_end_steady_ns'),
        'backoff':('backoff_begin_steady_ns','backoff_end_steady_ns')}
    derived=[];previous=None
    for row in rows:
        r={key:int(row[key]) for key in ['frame','host_utc_ns','read_begin_steady_ns','complete_steady_ns','status'] if row.get(key)}
        for name,(start,end) in pairs.items():
            a,b=int(row.get(start) or 0),int(row.get(end) or 0)
            r[name+'_duration_ns']=b-a if a>0 and b>=a else None
        if r.get('complete_steady_ns',0)>0:
            r['complete_gap_ns']=r['complete_steady_ns']-previous['complete_steady_ns'] if previous else None
            r['utc_gap_ns']=r['host_utc_ns']-previous['host_utc_ns'] if previous else None
            r['utc_minus_steady_gap_ns']=(r['utc_gap_ns']-r['complete_gap_ns']) if previous else None
            previous=r
        derived.append(r)
    for name in [n+'_duration_ns' for n in pairs]+['complete_gap_ns','utc_minus_steady_gap_ns']:
        values=[r[name] for r in derived if r.get(name) is not None]
        if values:
            ordered=sorted(values)
            result['duration_summary'][name]={'max_ns':max(values),'median_ns':statistics.median(values),
                'p99_ns':ordered[min(len(ordered)-1,math.ceil(.99*len(ordered))-1)],'sum_ns':sum(values)}
    result['status_counts']=dict(collections.Counter(row.get('status') for row in rows))
    result['frame_order_regressions']=sum(int(b['frame'])<=int(a['frame']) for a,b in zip(rows,rows[1:]))
    starts=[int(r.get('read_begin_steady_ns') or 0) for r in rows if int(r.get('read_begin_steady_ns') or 0)>0]
    ends=[int(r.get('dispatch_end_steady_ns') or r.get('complete_steady_ns') or 0) for r in rows]
    span=max(ends,default=0)-min(starts,default=0)
    result['recorded_span_ns']=span
    for name,stats in result['duration_summary'].items():
        if name.endswith('_duration_ns') and span>0:stats['fraction_of_recorded_span']=stats['sum_ns']/span
    result['meta_candidates']={}
    for candidate in [pathlib.Path(path).with_suffix('.meta.json'),pathlib.Path(path).with_name('feedback_timing_meta.json')]:
        if candidate.exists():result['meta_candidates'][str(candidate)]=json.loads(candidate.read_text())
    text_meta=pathlib.Path(path).with_name('feedback_timing_meta.txt')
    if text_meta.exists():
        values=dict(line.split('=',1) for line in text_meta.read_text().splitlines() if '=' in line)
        result['meta_candidates'][str(text_meta)]=values
        result['meta_rows_match']=int(values.get('rows',-1))==len(rows)
        result['trace_dropped']=int(values.get('dropped',-1))
    result['_frames']=derived
    # Also support explicit named duration columns without crossing clocks.
    for name in result['columns']:
        if not any(token in name for token in ('duration','elapsed','_wait_','_cost_')):continue
        if not name.endswith('_ns'):continue
        try:values=[int(row[name]) for row in rows if row.get(name)]
        except ValueError:continue
        if values:result['duration_summary'][name]={'max_ns':max(values),'median_ns':statistics.median(values)}
    return result


def pcap_summary(path,source,port):
    source=str(ipaddress.IPv4Address(source))
    if not 1<=port<=65535:raise ValueError('feedback source port must be 1..65535')
    command=['tcpdump','-tt','-nn','-S','-r',str(path),
             f'src host {source} and src port {port} and tcp']
    read=subprocess.run(command,text=True,capture_output=True,check=True)
    packets=[];flows=collections.Counter();unparsed=0;seq_high={};overlaps=0;forward_gaps=forward_bytes=0
    for line in read.stdout.splitlines():
        match=re.match(r'^(\d+\.\d+) IP (\S+) > (\S+): .*\blength (\d+)\s*$',line)
        if not match:
            unparsed+=1;continue
        if int(match[4])<=0:continue
        ns=int(decimal.Decimal(match[1])*1_000_000_000)
        flow=match[2]+' > '+match[3];flows[flow]+=1
        seq=re.search(r'\bseq (\d+):(\d+)',line)
        overlap=False
        if seq:
            a,b=map(int,seq.groups());overlap=a<seq_high.get(flow,a)
            if flow in seq_high and a>seq_high[flow]:forward_gaps+=1;forward_bytes+=a-seq_high[flow]
            seq_high[flow]=max(seq_high.get(flow,b),b)
            overlaps+=overlap
        packets.append({'host_capture_utc_ns':ns,'payload_bytes':int(match[4]),'flow':flow,
                        'seq_start':int(seq[1]) if seq else None,'seq_end':int(seq[2]) if seq else None,
                        'sequence_overlap_heuristic':overlap})
    packets.sort(key=lambda p:p['host_capture_utc_ns'])
    gaps=[b['host_capture_utc_ns']-a['host_capture_utc_ns'] for a,b in zip(packets,packets[1:])]
    return {'path':str(path.resolve()),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'reader_argv':command,'reader_stderr':read.stderr,'payload_packets':len(packets),'flows':dict(flows),
        'unparsed_text_lines':unparsed,'max_payload_packet_gap_ns':max(gaps,default=None),
        'median_payload_packet_gap_ns':statistics.median(gaps) if gaps else None,
        'sequence_overlap_heuristic_count':overlaps,
        'sequence_forward_gap_heuristic_count':forward_gaps,'sequence_forward_gap_bytes':forward_bytes,
        'limitations':'tcpdump software capture timestamps, not controller production. Sequence overlap is a hint, not TCP reassembly/retransmission proof. Capture drop counters must be reviewed separately. Multiple flows require separate interpretation.',
        '_packets':packets}


def compare_gaps(trace,pcap,threshold):
    frames=[r for r in trace['_frames'] if r.get('complete_steady_ns',0)>0]
    packets=pcap['_packets'] if pcap else []
    times=[p['host_capture_utc_ns'] for p in packets]
    comparisons=[]
    top_after={after['frame'] for before,after in sorted(zip(frames,frames[1:]),
        key=lambda pair:pair[1]['complete_steady_ns']-pair[0]['complete_steady_ns'],reverse=True)[:10]}
    for before,after in zip(frames,frames[1:]):
        gap=after['complete_steady_ns']-before['complete_steady_ns']
        if gap<=threshold*1e9 and after['frame'] not in top_after:continue
        row={'before_frame':before['frame'],'after_frame':after['frame'],
             'complete_gap_ns':gap,'over_analysis_threshold':gap>threshold*1e9,
             'before_host_utc_ns':before['host_utc_ns'],
             'after_host_utc_ns':after['host_utc_ns'],
             'previous_complete_to_next_read_begin_ns':after['read_begin_steady_ns']-before['complete_steady_ns'],
             **{'previous_'+key:before.get(key) for key in ['audit_duration_ns','decode_duration_ns','dispatch_duration_ns','backoff_duration_ns']},
             **{key:after.get(key) for key in ['read_duration_ns','audit_duration_ns','decode_duration_ns',
                                             'dispatch_duration_ns','backoff_duration_ns']}}
        if packets:
            lo=bisect.bisect_left(times,before['host_utc_ns']);hi=bisect.bisect_right(times,after['host_utc_ns'])
            window=times[max(0,lo-1):min(len(times),hi+1)]
            gaps=[b-a for a,b in zip(window,window[1:])]
            row.update(kernel_payload_packets_in_window=hi-lo,
                surrounding_kernel_max_packet_gap_ns=max(gaps,default=None),
                kernel_payload_bytes_in_window=sum(p['payload_bytes'] for p in packets[lo:hi]),
                alignment='UTC near complete-frame receipt vs software capture; no exact frame/TCP reassembly')
        comparisons.append(row)
    return comparisons


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit',type=pathlib.Path,action='append',required=True)
    parser.add_argument('--threshold-s',type=float,default=.1)
    parser.add_argument('--timing-csv',type=pathlib.Path)
    parser.add_argument('--pcap',type=pathlib.Path)
    parser.add_argument('--capture-log',type=pathlib.Path,help='Closed capture stderr/meta log containing actual capture-drop counters')
    parser.add_argument('--feedback-source',help='Actual controller IPv4; pcap input only, no connection')
    parser.add_argument('--feedback-port',type=int,default=30004)
    parser.add_argument('--out',type=pathlib.Path,required=True)
    args=parser.parse_args()
    if not math.isfinite(args.threshold_s) or args.threshold_s<=0:raise ValueError('positive analysis threshold required')
    args.out.mkdir(parents=True,exist_ok=False)
    reports=[analyze(path,args.threshold_s) for path in args.audit]
    summary={'schema':'chess_robot.offline_feedback_gap/v1','offline_only':True,'audits':reports,
        'cross_file_epochs_not_implicitly_joined':True,'raw_files_unchanged':True,
        'root_cause':'UNRESOLVED: cannot distinguish controller production, link, host scheduling, audit I/O or parser from raw receipt timestamps alone'}
    trace=trace_summary(args.timing_csv) if args.timing_csv else None
    if args.pcap and not args.feedback_source:raise ValueError('--pcap needs explicit --feedback-source')
    pcap=pcap_summary(args.pcap,args.feedback_source,args.feedback_port) if args.pcap else None
    def save_rows(name,rows):
        if not rows:return
        with (args.out/name).open('w',newline='') as f:
            fields=list(dict.fromkeys(key for row in rows for key in row))
            w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
    if trace:
        save_rows('timing-frames.csv',trace['_frames'])
        comparisons=compare_gaps(trace,pcap,args.threshold_s)
        save_rows('timing-gaps.csv',comparisons)
        summary['timing_gap_comparisons']=comparisons
        summary['timing_trace']={key:value for key,value in trace.items() if not key.startswith('_')}
    if pcap:
        save_rows('capture-payload-packets.csv',pcap['_packets'])
        summary['capture']={key:value for key,value in pcap.items() if not key.startswith('_')}
        if args.capture_log:
            text=args.capture_log.read_text()
            summary['capture']['capture_log']={'path':str(args.capture_log.resolve()),
                'sha256':hashlib.sha256(args.capture_log.read_bytes()).hexdigest(),
                'counters':{name:int(value) for value,name in re.findall(
                    r'(\d+) packets (captured|received by filter|dropped by kernel)',text)},
                'original_text':text}
    (args.out/'analysis.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    gaps=[dict(audit_path=r['path'],**gap) for r in reports for gap in r['gaps']]
    fields=['audit_path','before_frame','after_frame','before_host_utc_ns','after_host_utc_ns','receipt_gap_s',
            'controller_delta_raw','before_mode','after_mode','max_q_delta_rad','after_max_qd_rad_s']
    with (args.out/'gaps.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(gaps)
    for report in reports:print(json.dumps({key:value for key,value in report.items() if key!='gaps'},ensure_ascii=False))


if __name__=='__main__':main()
