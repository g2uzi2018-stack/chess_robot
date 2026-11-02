"""Read-only, fixed-view screening benchmark; no production code or model changes."""
import argparse,json,time
from pathlib import Path
import cv2
import numpy as np

def edges(gray):
    blurred=cv2.GaussianBlur(gray,(3,3),0)
    contours,_=cv2.findContours(cv2.Canny(blurred,50,150),cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    output=[]
    for contour in contours:
        if len(contour)<8:continue
        area=cv2.contourArea(contour)
        if not 150<area<7000:continue
        (x,y),(w,h),angle=cv2.fitEllipse(contour)
        if min(w,h)<18 or max(w,h)>130 or min(w,h)/max(w,h)<.5:continue
        if not .8<area/(np.pi*w*h/4)<1.12:continue
        if area/max(1,cv2.contourArea(cv2.convexHull(contour)))<.9:continue
        if any(np.hypot(x-v[0],y-v[1])<min(w,h)*.25 for v in output):continue
        output.append((x,y,w,h))
    return output

def hough(gray):
    blurred=cv2.GaussianBlur(gray,(5,5),1)
    circles=cv2.HoughCircles(blurred,cv2.HOUGH_GRADIENT,dp=1,minDist=25,
                            param1=100,param2=25,minRadius=12,maxRadius=40)
    return [] if circles is None else circles[0].tolist()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('captures',type=Path);p.add_argument('--output',type=Path)
    args=p.parse_args();cv2.setNumThreads(1)
    anchors=np.array([[537.54,431.73],[682.20,449.88],[827.41,468.38],[955.81,490.23]])/2
    result={}
    for fn in (edges,hough):
        rows=[];timings=[]
        for folder in sorted(args.captures.glob('[0-9][0-9][0-9]')):
            image=cv2.imread(str(folder/'color.png'))
            for repeat in range(10):
                started=time.perf_counter()
                gray=cv2.cvtColor(cv2.resize(image,None,fx=.5,fy=.5,interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2GRAY)
                proposals=fn(gray)
                elapsed=(time.perf_counter()-started)*1000
                if repeat:timings.append(elapsed)
            centers=np.array([d[:2] for d in proposals]).reshape(-1,2)
            found=sum(np.any(np.linalg.norm(centers-anchor,axis=1)<15) for anchor in anchors)
            rows.append(dict(sample=folder.name,count=len(proposals),anchors_found=int(found),proposals=proposals))
        result[fn.__name__]=dict(frames=len(rows),input_note='same 12 static four-piece frames; no board, viewpoint or lighting variation',
            all_four_frames=sum(r['anchors_found']==4 for r in rows),counts=[r['count'] for r in rows],
            p50_ms=float(np.median(timings)),p95_ms=float(np.percentile(timings,95)),samples=rows)
    if args.output:args.output.write_text(json.dumps(result,indent=2))
    print(json.dumps({k:{n:v for n,v in r.items() if n!='samples'} for k,r in result.items()}))
