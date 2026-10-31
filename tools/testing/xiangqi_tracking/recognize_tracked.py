"""Offline segmentation + visible surface position experiment; no robot commands."""
import argparse, hashlib, json, sys, time, xml.etree.ElementTree as ET
from pathlib import Path
import cv2, numpy as np, yaml

ROOT=Path('/home/kuang/workspace/chess_robot')
TRAIN=Path('/home/kuang/workspace/xiangqi-yolo-training/20261031-yolo26s-seg-v1')
WEIGHTS=TRAIN/'runs/train/weights/best.pt'
CALIB=ROOT/'docs/validation/handeye-20261030/flange_T_camera_candidate.yaml'
URDF=ROOT/'src/chess_robot_description/urdf/nova5_with_ag95.urdf'

def iou(a,b):
    lo=np.maximum(a[:2],b[:2]); hi=np.minimum(a[2:],b[2:]); inter=np.maximum(hi-lo,0).prod()
    return float(inter/max(1e-9,np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2])-inter))

def surface_position(mask,depth,K):
    """Estimate depth at mask centroid from robust visible-surface plane, in metres."""
    # A zero rim preserves distanceTransform values inside a complete mask.
    # Keep image-edge masks on the original path: their boundary semantics differ.
    offset_x=offset_y=0
    bx,by,bw,bh=cv2.boundingRect(mask)
    if bw and bh and bx>0 and by>0 and bx+bw<mask.shape[1] and by+bh<mask.shape[0]:
        offset_x,offset_y=bx-1,by-1
        region=np.s_[offset_y:by+bh+1,offset_x:bx+bw+1]
        mask=mask[region];depth=depth[region]
    mm=cv2.moments(mask,True)
    if mm['m00']<100:return dict(valid=False,reason='mask too small')
    uv=np.array([mm['m10']/mm['m00']+offset_x,mm['m01']/mm['m00']+offset_y])
    distance=cv2.distanceTransform(mask,cv2.DIST_L2,5)
    inner=distance>max(3.,.25*float(distance.max()))
    valid=inner & np.isfinite(depth) & (depth>.02) & (depth<.6)
    n=int(valid.sum()); fraction=n/max(1,int(inner.sum()))
    out=dict(center_pixel=uv.tolist(),valid_depth_pixels=n,inner_valid_depth_fraction=fraction,valid=False)
    if n<50 or fraction<.5:return dict(out,reason='insufficient inner-mask depth')
    y,x=np.nonzero(valid);x=x+offset_x;y=y+offset_y; z=depth[valid].astype(float)
    median=float(np.median(z)); mad=float(np.median(np.abs(z-median)))
    choose=np.abs(z-median)<max(.002,4.5*mad)
    rays=np.linalg.inv(K)@np.stack([x,y,np.ones(n)])
    points=(rays*z).T[choose]
    if len(points)<50:return dict(out,reason='insufficient robust depth')
    # Deterministic subsampling bounds runtime without selecting only one side of the mask.
    points=points[::max(1,len(points)//3000)]
    kept=points
    for _ in range(3):
        mean=kept.mean(0);centered=kept-mean;_,vectors=np.linalg.eigh(centered.T@centered);normal=vectors[:,0]
        error=np.abs((points-mean)@normal); retained=error<max(.001,3*float(np.median(error)))
        if retained.sum()<50:return dict(out,reason='surface plane rejected')
        kept=points[retained]
    residual=float(np.sqrt(np.mean(((kept-mean)@normal)**2)))
    center_ray=np.linalg.inv(K)@np.r_[uv,1.]
    denominator=float(normal@center_ray)
    if abs(denominator)<.2 or residual>.002:return dict(out,reason='surface plane unreliable',plane_rms_m=residual)
    zcenter=float(normal@mean/denominator)
    if not .02<zcenter<.6:return dict(out,reason='invalid plane intersection')
    if normal[2]>0:normal=-normal
    return dict(out,valid=True,camera_surface_center_m=(center_ray*zcenter).tolist(),camera_surface_normal=normal.tolist(),plane_rms_m=residual,depth_median_m=median,point_semantics='visible top surface center; not cylinder center or grasp TCP')

class Experiment:
    def __init__(self,imgsz=640,half=False,geometry_only=False):
        import torch,ultralytics
        from ultralytics import YOLO
        torch.set_num_threads(2); cv2.setNumThreads(1)
        self.model=None if geometry_only else YOLO(str(WEIGHTS))
        if self.model is not None:assert self.model.task=='segment'
        self.imgsz=imgsz
        self.half=half
        calibration=yaml.safe_load(CALIB.read_text()); self.X=np.array(calibration['flange_T_camera'])
        assert calibration['parent_frame']=='Link6' and calibration['child_frame']=='d405_color_optical_frame'
        self.robot=ET.parse(URDF).getroot()
        sys.path.insert(0,str(ROOT/'tools/description'));from rigid_geometry import fk
        self.fk=fk
        self.identity=dict(weights=str(WEIGHTS),weights_sha256=hashlib.sha256(WEIGHTS.read_bytes()).hexdigest(),calibration=str(CALIB),calibration_sha256=hashlib.sha256(CALIB.read_bytes()).hexdigest(),urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),ultralytics=ultralytics.__version__,torch=torch.__version__,imgsz=imgsz)

    def detect(self,color,roi=None):
        assert self.model is not None
        start=time.monotonic()
        x1,y1,x2,y2=roi if roi is not None else (0,0,color.shape[1],color.shape[0])
        result=self.model.predict(color[y1:y2,x1:x2],device=0,imgsz=self.imgsz,conf=.25,verbose=False,retina_masks=True,half=self.half)[0]
        predictions=result.boxes.data.cpu().numpy()
        masks=(result.masks.data.cpu().numpy()>.5).astype(np.uint8) if result.masks is not None else None
        compute=dict(predictor_device=str(self.model.predictor.device),model_parameter_device=str(next(self.model.model.parameters()).device),boxes_device=str(result.boxes.data.device),masks_device=str(result.masks.data.device) if result.masks is not None else None,fp16=bool(self.model.predictor.model.fp16))
        return dict(predictions=predictions,masks=masks,names=dict(result.names),compute=compute,inference_speed_ms=result.speed,detector_s=time.monotonic()-start,identity=self.identity,roi_xyxy=[x1,y1,x2,y2],image_size=[color.shape[1],color.shape[0]])

    def run(self,color,depth,metadata,roi=None,draw=True,detection=None):
        if detection is None:detection=self.detect(color,roi)
        start=time.monotonic(); K=np.array(metadata['K']).reshape(3,3)
        assert color.shape[:2]==depth.shape and depth.dtype==np.float32
        joint=metadata['joint']; assert all('joint'+str(i) in joint['name'] for i in range(1,7))
        poses=self.fk(self.robot,dict(zip(joint['name'],joint['position'])))
        transform=np.array(poses['Link6'])@self.X
        # Verify the offline FK matches the running arm's model TCP before claiming base coordinates.
        position=metadata['arm']['tcp']['pose']['position']; actual=np.array([position[k] for k in ('x','y','z')])
        fk_error=float(np.linalg.norm(np.array(poses['grasp_link'])[:3,3]-actual))
        base_valid=fk_error<.0001 and metadata['arm']['tcp']['header']['frame_id']=='base_link'
        x1,y1,x2,y2=roi if roi is not None else (0,0,color.shape[1],color.shape[0])
        assert detection['roi_xyxy']==[x1,y1,x2,y2] and detection['image_size']==[color.shape[1],color.shape[0]]
        predictions=detection['predictions'];names=detection['names']
        boxes=predictions[:,:4]+[x1,y1,x1,y1];conf=predictions[:,-2];classes=predictions[:,-1].astype(int)
        masks=detection['masks']
        roi_K=K.copy();roi_K[0,2]-=x1;roi_K[1,2]-=y1
        groups=[]
        for i in np.argsort(-conf):
            existing=next((g for g in groups if iou(boxes[i],boxes[g[0]])>.65),None)
            if existing is None:groups.append([int(i)])
            else:existing.append(int(i))
        overlay=color.copy() if draw else None; detections=[]
        for number,group in enumerate(groups):
            i=group[0];box=boxes[i];name=names[int(classes[i])]
            record=dict(id=number,class_id=int(classes[i]),class_name=name,confidence=float(conf[i]),box_xyxy=box.tolist(),alternatives=[dict(name=names[int(classes[k])],confidence=float(conf[k])) for k in group[1:]],touches_image_edge=bool(box[0]<2 or box[1]<2 or box[2]>color.shape[1]-2 or box[3]>color.shape[0]-2),class_verified=False)
            if masks is None:record.update(valid=False,reason='no instance mask')
            else:
                mask=(masks[i]>.5).astype(np.uint8)
                assert mask.shape==depth[y1:y2,x1:x2].shape
                record.update(surface_position(mask,depth[y1:y2,x1:x2],roi_K))
                if 'center_pixel' in record:record['center_pixel']=(np.array(record['center_pixel'])+[x1,y1]).tolist()
                contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    outline=max(contours,key=cv2.contourArea);outline=cv2.approxPolyDP(outline,1.5,True)
                    record['mask_outline_xy']=(outline.reshape(-1,2)+[x1,y1]).tolist()
                if draw:cv2.drawContours(overlay,contours,-1,(0,230,0),2,offset=(x1,y1))
            if record.get('valid') and base_valid:
                record['base_surface_center_m']=(transform@np.r_[record['camera_surface_center_m'],1])[:3].tolist()
                record['base_surface_normal']=(transform[:3,:3]@record['camera_surface_normal']).tolist()
            elif record.get('valid'):record.update(valid=False,reason='arm model FK mismatch')
            record['grasp_ready']=False
            record['touches_processing_edge']=bool(box[0]<x1+2 or box[1]<y1+2 or box[2]>x2-2 or box[3]>y2-2)
            record['complete_center_target']=bool(record.get('valid') and not record['touches_image_edge'] and not record['touches_processing_edge'])
            x,y=map(int,box[:2])
            if draw:cv2.putText(overlay,f"#{number} {name} {conf[i]:.2f}",(max(2,x),max(65,y-8)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,230,0),2)
            if draw and record.get('valid'):
                u,v=map(lambda x:int(round(x)),record['center_pixel']);cv2.drawMarker(overlay,(u,v),(0,255,255),cv2.MARKER_CROSS,22,2)
                xyz=np.array(record['base_surface_center_m'])*1000
                cv2.putText(overlay,f"base mm {xyz[0]:.1f},{xyz[1]:.1f},{xyz[2]:.1f}",(max(2,x),min(color.shape[0]-12,int(box[3])+20)),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,255,255),1)
            detections.append(record)
        elapsed=time.monotonic()-start
        if draw:
            cv2.rectangle(overlay,(0,0),(color.shape[1],48),(0,0,0),-1)
            cv2.putText(overlay,f'EXPERIMENT / {color.shape[1]}x{color.shape[0]} / YOLO imgsz={self.imgsz} / {len(groups)} objects / {elapsed*1000:.0f}ms',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
            cv2.putText(overlay,'Classes unverified | surface XYZ only | no grasp commands',(8,40),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,255,255),1)
        record=dict(stamp_ns=metadata['stamp_ns'],source_joint_position_rad=joint['position'],frame_id='base_link',identity=detection['identity'],compute=detection['compute'],image_size=[color.shape[1],color.shape[0]],inference_roi_xyxy=[x1,y1,x2,y2],detections=detections,raw_detection_count=len(boxes),deduplicated_count=len(groups),fk_tcp_error_m=fk_error,base_transform_valid=base_valid,inference_speed_ms=detection['inference_speed_ms'],geometry_ms=elapsed*1000,detector_ms=detection['detector_s']*1000,processing_s=elapsed+detection['detector_s'],full_6d_pose_estimated=False,robot_commands=0)
        return record,overlay

def main():
    p=argparse.ArgumentParser();p.add_argument('capture');p.add_argument('output');p.add_argument('--imgsz',type=int,default=640);args=p.parse_args()
    out=Path(args.output);out.mkdir(exist_ok=False,parents=True);experiment=Experiment(args.imgsz);reports=[]
    for directory in sorted(Path(args.capture).glob('[0-9][0-9][0-9]')):
        color=cv2.imread(str(directory/'color.png'));depth=np.load(directory/'depth_m.npy');metadata=json.loads((directory/'metadata.json').read_text())
        record,image=experiment.run(color,depth,metadata);record['sample']=directory.name;reports.append(record)
        (out/(directory.name+'.json')).write_text(json.dumps(record,indent=2,allow_nan=False));cv2.imwrite(str(out/(directory.name+'.png')),image)
        print(json.dumps(dict(sample=directory.name,detections=[{k:d[k] for k in ('class_name','confidence','valid','touches_image_edge','alternatives','base_surface_center_m') if k in d} for d in record['detections']],speed=record['inference_speed_ms']),allow_nan=False),flush=True)
    (out/'summary.json').write_text(json.dumps(reports,indent=2,allow_nan=False))
if __name__=='__main__':main()
