"""Offline segmentation + visible surface position experiment; no robot commands."""
import argparse, hashlib, json, os, sys, time
from pathlib import Path
import cv2, numpy as np

ROOT=Path('/home/kuang/workspace/chess_robot')
TRAIN=Path('/home/kuang/workspace/xiangqi-yolo-training/20261031-yolo26s-seg-v1')
WEIGHTS=TRAIN/'runs/train/weights/best.pt'
MODEL_ARTIFACT=Path(os.environ.get('XIANGQI_MODEL_ARTIFACT',str(WEIGHTS)))
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
    def __init__(self,imgsz=640,half=False,geometry_only=False,camera_only=False):
        import torch,ultralytics
        from ultralytics import YOLO
        torch.set_num_threads(2); cv2.setNumThreads(1)
        if MODEL_ARTIFACT.suffix not in ('.pt','.engine'):raise ValueError('Only PT and TensorRT artifacts are supported')
        self.inference_backend='tensorrt' if MODEL_ARTIFACT.suffix=='.engine' else 'pytorch'
        artifact_sha=hashlib.sha256(MODEL_ARTIFACT.read_bytes()).hexdigest()
        expected=os.environ.get('XIANGQI_MODEL_ARTIFACT_SHA256')
        if expected and artifact_sha!=expected:raise ValueError('Inference artifact SHA256 mismatch')
        source_sha=hashlib.sha256(WEIGHTS.read_bytes()).hexdigest()
        expected_source=os.environ.get('XIANGQI_SOURCE_WEIGHTS_SHA256')
        if expected_source and source_sha!=expected_source:raise ValueError('Source weights changed; rebuild/validate the engine')
        if self.inference_backend=='tensorrt':
            if half:raise ValueError('The deployed TensorRT policy requires FP32')
            torch.set_float32_matmul_precision('highest')
            torch.backends.cuda.matmul.allow_tf32=False
            torch.backends.cudnn.allow_tf32=False
        self.artifact_sha256=artifact_sha
        self.model=None if geometry_only else YOLO(str(MODEL_ARTIFACT),task='segment')
        if self.model is not None:assert self.model.task=='segment'
        self.imgsz=imgsz
        self.half=half
        self.camera_only=bool(camera_only)
        self.candidate_mode=os.environ.get('XIANGQI_CANDIDATE_MODE','yolo')
        if self.candidate_mode not in ('yolo','depth_shadow','lite'):
            raise ValueError('XIANGQI_CANDIDATE_MODE must be yolo, depth_shadow or lite')
        self.candidate_model=None
        if self.candidate_mode=='lite':
            self.candidate_artifact=Path(os.environ['XIANGQI_CANDIDATE_ARTIFACT'])
            self.candidate_sha256=hashlib.sha256(self.candidate_artifact.read_bytes()).hexdigest()
            if self.candidate_sha256!=os.environ['XIANGQI_CANDIDATE_ARTIFACT_SHA256']:
                raise ValueError('Candidate engine SHA256 mismatch')
            if self.candidate_artifact.suffix!='.engine' or half:
                raise ValueError('Lite candidate deployment requires the validated FP32 engine')
            self.candidate_conf=float(os.environ.get('XIANGQI_CANDIDATE_CONF','0.1'))
            if not 0 < self.candidate_conf < 1:raise ValueError('Invalid candidate confidence')
            if not geometry_only:self.candidate_model=YOLO(str(self.candidate_artifact),task='detect')
        self.depth_configuration=None
        if self.candidate_mode=='depth_shadow':
            from candidate_roi import load_depth_configuration
            self.depth_configuration=load_depth_configuration(os.environ.get('XIANGQI_DEPTH_CONFIG'))
        self.X=self.robot=self.fk=None;self.base_model_reason='base transform belongs to scene node'
        base_identity={}
        if not self.camera_only:
            from base_geometry import BaseGeometry
            base=BaseGeometry()
            self.X,self.robot,self.fk=base.X,base.robot,base.fk
            self.base_model_reason=base.base_model_reason;base_identity=base.identity
        self.identity=dict(weights=str(WEIGHTS),weights_sha256=source_sha,ultralytics=ultralytics.__version__,torch=torch.__version__,imgsz=imgsz,inference_backend=self.inference_backend,inference_artifact=str(MODEL_ARTIFACT),inference_artifact_sha256=self.artifact_sha256)
        self.identity.update(base_identity)
        if self.candidate_mode=='lite':
            self.identity.update(candidate_mode='lite',candidate_artifact=str(self.candidate_artifact),
                                 candidate_artifact_sha256=self.candidate_sha256,
                                 candidate_input_hw=[384,640],candidate_conf=self.candidate_conf,
                                 candidate_dedup_iou=.65)
        if self.depth_configuration is not None:
            self.identity.update(candidate_mode=self.candidate_mode,
                                 depth_config_sha256=self.depth_configuration['config_sha256'],
                                 depth_qualification_mode='shadow')

    def base_transform(self,metadata):
        from base_geometry import base_transform
        return base_transform(self,metadata)

    def detect(self,color,roi=None):
        assert self.model is not None
        start=time.monotonic()
        x1,y1,x2,y2=roi if roi is not None else (0,0,color.shape[1],color.shape[0])
        result=self.model.predict(color[y1:y2,x1:x2],device=0,imgsz=self.imgsz,conf=.25,verbose=False,retina_masks=True,half=self.half)[0]
        predictions=result.boxes.data.cpu().numpy()
        masks=(result.masks.data.cpu().numpy()>.5).astype(np.uint8) if result.masks is not None else None
        parameter_device=None
        if hasattr(self.model.model,'parameters'):
            try:parameter_device=str(next(self.model.model.parameters()).device)
            except StopIteration:pass
        fp16=bool(self.model.predictor.model.fp16)
        if self.inference_backend=='tensorrt' and fp16:raise ValueError('TensorRT engine is not FP32')
        compute=dict(predictor_device=str(self.model.predictor.device),model_parameter_device=parameter_device,boxes_device=str(result.boxes.data.device),masks_device=str(result.masks.data.device) if result.masks is not None else None,fp16=fp16,inference_backend=self.inference_backend,inference_artifact_sha256=self.artifact_sha256)
        if self.inference_backend=='tensorrt':
            compute.update(matmul_allow_tf32=bool(__import__('torch').backends.cuda.matmul.allow_tf32),cudnn_allow_tf32=bool(__import__('torch').backends.cudnn.allow_tf32))
        return dict(predictions=predictions,masks=masks,names=dict(result.names),compute=compute,inference_speed_ms=result.speed,detector_s=time.monotonic()-start,identity=self.identity,roi_xyxy=[x1,y1,x2,y2],image_size=[color.shape[1],color.shape[0]])

    def detect_lite(self,color):
        if color.shape[:2]!=(720,1280):raise ValueError('Candidate engine requires 1280x720 source images')
        start=time.monotonic()
        result=self.candidate_model.predict(color,device=0,imgsz=[384,640],rect=True,
                   conf=self.candidate_conf,verbose=False,half=False)[0]
        if dict(result.names)!={0:'piece'}:raise ValueError('Candidate engine class map differs from delivery')
        if self.candidate_model.predictor.model.fp16:raise ValueError('Candidate engine is not FP32')
        return dict(predictions=result.boxes.data.cpu().numpy(),masks=None,names=dict(result.names),
                    compute=dict(candidate_artifact_sha256=self.candidate_sha256,
                                 candidate_backend='tensorrt',candidate_fp16=False),
                    identity=self.identity,inference_speed_ms=result.speed,
                    detector_s=time.monotonic()-start,roi_xyxy=[0,0,1280,720],image_size=[1280,720])

    def detect_candidates(self,color,hint=None,depth=None,metadata=None):
        from candidate_roi import recognize, recognize_depth, recognize_lite
        if getattr(self,'candidate_mode','yolo')=='lite':
            return recognize_lite(color,self.detect_lite,self.detect,self.model.names,hint)
        if getattr(self,'candidate_mode','yolo')=='depth_shadow':
            if depth is None or metadata is None:
                raise ValueError('depth_shadow requires same-frame depth and acquisition metadata')
            if self.model is None:
                raise ValueError('ROI classification requires the model worker')
            return recognize_depth(color,depth,metadata,self.detect,self.model.names,
                                   self.identity,dict(inference_backend=self.inference_backend,
                                   inference_artifact_sha256=self.artifact_sha256,
                                   fp16=False,candidate_backend='cpu_depth'),
                                   self.depth_configuration,hint)
        return recognize(color,self.detect,hint)

    def run(self,color,depth,metadata,roi=None,draw=True,detection=None):
        if detection is None:detection=self.detect(color,roi)
        if detection.get('recognition_policy'):
            # In the split chain roi is only an optional classification hint;
            # proposal coverage and all geometry are in the full image plane.
            roi=None
        start=time.monotonic(); K=np.array(metadata['K']).reshape(3,3)
        assert color.shape[:2]==depth.shape and depth.dtype==np.float32
        transform,fk_error,base_reason=(None,None,'base transform belongs to scene node') if getattr(self,'camera_only',False) else self.base_transform(metadata);base_valid=transform is not None
        joint=metadata.get('joint',{})
        x1,y1,x2,y2=roi if roi is not None else (0,0,color.shape[1],color.shape[0])
        assert detection['roi_xyxy']==[x1,y1,x2,y2] and detection['image_size']==[color.shape[1],color.shape[0]]
        predictions=detection['predictions'];names=detection['names']
        boxes=predictions[:,:4]+[x1,y1,x1,y1];conf=predictions[:,-2];classes=predictions[:,-1].astype(int)
        masks=detection['masks']
        roi_K=K.copy();roi_K[0,2]-=x1;roi_K[1,2]-=y1
        groups=[]
        for i in np.argsort(-conf):
            if detection.get('recognition_policy'):
                groups.append([int(i)]);continue
            existing=next((g for g in groups if iou(boxes[i],boxes[g[0]])>.65),None)
            if existing is None:groups.append([int(i)])
            else:existing.append(int(i))
        overlay=color.copy() if draw else None; detections=[]
        for number,group in enumerate(groups):
            i=group[0];box=boxes[i];name=names[int(classes[i])]
            record=dict(id=number,class_id=int(classes[i]),class_name=name,confidence=float(conf[i]),box_xyxy=box.tolist(),alternatives=[dict(name=names[int(classes[k])],confidence=float(conf[k])) for k in group[1:]],touches_image_edge=bool(box[0]<2 or box[1]<2 or box[2]>color.shape[1]-2 or box[3]>color.shape[0]-2),class_verified=False)
            if detection.get('recognition_policy'):
                record.update(detection['classification_evidence'][i])
                record['classification_stamp_ns']=metadata['stamp_ns']
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
            if record.get('candidate_geometry_valid') is False:
                # Class evidence and geometry qualification are independent:
                # a large depth component with holes still deserves an ROI
                # class check, but a good fitted inner surface cannot erase
                # the candidate scanner's unresolved boundary/coverage gate.
                record.update(valid=False,reason='depth candidate geometry incomplete')
            record['detection_valid']=True
            record['camera_geometry_valid']=bool(record.get('valid'))
            record['base_geometry_valid']=bool(record['camera_geometry_valid'] and base_valid)
            record['valid']=record['base_geometry_valid']
            if record['base_geometry_valid']:
                record['base_surface_center_m']=(transform@np.r_[record['camera_surface_center_m'],1])[:3].tolist()
                record['base_surface_normal']=(transform[:3,:3]@record['camera_surface_normal']).tolist()
            elif record['camera_geometry_valid']:record.update(valid=False,reason=base_reason)
            record['motion_ready']=record['grasp_ready']=False
            record['camera_frame_id']='d405_color_optical_frame';record['base_frame_id']='base_link'
            record['touches_processing_edge']=bool(box[0]<x1+2 or box[1]<y1+2 or box[2]>x2-2 or box[3]>y2-2)
            record['camera_complete_center_target']=bool(record['camera_geometry_valid'] and not record['touches_image_edge'] and not record['touches_processing_edge'])
            record['complete_center_target']=bool(record.get('valid') and not record['touches_image_edge'] and not record['touches_processing_edge'])
            x,y=map(int,box[:2])
            if draw:cv2.putText(overlay,f"#{number} {name} {conf[i]:.2f}",(max(2,x),max(65,y-8)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,230,0),2)
            if draw and record.get('camera_geometry_valid'):
                u,v=map(lambda x:int(round(x)),record['center_pixel']);cv2.drawMarker(overlay,(u,v),(0,255,255),cv2.MARKER_CROSS,22,2)
                xyz=np.array(record['camera_surface_center_m'])*1000
                cv2.putText(overlay,f"camera mm {xyz[0]:.1f},{xyz[1]:.1f},{xyz[2]:.1f}",(max(2,x),min(color.shape[0]-12,int(box[3])+20)),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,255,255),1)
            detections.append(record)
        elapsed=time.monotonic()-start
        if draw:
            cv2.rectangle(overlay,(0,0),(color.shape[1],48),(0,0,0),-1)
            cv2.putText(overlay,f'EXPERIMENT / {color.shape[1]}x{color.shape[0]} / YOLO imgsz={self.imgsz} / {len(groups)} objects / {elapsed*1000:.0f}ms',(8,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
            cv2.putText(overlay,'Classes unverified | surface XYZ only | no grasp commands',(8,40),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,255,255),1)
        record=dict(stamp_ns=metadata['stamp_ns'],source_joint_position_rad=joint.get('position',[]),source_joint_names=joint.get('name',[]),frame_id='base_link',camera_frame_id='d405_color_optical_frame',base_frame_id='base_link',schema='xiangqi_measurements/v2',detection_valid=True,camera_geometry_valid=any(d['camera_geometry_valid'] for d in detections),motion_ready=False,grasp_ready=False,base_transform_reason=base_reason,identity=detection['identity'],compute=detection['compute'],image_size=[color.shape[1],color.shape[0]],inference_roi_xyxy=[x1,y1,x2,y2],detections=detections,raw_detection_count=len(boxes),deduplicated_count=len(groups),fk_tcp_error_m=fk_error,base_transform_valid=base_valid,inference_speed_ms=detection['inference_speed_ms'],geometry_ms=elapsed*1000,detector_ms=detection['detector_s']*1000,processing_s=elapsed+detection['detector_s'],full_6d_pose_estimated=False,robot_commands=0)
        if detection.get('recognition_policy'):
            for key in ('recognition_policy','full_scene_coverage','full_candidate_count',
                        'classification_complete','classification_rois'):
                record[key]=detection[key]
            if 'depth_scan' in detection:
                record['depth_scan']=detection['depth_scan']
            record['candidate_scan_stamp_ns']=metadata['stamp_ns']
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
