"""Bounded one-class detector training. Acquire the production GPU lease before CUDA."""
import os, sys, json, time, signal, fcntl, hashlib, traceback, math
from pathlib import Path

ROOT=Path('/home/kuang/workspace/xiangqi-yolo-training/20261102-yolo26n-piece-v1')
LOCK=Path('/home/kuang/workspace/chess_robot/runs/perf_ceiling_20261031/benchmark.lock')
STATUS=ROOT/'runtime/train-status.json'
started=time.time()
def write(**extra):
    state.update(extra, updated_at_unix=time.time(),elapsed_s=time.time()-started)
    p=STATUS.with_suffix('.tmp');p.write_text(json.dumps(state,indent=2,allow_nan=False));p.replace(STATUS)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
state=dict(state='waiting_for_gpu_lock',pid=os.getpid(),deployment_ready=False,physical_commands=0)
STATUS.parent.mkdir(parents=True,exist_ok=True);write()
def timeout(*_):raise TimeoutError('GPU blocking flock timeout (180 seconds)')
with LOCK.open('a') as lock:
    signal.signal(signal.SIGALRM,timeout);signal.alarm(180)
    try:
        fcntl.flock(lock,fcntl.LOCK_EX);signal.alarm(0)
        write(state='training',gpu_lock_acquired=True,lease_acquired_unix=time.time())
        import numpy as np
        import torch, ultralytics
        from ultralytics import YOLO
        from ultralytics.models.yolo.detect.train import DetectionTrainer
        assert ultralytics.__version__=='8.4.32',ultralytics.__version__
        torch.set_num_threads(4);torch.set_num_interop_threads(1)
        torch.set_float32_matmul_precision('highest')
        torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
        manifest=json.loads((ROOT/'dataset/manifest.json').read_text())
        items={x['id']:x for x in manifest['items']}
        fixed={name:sha(ROOT/'dataset'/name) for name in manifest['fixed_data_files_sha256']}
        expected='9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef'
        assert sha(ROOT/'weights/yolo26n.pt')==expected
        class PolygonAwareDetectionTrainer(DetectionTrainer):
            """Use existing polygons ONLY to keep rotated/cropped detect boxes tight.

            The published dataset has standard five-column detection labels;
            the model has no segmentation branch or mask loss. SDK unmodified.
            """
            def build_dataset(self,img_path,mode='train',batch=None):
                ds=super().build_dataset(img_path,mode,batch)
                if mode=='train':
                    total=0
                    for label in ds.labels:
                        item=items[Path(label['im_file']).stem]
                        assert item['split']=='train'
                        polygons=[np.array(x['source_polygon_normalized_xy'],dtype=np.float32) for x in item['instances']]
                        assert len(polygons)==len(label['cls'])
                        # Original object order must agree with the bbox labels.
                        expected_boxes=np.array([x['bbox_normalized_xywh'] for x in item['instances']])
                        assert np.allclose(label['bboxes'],expected_boxes,atol=2e-7)
                        label['segments']=polygons;total+=len(polygons)
                    write(polygon_augmentation_instances=total)
                return ds
        params=dict(data=str(ROOT/'dataset/data.yaml'),epochs=50,patience=10,imgsz=640,
                    batch=8,workers=2,device=0,optimizer='AdamW',lr0=.001,lrf=.01,
                    cos_lr=True,warmup_epochs=3,seed=42,deterministic=True,
                    amp=False,cache=False,save=True,save_period=10,
                    fliplr=0.,flipud=0.,hsv_h=0.,hsv_s=.15,hsv_v=.20,
                    degrees=180.,translate=.1,scale=.5,shear=0.,perspective=.0001,
                    mosaic=.5,close_mosaic=10,mixup=0.,cutmix=0.,copy_paste=0.,
                    project=str(ROOT/'runs'),name='train',exist_ok=False,plots=True,
                    verbose=True)
        model=YOLO(str(ROOT/'weights/yolo26n.pt'),task='detect')
        write(version=ultralytics.__version__,torch=torch.__version__,requested_parameters=params,
              pretrained_sha256=expected,augmentation_box_method='source polygons transformed by standard RandomPerspective, then enclosing boxes; nc1 detect model')
        def epoch_end(t):
            losses={k:float(v) for k,v in t.label_loss_items(t.tloss,prefix='train').items()}
            if not all(math.isfinite(v) for v in losses.values()):raise RuntimeError('nonfinite training loss')
            write(epoch=t.epoch+1,losses=losses,metrics={k:float(v) for k,v in t.metrics.items()},
                  peak_allocated_bytes=torch.cuda.max_memory_allocated(),effective_parameters=vars(t.args))
        model.add_callback('on_fit_epoch_end',epoch_end)
        metrics=model.train(trainer=PolygonAwareDetectionTrainer,**params)
        assert all(sha(ROOT/'dataset'/k)==v for k,v in fixed.items()),'immutable data changed'
        best=ROOT/'runs/train/weights/best.pt';last=best.with_name('last.pt')
        final=dict(aggregate={k:float(v) for k,v in metrics.results_dict.items()},
                   speed_ms_per_image=metrics.speed,epochs=state.get('epoch'),
                   weights={k:dict(path=str(p),sha256=sha(p),bytes=p.stat().st_size) for k,p in [('best',best),('last',last)]},
                   fixed_dataset_files_unchanged=len(fixed),source_split_unchanged=True,
                   training_precision='FP32 AMP=false TF32=false',deployment_ready=False)
        (ROOT/'runs/train/final_metrics.json').write_text(json.dumps(final,indent=2,allow_nan=False))
        write(state='completed',final=final,lease_released_on_exit=True)
    except BaseException as exc:
        write(state='failed',error=repr(exc),traceback=traceback.format_exc(),lease_released_on_exit=True)
        raise
    finally:
        signal.alarm(0)
        fcntl.flock(lock,fcntl.LOCK_UN)
