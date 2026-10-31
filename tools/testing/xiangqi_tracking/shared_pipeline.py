"""Three bounded slots: single GPU inference overlaps CPU geometry on the preceding frame."""
import multiprocessing
from multiprocessing import shared_memory
from concurrent.futures import ProcessPoolExecutor
import time
import numpy as np

class FrameSlots:
    def __init__(self,width=1280,height=720,count=3,max_masks=300,names=None,locks=None):
        self.width=width;self.height=height;self.count=count;self.max_masks=max_masks;self.owner=names is None
        self.color_offset=64;self.depth_offset=self.color_offset+width*height*3;self.mask_offset=self.depth_offset+width*height*4
        size=self.mask_offset+max_masks*width*height
        self.blocks=[shared_memory.SharedMemory(create=True,size=size) for _ in range(count)] if self.owner else [shared_memory.SharedMemory(name=n) for n in names]
        self.locks=locks if locks is not None else [multiprocessing.get_context('spawn').Lock() for _ in range(count)]
        self.headers=[np.ndarray((2,),np.int64,buffer=s.buf) for s in self.blocks]
        self.colors=[np.ndarray((height,width,3),np.uint8,buffer=s.buf,offset=self.color_offset) for s in self.blocks]
        self.depths=[np.ndarray((height,width),np.float32,buffer=s.buf,offset=self.depth_offset) for s in self.blocks]
        self.masks=[np.ndarray((max_masks,height,width),np.uint8,buffer=s.buf,offset=self.mask_offset) for s in self.blocks]
    def configuration(self):return dict(width=self.width,height=self.height,count=self.count,max_masks=self.max_masks,names=[s.name for s in self.blocks],locks=self.locks)
    def write(self,slot,job_id,stamp_ns,color,depth):
        assert color.shape==(self.height,self.width,3) and color.dtype==np.uint8
        assert depth.shape==(self.height,self.width) and depth.dtype==np.float32
        with self.locks[slot]:
            np.copyto(self.colors[slot],color);np.copyto(self.depths[slot],depth);self.headers[slot][:]=[job_id,stamp_ns]
    def check(self,job):
        assert self.headers[job['slot']].tolist()==[job['job_id'],job['metadata']['stamp_ns']],'Shared frame ownership/stamp mismatch'
    def close(self):
        self.headers.clear();self.colors.clear();self.depths.clear();self.masks.clear()
        for block in self.blocks:
            block.close()
            if self.owner:block.unlink()

worker_slots=None;worker_experiment=None
def initialize_gpu(configuration):
    global worker_slots,worker_experiment
    from recognize_tracked import Experiment
    worker_slots=FrameSlots(**configuration);worker_experiment=Experiment(640,half=False)
def initialize_geometry(configuration):
    global worker_slots,worker_experiment
    from recognize_tracked import Experiment
    worker_slots=FrameSlots(**configuration);worker_experiment=Experiment(640,half=False,geometry_only=True)
def detect_slot(job):
    start=time.monotonic();slot=job['slot']
    with worker_slots.locks[slot]:
        worker_slots.check(job)
        packet=worker_experiment.detect(worker_slots.colors[slot],job['roi'])
        masks=packet.pop('masks');packet['mask_shape']=None if masks is None else list(masks.shape)
        if masks is not None:
            n,h,w=masks.shape;assert n<=worker_slots.max_masks and h<=worker_slots.height and w<=worker_slots.width
            worker_slots.masks[slot][:n,:h,:w]=masks
        worker_slots.check(job)
    end=time.monotonic()
    return dict(job_id=job['job_id'],stamp_ns=job['metadata']['stamp_ns'],packet=packet,gpu_worker_ms=(end-start)*1000,gpu_interval=[start,end])
def geometry_slot(job,detection):
    start=time.monotonic();slot=job['slot']
    assert detection['job_id']==job['job_id'] and detection['stamp_ns']==job['metadata']['stamp_ns'],'Inference/frame pairing mismatch'
    with worker_slots.locks[slot]:
        worker_slots.check(job);packet=dict(detection['packet']);shape=packet.pop('mask_shape')
        packet['masks']=None if shape is None else worker_slots.masks[slot][:shape[0],:shape[1],:shape[2]]
        record,overlay=worker_experiment.run(worker_slots.colors[slot],worker_slots.depths[slot],job['metadata'],job['roi'],draw=job.get('draw',False),detection=packet)
        worker_slots.check(job)
    record['pipeline_job_id']=job['job_id'];record['gpu_worker_ms']=detection['gpu_worker_ms'];record['geometry_worker_ms']=(time.monotonic()-start)*1000
    record['pipeline_trace']=dict(gpu=detection['gpu_interval'],cpu=[start,time.monotonic()])
    return record,overlay

class Pipeline:
    def __init__(self,width=1280,height=720):
        self.slots=FrameSlots(width,height);context=multiprocessing.get_context('spawn');configuration=self.slots.configuration()
        self.gpu=ProcessPoolExecutor(max_workers=1,mp_context=context,initializer=initialize_gpu,initargs=(configuration,))
        self.geometry=ProcessPoolExecutor(max_workers=1,mp_context=context,initializer=initialize_geometry,initargs=(configuration,))
        self.gpu_job=None;self.geometry_jobs=[];self.free=list(range(self.slots.count));self.next_id=0;self.last_returned=-1
    def can_submit(self):return self.gpu_job is None and bool(self.free)
    def submit(self,color,depth,metadata,roi=None,draw=False,context=None):
        assert self.can_submit(),'Pipeline full; never overwrite an in-flight frame'
        slot=self.free.pop(0);job=dict(slot=slot,job_id=self.next_id,metadata=metadata,roi=roi,draw=draw,context=context,started=time.monotonic());self.next_id+=1
        self.slots.write(slot,job['job_id'],metadata['stamp_ns'],color,depth)
        future=self.gpu.submit(detect_slot,job);self.gpu_job=(job,future)
        return job['job_id']
    def poll(self):
        if self.gpu_job is not None:
            job,future=self.gpu_job
            if future.done():
                detection=future.result();self.geometry_jobs.append((job,self.geometry.submit(geometry_slot,job,detection)));self.gpu_job=None
        ready=[]
        while self.geometry_jobs and self.geometry_jobs[0][1].done():
            job,future=self.geometry_jobs.pop(0);record,overlay=future.result()
            assert job['job_id']>self.last_returned and record['pipeline_job_id']==job['job_id']
            self.last_returned=job['job_id'];self.free.append(job['slot']);ready.append((job,record,overlay))
        return ready
    def busy(self):return self.gpu_job is not None or bool(self.geometry_jobs)
    def close(self):
        try:
            self.gpu.shutdown(wait=True,cancel_futures=True)
            self.geometry.shutdown(wait=True,cancel_futures=True)
        finally:self.slots.close()
