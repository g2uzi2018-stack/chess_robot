from pathlib import Path
import hashlib,json,os,sys
import numpy as np
import cv2
import torch
from scipy.optimize import linear_sum_assignment

O=Path(__file__).resolve().parent
artifact=Path(sys.argv[1]);label=sys.argv[2]
os.environ['XIANGQI_MODEL_ARTIFACT']=str(artifact)
os.environ['XIANGQI_MODEL_ARTIFACT_SHA256']=hashlib.sha256(artifact.read_bytes()).hexdigest()
os.environ['XIANGQI_SOURCE_WEIGHTS_SHA256']='f4fce2c0217b2ce292820e45f1779654b4cf58da3d156aff6306d56ae6aabf7e'
sys.path.insert(0,str(O/'source'))
torch.set_float32_matmul_precision('highest');torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
from recognize_tracked import Experiment
out=O/('parity-'+label);out.mkdir(exist_ok=False)
exp=Experiment(640,half=False)
capture=O/'baseline/capture';color=cv2.imread(str(capture/'color.png'));depth=np.load(capture/'depth_m.npy');meta=json.loads((capture/'metadata.json').read_text())
cases=[None,[320,80,960,720]];summary=[]
for i,roi in enumerate(cases):
    detection=exp.detect(color,roi);record,_=exp.run(color,depth,meta,roi=roi,draw=False,detection=detection)
    np.savez_compressed(out/(str(i)+'.npz'),predictions=detection['predictions'],masks=detection['masks'])
    (out/(str(i)+'.json')).write_text(json.dumps(record,indent=2,allow_nan=False))
    summary.append(dict(case=i,roi=roi,raw=len(detection['predictions']),deduplicated=record['deduplicated_count'],compute=detection['compute']))
(out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
