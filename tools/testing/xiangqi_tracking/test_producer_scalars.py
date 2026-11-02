"""Real Experiment record + cached detector output, no inference or device IO."""
import argparse
import copy
import json
import os
from pathlib import Path
import sys

import cv2
import numpy as np

import measurement_contract as wire


def non_native(value, path='$'):
    found=[]
    if isinstance(value, dict):
        for key, child in value.items():found.extend(non_native(child,path+'.'+str(key)))
    elif isinstance(value, list):
        for i, child in enumerate(value):found.extend(non_native(child,path+'['+str(i)+']'))
    elif value is not None and type(value) not in (str,int,float,bool):
        found.append(dict(path=path,type=type(value).__module__+'.'+type(value).__name__))
    return found


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    root=Path('/home/kuang/workspace/chess_robot/runs/vision_acceleration_20261101_01')
    capture=root/'baseline/capture';cached=root/'parity-trt'
    metadata=json.loads((capture/'metadata.json').read_text())
    color=cv2.imread(str(capture/'color.png'));depth=np.load(capture/'depth_m.npy')
    saved=np.load(cached/'0.npz');prior=json.loads((cached/'0.json').read_text())
    engine=Path(prior['identity']['inference_artifact'])
    with engine.open('rb') as f:
        size=int.from_bytes(f.read(4),'little');assert 0<size<1024*1024
        names={int(k):v for k,v in json.loads(f.read(size))['names'].items()}
    os.environ['XIANGQI_MODEL_ARTIFACT']=str(engine)
    from recognize_tracked import Experiment
    experiment=Experiment(camera_only=True,geometry_only=True)
    detection=dict(predictions=saved['predictions'],masks=saved['masks'],names=names,
                   roi_xyxy=[0,0,1280,720],image_size=[1280,720],identity=experiment.identity,
                   compute=prior['compute'],detector_s=0.,inference_speed_ms={})
    record,_=experiment.run(color,depth,metadata,draw=False,detection=detection)
    actual_paths=non_native(record)
    assert any(row['path']=='$.identity.torch' and row['type'].endswith('.TorchVersion') for row in actual_paths),actual_paths
    # Demonstrate the precise pre-fix failure without mutating any frozen file.
    before_error=None
    try:wire.check_json(record)
    except wire.ContractError as error:before_error=str(error)
    assert before_error is not None
    t=metadata['stamp_ns'];m=dict(metadata,camera_epoch='scalar-replay',rgbd_frame_key=['scalar-replay',1,1])
    packet=wire.camera_measurement(record,m,'scalar-replay',0,t+30_000_000)
    # This old cache has no same-image ROI class evidence. Scalar conversion is
    # independently testable, but the new production scene must reject it.
    legacy_rejection=None
    try:wire.validate(packet,t+40_000_000)
    except wire.ContractError as error:legacy_rejection=str(error)
    assert legacy_rejection=='missing full-candidate/ROI-class policy',legacy_rejection
    assert non_native(packet)==[]
    fixture=copy.deepcopy(record)
    fixture['compute'].update(fixture_bool=np.bool_(True),fixture_int=np.int64(7),fixture_float=np.float32(.25))
    normalized=wire.camera_measurement(fixture,m,'scalar-replay',0,t+30_000_000)
    assert type(normalized['compute']['fixture_bool']) is bool
    assert type(normalized['compute']['fixture_int']) is int
    assert type(normalized['compute']['fixture_float']) is float
    rejected=0
    for value in (np.float64('nan'),np.float32('inf'),np.float64('-inf'),np.array([1.,2.])):
        bad=copy.deepcopy(record);bad['compute']['fixture_bad']=value
        try:wire.camera_measurement(bad,m,'scalar-replay',0,t+30_000_000)
        except wire.ContractError:rejected+=1
    assert rejected==4
    external=copy.deepcopy(packet);external['compute']['bad']=np.bool_(True)
    try:wire.validate(external,t+40_000_000)
    except wire.ContractError:pass
    else:raise AssertionError('external validator was relaxed')
    result=dict(passed=True,real_record_non_native_paths=actual_paths,pre_fix_check_error=before_error,
                normalized_packet_non_native_paths=non_native(packet),payload_bytes=len(wire.encode(packet).encode()),
                synthetic_numpy_scalar_types=['bool','int','float'],nonfinite_and_array_rejections=rejected,
                external_validation_remains_strict=True,actual_model_constructor=True,
                inference_backend=experiment.identity['inference_backend'],
                inference_executed=False,cached_detector_outputs=True,physical_commands=0,
                production_protocol_validated=False,legacy_cache_protocol_rejection=legacy_rejection)
    a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__=='__main__':main()
