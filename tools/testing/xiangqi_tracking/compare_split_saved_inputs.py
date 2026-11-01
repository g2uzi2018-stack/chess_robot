"""CPU replay of saved detector outputs: original combined geometry vs topic split.

No inference, ROS, live camera, clocks or device connections are used.
"""
import argparse
import copy
import json
from pathlib import Path
import time

import cv2
import numpy as np

from base_geometry import BaseGeometry
from measurement_contract import camera_measurement, encode, decode
from recognize_tracked import Experiment
from scene_state import SceneState
from stable_observation import FeedbackWindow


def experiment(base, camera_only):
    e = Experiment.__new__(Experiment)
    e.imgsz = 640
    e.camera_only = camera_only
    e.X, e.robot, e.fk, e.base_model_reason = base.X, base.robot, base.fk, base.base_model_reason
    return e


def main():
    root = Path('/home/kuang/workspace/chess_robot/runs')
    p = argparse.ArgumentParser()
    p.add_argument('--capture', type=Path, default=root/'vision_acceleration_20261101_01/baseline/capture')
    p.add_argument('--cache', type=Path, default=root/'vision_acceleration_20261101_01/parity-trt')
    p.add_argument('--engine', type=Path, default=root/'perf_ceiling_20261031/01_backend/weights/20261031T143600930755709/best.engine')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=False)
    with a.engine.open('rb') as stream:
        length = int.from_bytes(stream.read(4), 'little'); assert 0 < length < 1024*1024
        names = {int(k): v for k, v in json.loads(stream.read(length))['names'].items()}
    color = cv2.imread(str(a.capture/'color.png')); depth = np.load(a.capture/'depth_m.npy')
    metadata = json.loads((a.capture/'metadata.json').read_text())
    base = BaseGeometry(); assert base.X is not None, base.base_model_reason
    old, camera = experiment(base, False), experiment(base, True)
    cases = []
    for i, roi in enumerate([None, [320, 80, 960, 720]]):
        saved = np.load(a.cache/(str(i)+'.npz'))
        previous = json.loads((a.cache/(str(i)+'.json')).read_text())
        identity = previous['identity']
        detection = dict(predictions=saved['predictions'], masks=saved['masks'], names=names,
                         roi_xyxy=roi or [0,0,1280,720], image_size=[1280,720],
                         identity=identity, compute=previous['compute'], detector_s=0., inference_speed_ms={})
        reference, _ = old.run(color, depth, metadata, roi, draw=False, detection=detection)
        camera_detection = dict(detection, identity={k:v for k,v in identity.items()
                                                    if k not in ('calibration','calibration_sha256','urdf_sha256')})
        measured, _ = camera.run(color, depth, metadata, roi, draw=False, detection=camera_detection)
        t = metadata['stamp_ns']
        m = dict(K=metadata['K'], camera_epoch='saved-one', rgbd_frame_key=['saved-one', i+1, i+1])
        packet = camera_measurement(measured, m, 'saved-one', 0, t)
        feedback = FeedbackWindow()
        for ft in range(t-1_000_000_000, t+50_000_001, 50_000_000):
            feedback.ingest(ft, metadata['arm'], metadata['joint'], ft, 'saved-controller')
        scene = SceneState(base=base)
        candidate = scene.process(decode(encode(packet)), t+80_000_000, feedback, True, 'saved-controller')
        assert candidate['base_transform_valid'] == reference['base_transform_valid']
        assert len(candidate['detections']) == len(reference['detections'])
        largest = 0.
        for before, after in zip(reference['detections'], candidate['detections']):
            for key in ('class_name','class_id','camera_geometry_valid','valid','complete_center_target','camera_complete_center_target'):
                assert before[key] == after[key], (i,key,before[key],after[key])
            for key in ('center_pixel','camera_surface_center_m','camera_surface_normal','base_surface_center_m','base_surface_normal'):
                if key in before:
                    delta = float(np.max(np.abs(np.array(before[key])-np.array(after[key]))))
                    largest = max(largest, delta); assert delta <= 1e-12, (i,key,delta)
        (a.output/('reference-'+str(i)+'.json')).write_text(json.dumps(reference,indent=2))
        (a.output/('split-'+str(i)+'.json')).write_text(json.dumps(candidate,indent=2))
        cases.append(dict(case=i,roi=roi,raw_targets=len(saved['predictions']),targets=len(candidate['detections']),
                          maximum_coordinate_normal_delta=largest,payload_bytes=len(encode(packet).encode()),passed=True))
    result = dict(passed=True, independent_saved_scenes=1, same_cached_detector_outputs=True,
                  pure_cpu_no_inference=True,cases=cases,physical_commands=0,
                  limitation='Full/ROI of one saved frame, not labelled accuracy or split production FPS acceptance')
    (a.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))


if __name__ == '__main__':
    main()
