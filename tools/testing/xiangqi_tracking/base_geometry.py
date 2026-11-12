"""Read-only base geometry, deliberately independent of Torch and detector APIs."""
import hashlib
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

from qualification import measured_stopped

ROOT = Path('/home/kuang/workspace/chess_robot')
CALIB = ROOT/'docs/validation/handeye-20261030/flange_T_camera_candidate.yaml'
URDF = ROOT/'src/chess_robot_description/urdf/nova5_with_ag95.urdf'


def base_transform(model, metadata):
    """Same qualified FK/TCP calculation used by the original Experiment."""
    arm, joint = metadata.get('arm', {}), metadata.get('joint', {})
    if not metadata.get('base_eligible', False):
        return None, None, metadata.get('base_reason', 'synchronized stopped feedback unavailable')
    if not measured_stopped(arm, joint, metadata.get('joint_age_s', 0)):
        return None, None, 'measured feedback rejected'
    if model.X is None or model.fk is None:
        return None, None, model.base_model_reason
    try:
        poses = model.fk(model.robot, dict(zip(joint['name'], joint['position'])))
        transform = np.array(poses['Link6']) @ model.X
        position = arm['tcp']['pose']['position']
        actual = np.array([position[k] for k in ('x', 'y', 'z')])
        error = float(np.linalg.norm(np.array(poses['grasp_link'])[:3, 3]-actual))
        assert np.isfinite(transform).all() and np.isfinite(error) and error < .0001
        assert np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-6)
        assert abs(np.linalg.det(transform[:3, :3])-1) < 1e-6
        assert arm['tcp']['header']['frame_id'] == 'base_link'
        return transform, error, 'read-only measured FK; motion qualification independent'
    except Exception:
        return None, None, 'arm model FK mismatch or invalid transform'


class BaseGeometry:
    def __init__(self, calibration=CALIB, urdf=URDF):
        self.X = self.robot = self.fk = None
        self.base_model_reason = 'base model unavailable'
        self.identity = dict(calibration=str(calibration), calibration_sha256=None, urdf_sha256=None)
        try:
            raw = Path(calibration).read_bytes()
            data = yaml.safe_load(raw)
            X = np.array(data['flange_T_camera'], dtype=float)
            assert data['parent_frame'] == 'Link6'
            assert data['child_frame'] == 'd405_color_optical_frame'
            assert X.shape == (4, 4) and np.isfinite(X).all() and np.allclose(X[3], [0, 0, 0, 1])
            assert np.allclose(X[:3, :3].T @ X[:3, :3], np.eye(3), atol=1e-6)
            assert abs(np.linalg.det(X[:3, :3])-1) < 1e-6
            robot_raw = Path(urdf).read_bytes()
            robot = ET.fromstring(robot_raw)
            sys.path.insert(0, str(ROOT/'tools/description'))
            from rigid_geometry import fk
            self.X, self.robot, self.fk = X, robot, fk
            self.base_model_reason = 'model loaded; live qualification required'
            self.identity.update(calibration_sha256=hashlib.sha256(raw).hexdigest(),
                                 urdf_sha256=hashlib.sha256(robot_raw).hexdigest())
        except Exception as error:
            self.base_model_reason = 'base model unavailable: '+type(error).__name__

    def base_transform(self, metadata):
        return base_transform(self, metadata)
