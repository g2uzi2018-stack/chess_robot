"""Read-only measurement policy. No arm reference is an image prerequisite."""
import math

CAMERA_FRAME = 'd405_color_optical_frame'


def measured_stopped(arm, joint, age_s):
    names = joint.get('name', [])
    q, v = joint.get('position', []), joint.get('velocity', [])
    return (0 <= age_s <= .2 and arm.get('feedback_fresh', False)
            and not arm.get('simulated', True) and not arm.get('busy', True)
            and arm.get('controller_mode') in (4, 5)
            and len(names) == len(set(names)) == len(q) == len(v)
            and all('joint'+str(i) in names for i in range(1, 7))
            and all(math.isfinite(x) for x in q+v)
            and max(map(abs, v)) <= .002)


def revoke_base(record, reason):
    """Keep camera observations; never leave stale base fields or identities."""
    record.update(base_transform_valid=False, base_transform_reason=reason)
    record.pop('source_joint_position_rad', None)
    for d in record['detections']:
        for key in list(d):
            if key.startswith(('base_', 'track_', 'surface_normal_base')) or key in (
                    'surface_tilt_deg', 'position_stamp_ns', 'reacquired_by_position_after_restart'):
                d.pop(key, None)
        d.update(valid=False, base_geometry_valid=False, complete_center_target=False,
                 track_id=None, track_confirmed=False, reason=reason,
                 motion_ready=False, grasp_ready=False)


def expire_observation(record):
    revoke_base(record, 'image observation expired during pipeline')
    record.update(detection_valid=False, camera_geometry_valid=False)
    for d in record['detections']:
        d.update(detection_valid=False, camera_geometry_valid=False)
        for key in ('camera_surface_center_m', 'camera_surface_normal'):
            d.pop(key, None)
