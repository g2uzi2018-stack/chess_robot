"""Pure scene consumer: camera measurements -> qualified base IDs and snapshots."""
import copy
import json
import time
import uuid
from collections import Counter

import numpy as np

from base_geometry import BaseGeometry
from measurement_contract import ContractError, HINT_SCHEMA, SequenceGuard
from piece_tracker import PieceTracker
from qualification import expire_observation, revoke_base
from stable_observation import Policy, StableObservation, observation_roi


class SceneState:
    def __init__(self, policy=None, base=None, tracker=None, save=None, session=None):
        self.policy = policy or Policy()
        self.base = base or BaseGeometry()
        self.tracker = tracker or PieceTracker()
        self.observation = StableObservation(self.policy, save)
        self.session = session or uuid.uuid4().hex
        self.guard = SequenceGuard()
        self.last_base_stamp = 0
        self.retire_serial = 0
        self.last_record = None
        self.last_measurement = None
        self.roi = None
        self.roi_source = None
        self.rejects = Counter()
        self.packet_rejections = Counter()
        self.source_identity = None
        self.source_configuration_fault = False
        self.configuration_fault = False

    def _retire(self, reason):
        self.tracker.tracks.clear()
        self.tracker.startup_ids.clear()
        self.last_base_stamp = 0
        self.observation.invalidate(reason)
        self.observation.requested = True
        self.observation.last_t = 0
        self.observation.frame_keys.clear()
        self.roi = None
        self.roi_source = None

    def source_fresh(self, now):
        return bool(self.last_measurement and self.last_measurement.get('detection_valid')
                    and 0 <= (now-self.last_measurement['stamp_ns'])*1e-9 <= .5)

    def trusted(self, now, feedback, raw_good, audit_token):
        held = feedback.at(now, now)
        return bool(raw_good and held['ok'] and audit_token == feedback.epoch
                    and not self.configuration_fault and not self.source_configuration_fault)

    def tick(self, now, feedback, raw_good, audit_token):
        good = self.trusted(now, feedback, raw_good, audit_token) and self.source_fresh(now)
        self.observation.tick(now, good, feedback.generation, audit_token)
        if self.configuration_fault:
            self.observation.invalidate('on-disk scene configuration changed; scene restart required')
        elif self.source_configuration_fault:
            self.observation.invalidate('perception configuration changed; perception restart required')
        return good

    def reject_packet(self, reason, now):
        self.packet_rejections[reason] += 1
        self.observation.pending = True
        self.observation.state = 'pending_confirmation'
        self.observation.reason = 'camera measurement rejected: '+reason
        return self.expired_record(now, 'camera measurement rejected: '+reason)

    def expired_record(self, now, reason='perception stream unavailable or expired'):
        """Preserve acquisition stamp while explicitly revoking current coordinates."""
        if self.last_record is None:
            return None
        record = copy.deepcopy(self.last_record)
        expire_observation(record)
        record.update(reason=reason, base_transform_reason=reason, observation=self.observation.status(now),
                      publish_latency_s=(now-record['stamp_ns'])*1e-9,
                      scene_packet_rejections=dict(self.packet_rejections))
        return record

    def process(self, packet, now, feedback, raw_good, audit_token):
        changed = self.guard.accept(packet, now)
        if changed:
            self._retire('perception source/camera epoch changed')
            self.source_identity = None
            self.source_configuration_fault = False
        if packet['detection_valid']:
            signature = json.dumps(dict(identity=packet.get('identity'), K=packet.get('source_K'),
                                        image_size=packet.get('image_size')), sort_keys=True, allow_nan=False)
            if self.source_identity is None:
                self.source_identity = signature
            elif self.source_identity != signature:
                self.source_configuration_fault = True
        if not packet['configuration_valid']:
            self.source_configuration_fault = True
        self.last_measurement = copy.deepcopy(packet)
        safe = self.tick(now, feedback, raw_good, audit_token)
        t = packet['stamp_ns']
        check = feedback.at(t, now)
        safe = bool(safe and check['ok'])
        geometry_start = time.monotonic()
        metadata = dict(base_eligible=safe,
                        base_reason=check['reason'] if not check['ok'] else 'passive feedback or configuration unavailable')
        if safe:
            row = check['row']
            metadata.update(arm=row['arm'], joint=row['joint'], joint_age_s=(t-row['t'])*1e-9)
        transform, error, reason = self.base.base_transform(metadata)
        base_valid = bool(transform is not None and packet['detection_valid'])
        record = copy.deepcopy(packet)
        record.update(schema='xiangqi_measurements/v2', frame_id='base_link', base_frame_id='base_link',
                      base_transform_valid=base_valid, base_transform_reason=reason, fk_tcp_error_m=error,
                      source_joint_position_rad=metadata.get('joint', {}).get('position', []),
                      source_joint_names=metadata.get('joint', {}).get('name', []))
        record['identity'] = dict(packet.get('identity') or {}, **self.base.identity)
        for d in record['detections']:
            d['id'] = d.pop('instance_index')
            d['base_frame_id'] = 'base_link'
            d['base_geometry_valid'] = bool(d['camera_geometry_valid'] and base_valid)
            d['valid'] = d['base_geometry_valid']
            d['complete_center_target'] = bool(d['base_geometry_valid'] and not d.get('touches_image_edge')
                                               and not d.get('touches_processing_edge'))
            if d['base_geometry_valid']:
                d['base_surface_center_m'] = (transform @ np.r_[d['camera_surface_center_m'], 1])[:3].tolist()
                d['base_surface_normal'] = (transform[:3, :3] @ d['camera_surface_normal']).tolist()
            elif d['camera_geometry_valid']:
                d['reason'] = reason
            d['motion_ready'] = d['grasp_ready'] = False
        base_ms = (time.monotonic()-geometry_start)*1000
        retire = bool(feedback.retire_serial != self.retire_serial or
                      (self.last_base_stamp and (t-self.last_base_stamp)*1e-9 > self.policy.retire_gap_s))
        if retire:
            self._retire('feedback or observation continuity changed')
        self.retire_serial = feedback.retire_serial
        tracking_start = time.monotonic()
        if base_valid:
            self.tracker.update(record['detections'], t)
            if any(d.get('track_id') is not None for d in record['detections']):
                self.last_base_stamp = t
        else:
            for d in record['detections']:
                d.update(track_id=None, track_confirmed=False)
            self.rejects[reason] += 1
        tracking_ms = (time.monotonic()-tracking_start)*1000
        record.update(track_namespace=self.session, feedback_stamp_ns=check['feedback_stamp_ns'],
                      controller_epoch=audit_token, stationary_generation=feedback.generation,
                      observation_frame_index=packet['sequence'], tracker_state='retired' if retire or changed else
                      ('observed' if base_valid else 'suspended'), base_rejection_counts=dict(self.rejects),
                      scene_packet_rejections=dict(self.packet_rejections), tracking_ms=tracking_ms)
        # Every sampling AND confirmation frame requires full-image candidate
        # coverage plus current ROI labels and geometry for the entire union.
        # Keep unresolved detections visible; never silently sample a subset.
        semantic_complete = bool(packet.get('full_scene_coverage') and packet.get('classification_complete')
                                 and record['detections'] and all(d.get('classification_valid')
                                                                 for d in record['detections']))
        scene_complete = bool(semantic_complete and all(d.get('camera_complete_center_target')
                                                       for d in record['detections']))
        observation_trusted = bool(safe and base_valid)
        if not packet['detection_valid']:
            # Invalid camera routing events must pause current qualification,
            # not replace the accepted model/intrinsic fingerprint with nulls.
            self.observation.tick(now, False, feedback.generation, audit_token)
        elif scene_complete:
            self.observation.observe(record, now, observation_trusted, feedback.generation, audit_token)
        else:
            # observe() must still see this frame to pause current eligibility,
            # count independent-frame gaps and revoke after repeated mismatch.
            self.observation.observe(record, now, False, feedback.generation, audit_token)
            # A semantic/coverage gap cannot be averaged into a partial scene.
            # Pure geometry rejection retains the established gap/fraction
            # policy: skip this frame without requiring 60 consecutive passes.
            if not semantic_complete:
                self.observation.frames.clear()
            self.observation.reason = 'full-scene ROI classification or geometry incomplete'
        if self.configuration_fault or self.source_configuration_fault:
            self.tick(now, feedback, raw_good, audit_token)
        record['observation'] = self.observation.status(now)
        usable = [d['box_xyxy'] for d in record['detections'] if d.get('camera_complete_center_target')]
        if usable:
            self.roi = observation_roi(usable)
            self.roi_source = dict(source_epoch=packet['source_epoch'], sequence=packet['sequence'], stamp_ns=t)
        for d in record['detections']:
            d['track_namespace'] = self.session if d.get('track_id') is not None else None
        transit_ms = max(0, (now-packet.get('perception_publish_ns', now))*1e-6)
        record['live_stage_ms'] = dict(packet.get('live_stage_ms') or {})
        stages = record['live_stage_ms']
        stages.update(geometry_stage=stages.get('geometry_stage', record.get('geometry_ms', 0))+base_ms,
                      scene_base_transform=base_ms, tracking=tracking_ms, perception_to_scene=transit_ms,
                      frame_pipeline_elapsed=stages.get('frame_pipeline_elapsed', 0)+transit_ms+tracking_ms+base_ms)
        stages['analysis'] = stages.get('analysis', record.get('processing_s', 0)*1000)+base_ms
        record['publish_latency_s'] = (now-t)*1e-9
        record['pipeline'] = 'bounded_shared_memory_gpu_cpu_overlap_then_scene_topic'
        record['motion_ready'] = record['grasp_ready'] = False
        self.last_record = copy.deepcopy(record)
        return record

    def hint(self, now):
        if not self.source_fresh(now):
            return None
        p = self.last_measurement
        origin = self.roi_source
        roi_fresh = bool(origin and origin['source_epoch'] == p['source_epoch']
                         and 0 <= (now-origin['stamp_ns'])*1e-9 <= .5)
        full = bool(self.observation.requested or self.roi is None or not roi_fresh or
                    self.configuration_fault or self.source_configuration_fault)
        origin = p if full else origin
        return dict(schema=HINT_SCHEMA, source_epoch=p['source_epoch'], based_on_sequence=origin['sequence'],
                    based_on_stamp_ns=origin['stamp_ns'], issued_ns=now, full_image=full,
                    roi_xyxy=None if full else self.roi)
