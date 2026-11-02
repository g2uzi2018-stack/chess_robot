"""Bounded camera measurement and optional ROI-hint wire contracts; no ROS imports."""
import copy
import json
import math
import re
from collections import deque
from candidate_roi import POLICY, DEPTH_POLICY, MAX_CROPS

CAMERA_FRAME = 'd405_color_optical_frame'
SCHEMA = 'xiangqi_camera_measurements/v2'
HINT_SCHEMA = 'xiangqi_roi_hint/v1'
MAX_MESSAGE_BYTES = 1024*1024
MAX_DETECTIONS = 300
MAX_OUTLINE_POINTS = 128
MAX_IMAGE_AGE_S = .5
MAX_JSON_DEPTH = 32
MAX_JSON_NODES = 250000


class ContractError(ValueError):
    pass


def integer(value, minimum=0):
    return type(value) is int and minimum <= value <= 2**64-1


def number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def token(value):
    return isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9_-]{1,64}', value) is not None


def vector(value, length):
    return (isinstance(value, list) and len(value) == length
            and all(number(v) for v in value))


def check_json(value):
    """Iterative whole-value checks also cover optional/invalid-frame metadata."""
    stack = [(value, 0)]
    count = 0
    while stack:
        item, depth = stack.pop()
        count += 1
        if depth > MAX_JSON_DEPTH or count > MAX_JSON_NODES:
            raise ContractError('JSON structure exceeds bounded depth/node count')
        if isinstance(item, dict):
            if not all(isinstance(k, str) for k in item):
                raise ContractError('JSON object keys must be strings')
            stack.extend((v, depth+1) for v in item.values())
        elif isinstance(item, list):
            stack.extend((v, depth+1) for v in item)
        elif type(item) is float and not math.isfinite(item):
            raise ContractError('non-finite JSON number')
        elif item is not None and type(item) not in (str, int, float, bool):
            raise ContractError('unsupported JSON value type')


def encode_json(record, max_bytes=MAX_MESSAGE_BYTES):
    check_json(record)
    try:
        wire = json.dumps(record, allow_nan=False, separators=(',', ':'))
    except (ValueError, TypeError, RecursionError) as error:
        raise ContractError('invalid measurement values') from error
    if len(wire.encode()) > max_bytes:
        raise ContractError('JSON exceeds bounded payload')
    return wire


def decode_json(wire, max_bytes=MAX_MESSAGE_BYTES):
    try:
        if not isinstance(wire, str) or len(wire.encode()) > max_bytes:
            raise ContractError('JSON exceeds bounded payload')
        depth = 0
        quoted = escaped = False
        for character in wire:
            if quoted:
                if escaped:
                    escaped = False
                elif character == '\\':
                    escaped = True
                elif character == '"':
                    quoted = False
            elif character == '"':
                quoted = True
            elif character in '[{':
                depth += 1
                if depth > MAX_JSON_DEPTH:
                    raise ContractError('JSON exceeds bounded depth')
            elif character in ']}':
                depth -= 1
        def finite_float(text):
            value = float(text)
            if not math.isfinite(value):
                raise ContractError('non-finite JSON number')
            return value
        value = json.loads(wire, parse_float=finite_float,
                           parse_constant=lambda text: (_ for _ in ()).throw(ContractError('non-finite JSON')))
        check_json(value)
        if not isinstance(value, dict):
            raise ContractError('JSON topic root must be an object')
        return value
    except (ValueError, TypeError, OverflowError, RecursionError) as error:
        raise ContractError('invalid, non-finite or unbounded JSON') from error


def encode(record):
    return encode_json(record)


def decode(wire):
    return decode_json(wire)


def producer_json_value(value):
    """Normalize internal scalar representations, never untrusted wire input.

    TorchVersion is a str subclass; ROS/NumPy geometry also supplies NumPy
    scalars. Arrays, unsupported objects and non-finite values remain errors.
    """
    import numpy as np
    count = 0
    def convert(item, depth):
        nonlocal count
        count += 1
        if depth > MAX_JSON_DEPTH or count > MAX_JSON_NODES:
            raise ContractError('producer JSON structure exceeds bounds')
        if item is None:
            return None
        if isinstance(item, (bool, np.bool_)):
            return bool(item)
        if isinstance(item, (int, np.integer)):
            return int(item)
        if isinstance(item, (float, np.floating)):
            converted = float(item)
            if not math.isfinite(converted):
                raise ContractError('non-finite producer number')
            return converted
        if isinstance(item, str):
            return str(item)
        if isinstance(item, dict):
            if not all(isinstance(key, str) for key in item):
                raise ContractError('producer JSON keys must be strings')
            return {str(key): convert(child, depth+1) for key, child in item.items()}
        if isinstance(item, list):
            return [convert(child, depth+1) for child in item]
        raise ContractError('unsupported producer JSON value type')
    return convert(value, 0)


def camera_measurement(record, metadata, source_epoch, sequence, published_ns, configuration_valid=True):
    """Copy small scalars/vectors only; no full masks, robot state or stable IDs."""
    if not record.get('detection_valid') or not configuration_valid:
        # Invalid camera data (including NaN K) is an event, not a valid model
        # measurement. Keep only finite, typed routing metadata for recovery.
        K = metadata.get('K')
        packet = dict(schema=SCHEMA, frame_id=CAMERA_FRAME, camera_frame_id=CAMERA_FRAME,
                      stamp_ns=metadata.get('stamp_ns', record.get('stamp_ns')),
                      source_epoch=source_epoch, sequence=sequence, perception_publish_ns=published_ns,
                      source_K=copy.deepcopy(K) if vector(K, 9) else None,
                      rgbd_frame_key=metadata['rgbd_frame_key'], camera_epoch=metadata['camera_epoch'],
                      image_size=[1280,720], configuration_valid=bool(configuration_valid),
                      detection_valid=False, camera_geometry_valid=False, detections=[],
                      motion_ready=False, grasp_ready=False, robot_commands=0,
                      reason=str(record.get('reason', 'camera image/configuration rejected'))[:256])
        packet = producer_json_value(packet)
        encode(packet)
        return packet
    packet = copy.deepcopy(record)
    for key in list(packet):
        if key.startswith(('base_', 'source_joint_', 'track_')) or key in ('fk_tcp_error_m', 'masks', 'mask', 'color', 'depth'):
            packet.pop(key, None)
    for detection in packet.get('detections', []):
        for key in list(detection):
            if key.startswith(('base_', 'track_', 'surface_normal_base')) or key in (
                    'valid', 'complete_center_target', 'surface_tilt_deg', 'position_stamp_ns',
                    'identity_ambiguous', 'reacquired_by_position_after_restart'):
                detection.pop(key, None)
        detection['instance_index'] = detection.pop('id', 0)
        detection['motion_ready'] = detection['grasp_ready'] = False
        for key in ('masks', 'mask', 'color', 'depth'):
            detection.pop(key, None)
        if detection.get('camera_geometry_valid'):
            detection.pop('reason', None)  # A missing base transform is not a camera error.
        outline = detection.get('mask_outline_xy', [])
        if len(outline) > MAX_OUTLINE_POINTS:
            # This is a display outline only; geometry used the unchanged full mask.
            stride = len(outline)/MAX_OUTLINE_POINTS
            detection['mask_outline_xy'] = [outline[int(i*stride)] for i in range(MAX_OUTLINE_POINTS)]
    packet.update(schema=SCHEMA, frame_id=CAMERA_FRAME, camera_frame_id=CAMERA_FRAME,
                  source_epoch=source_epoch, sequence=sequence, perception_publish_ns=published_ns,
                  source_K=metadata.get('K'), rgbd_frame_key=metadata['rgbd_frame_key'],
                  camera_epoch=metadata['camera_epoch'], configuration_valid=bool(configuration_valid),
                  motion_ready=False, grasp_ready=False, robot_commands=0)
    packet['camera_geometry_valid'] = bool(packet.get('detection_valid') and
                                          any(d.get('camera_geometry_valid') for d in packet.get('detections', [])))
    if not configuration_valid:
        packet.update(detection_valid=False, camera_geometry_valid=False, detections=[],
                      reason='on-disk perception configuration changed; restart required')
    packet = producer_json_value(packet)
    encode(packet)
    return packet


def validate(packet, now):
    check_json(packet)
    if not isinstance(packet, dict) or packet.get('schema') != SCHEMA:
        raise ContractError('unknown camera measurement schema')
    if not token(packet.get('source_epoch')) or not token(packet.get('camera_epoch')):
        raise ContractError('invalid source/camera epoch')
    if not integer(packet.get('sequence')) or not integer(packet.get('stamp_ns'), 1):
        raise ContractError('invalid sequence or original acquisition stamp')
    if not integer(packet.get('perception_publish_ns'), 1) or not packet['stamp_ns'] <= packet['perception_publish_ns'] <= now:
        raise ContractError('invalid perception publish stamp')
    if not 0 <= (now-packet['stamp_ns'])*1e-9 <= MAX_IMAGE_AGE_S:
        raise ContractError('camera measurement expired or future stamped')
    if packet.get('frame_id') != CAMERA_FRAME or packet.get('camera_frame_id') != CAMERA_FRAME:
        raise ContractError('measurement is not in camera frame')
    if packet.get('image_size') != [1280, 720]:
        raise ContractError('unexpected camera dimensions')
    key = packet.get('rgbd_frame_key')
    if not (isinstance(key, list) and len(key) == 3 and key[0] == packet['camera_epoch']
            and all(integer(v) for v in key[1:])):
        raise ContractError('invalid RGB-D source key')
    if any(packet.get(k, False) for k in ('motion_ready', 'grasp_ready', 'base_transform_valid')):
        raise ContractError('camera source must not grant base/motion qualification')
    if any(k.startswith(('base_', 'source_joint_', 'track_')) or k in ('mask', 'masks', 'color', 'depth') for k in packet):
        raise ContractError('robot/base state belongs to scene node')
    if type(packet.get('configuration_valid')) is not bool or type(packet.get('detection_valid')) is not bool:
        raise ContractError('missing qualification flags')
    detections = packet.get('detections')
    if not isinstance(detections, list) or len(detections) > MAX_DETECTIONS:
        raise ContractError('invalid bounded detection list')
    if type(packet.get('camera_geometry_valid')) is not bool:
        raise ContractError('missing camera qualification flag')
    for field in ('identity', 'compute', 'live_stage_ms', 'inference_speed_ms'):
        if packet.get(field) is not None and not isinstance(packet[field], dict):
            raise ContractError('invalid '+field+' object')
    for field in ('detector_ms', 'geometry_ms', 'processing_s', 'gpu_worker_ms', 'geometry_worker_ms'):
        if field in packet and (not number(packet[field]) or packet[field] < 0):
            raise ContractError('invalid arithmetic field '+field)
    for field in ('live_stage_ms', 'inference_speed_ms'):
        for key, value in (packet.get(field) or {}).items():
            if field == 'live_stage_ms' and key == 'debug_image_generated':
                if type(value) is not bool:
                    raise ContractError('invalid debug image flag')
            elif not number(value) or value < 0:
                raise ContractError('invalid numeric timing field')
    if packet.get('source_K') is not None and not vector(packet['source_K'], 9):
        raise ContractError('invalid camera intrinsics shape/value')
    if not packet['detection_valid']:
        if detections or packet.get('camera_geometry_valid'):
            raise ContractError('invalid image must not carry valid detections')
        encode(packet)
        return packet
    if not packet['configuration_valid']:
        raise ContractError('changed configuration cannot produce valid observations')
    K = packet.get('source_K')
    if not vector(K, 9) or K[0] <= 0 or K[4] <= 0 or abs(K[8]-1) > 1e-9:
        raise ContractError('invalid source intrinsics')
    roi = packet.get('inference_roi_xyxy')
    if not (isinstance(roi, list) and len(roi) == 4 and all(integer(x) for x in roi)
            and 0 <= roi[0] < roi[2] <= 1280 and 0 <= roi[1] < roi[3] <= 720):
        raise ContractError('invalid measurement ROI')
    if not isinstance(packet.get('identity'), dict) or not isinstance(packet.get('compute'), dict):
        raise ContractError('missing model identity/compute evidence')
    if packet.get('recognition_policy') not in (POLICY, DEPTH_POLICY):
        raise ContractError('missing full-candidate/ROI-class policy')
    if packet.get('candidate_scan_stamp_ns') != packet['stamp_ns'] or roi != [0, 0, 1280, 720]:
        raise ContractError('missing same-stamp full-image candidate scan')
    if any(type(packet.get(k)) is not bool for k in ('full_scene_coverage', 'classification_complete')):
        raise ContractError('missing scene/classification coverage flags')
    if not integer(packet.get('full_candidate_count')) or packet['full_candidate_count'] > MAX_DETECTIONS:
        raise ContractError('invalid full candidate count')
    crops = packet.get('classification_rois')
    if not isinstance(crops, list) or len(crops) > MAX_CROPS:
        raise ContractError('unbounded classification crops')
    for crop in crops:
        if not (isinstance(crop, list) and len(crop) == 4 and all(integer(x) for x in crop)
                and 0 <= crop[0] < crop[2] <= 1280 and 0 <= crop[1] < crop[3] <= 720
                and crop != [0, 0, 1280, 720]):
            raise ContractError('invalid classification crop')
    if packet['recognition_policy'] == DEPTH_POLICY:
        scan=packet.get('depth_scan')
        if (not isinstance(scan,dict) or scan.get('source') != 'aligned_depth_table_plane'
                or scan.get('units') != 'm' or scan.get('stamp_ns') != packet['stamp_ns']
                or scan.get('qualification_mode') != 'shadow'
                or scan.get('coverage_scope') != 'configured_workspace_current_view'
                or scan.get('pose_binding_verified') is not False
                or not isinstance(scan.get('view_id'),str) or not 1 <= len(scan['view_id']) <= 128):
            raise ContractError('invalid same-frame depth candidate provenance')
        sha=scan.get('config_sha256')
        if (not isinstance(sha,str) or re.fullmatch('[0-9a-f]{64}',sha) is None
                or packet['identity'].get('depth_config_sha256') != sha
                or packet['identity'].get('candidate_mode') != 'depth_shadow'
                or packet['identity'].get('depth_qualification_mode') != 'shadow'):
            raise ContractError('depth configuration identity mismatch')
        if any(type(scan.get(k)) is not bool for k in ('valid','coverage_valid','view_binding_valid')):
            raise ContractError('invalid depth quality flags')
        if scan['coverage_valid'] and not (scan['valid'] and scan['view_binding_valid']):
            raise ContractError('depth coverage lacks a valid bound reference')
        if packet['full_scene_coverage'] or packet['classification_complete']:
            raise ContractError('depth shadow cannot grant scene qualification')
        if (scan.get('candidate_network_calls') != 0 or type(scan['candidate_network_calls']) is not int
                or scan.get('roi_network_calls') != len(crops)
                or type(scan['roi_network_calls']) is not int
                or scan.get('candidate_count') != packet['full_candidate_count']
                or type(scan['candidate_count']) is not int
                or not integer(scan.get('board_count')) or scan['board_count'] > MAX_DETECTIONS
                or not integer(scan.get('roi_extra_count')) or scan['roi_extra_count'] > len(detections)
                or not number(scan.get('elapsed_ms')) or scan['elapsed_ms'] < 0):
            raise ContractError('invalid depth scan counts/timing')
        reasons=scan.get('reasons')
        if (not isinstance(reasons,list) or len(reasons)>32
                or not all(isinstance(s,str) and len(s)<=256 for s in reasons)
                or not isinstance(scan.get('diagnostics'),dict)):
            raise ContractError('invalid bounded depth diagnostics')
    elif 'depth_scan' in packet:
        raise ContractError('depth provenance mixed into model candidate policy')
    indices = set()
    for d in detections:
        if not isinstance(d, dict) or not integer(d.get('instance_index')):
            raise ContractError('invalid frame-local instance')
        if d['instance_index'] in indices:
            raise ContractError('duplicate frame-local instance')
        indices.add(d['instance_index'])
        if any(k.startswith(('base_', 'track_')) or k in ('mask', 'masks', 'color', 'depth') for k in d) or any(d.get(k, False) for k in ('motion_ready', 'grasp_ready')):
            raise ContractError('camera instance carries forbidden qualification')
        if not isinstance(d.get('class_name'), str) or not 1 <= len(d['class_name']) <= 128:
            raise ContractError('invalid class label')
        confidence = d.get('confidence')
        if (not vector(d.get('box_xyxy'), 4) or not number(confidence) or not 0 < confidence <= 1):
            raise ContractError('invalid box/confidence')
        box = d['box_xyxy']
        if not (0 <= box[0] < box[2] <= 1280 and 0 <= box[1] < box[3] <= 720):
            raise ContractError('box outside original image')
        if type(d.get('classification_valid')) is not bool or d.get('classification_stamp_ns') != packet['stamp_ns']:
            raise ContractError('missing same-image class evidence')
        if d['classification_valid']:
            crop = d.get('classification_roi_xyxy')
            if d.get('classification_source') != 'roi' or crop not in crops or d['class_name'] == 'unknown':
                raise ContractError('class did not originate in an admitted ROI')
            if not integer(d.get('class_id')):
                raise ContractError('invalid model class ID')
            if not (crop[0]+2 <= box[0] and crop[1]+2 <= box[1]
                    and box[2] <= crop[2]-2 and box[3] <= crop[3]-2):
                raise ContractError('class evidence touches crop edge')
        elif (d.get('classification_source') != 'unresolved' or d['class_name'] != 'unknown' or d.get('class_id') != -1
              or d.get('classification_roi_xyxy') is not None):
            raise ContractError('unresolved candidate carries a semantic label')
        if type(d.get('camera_geometry_valid')) is not bool or not d.get('detection_valid'):
            raise ContractError('invalid instance qualification')
        if packet['recognition_policy'] == DEPTH_POLICY:
            if (type(d.get('candidate_geometry_valid')) is not bool or type(d.get('roi_eligible')) is not bool
                    or d.get('candidate_geometry_source') not in ('aligned_depth','unadmitted_roi')
                    or not isinstance(d.get('candidate_geometry_reasons'),list)
                    or len(d['candidate_geometry_reasons'])>32
                    or not all(isinstance(s,str) and len(s)<=256 for s in d['candidate_geometry_reasons'])):
                raise ContractError('invalid depth instance geometry provenance')
            if d['camera_geometry_valid'] and not d['candidate_geometry_valid']:
                raise ContractError('depth candidate geometry rejection was bypassed')
            if d['classification_valid'] and (not d['roi_eligible'] or d['candidate_geometry_source']!='aligned_depth'):
                raise ContractError('class lacks admitted depth ROI candidate')
        outline = d.get('mask_outline_xy', [])
        if not isinstance(outline, list) or len(outline) > MAX_OUTLINE_POINTS or not all(vector(v, 2) for v in outline):
            raise ContractError('unbounded/invalid display outline')
        if d['camera_geometry_valid']:
            if not vector(d.get('camera_surface_center_m'), 3) or not .02 < d['camera_surface_center_m'][2] < .6:
                raise ContractError('invalid camera surface center')
            normal = d.get('camera_surface_normal')
            if not vector(normal, 3) or not .99 <= sum(v*v for v in normal) <= 1.01:
                raise ContractError('invalid camera surface normal')
            if not vector(d.get('center_pixel'), 2):
                raise ContractError('missing center pixel')
            if type(d.get('plane_rms_m')) not in (int, float) or not 0 <= d['plane_rms_m'] <= .002:
                raise ContractError('unqualified surface plane')
        d['camera_complete_center_target'] = bool(d['camera_geometry_valid'] and not d.get('touches_image_edge')
                                                and not d.get('touches_processing_edge'))
    complete = bool(packet['full_scene_coverage'] and all(d['classification_valid'] for d in detections))
    if packet['classification_complete'] != complete:
        raise ContractError('inconsistent classification completeness')
    if packet['full_scene_coverage'] and packet['full_candidate_count'] > len(detections):
        raise ContractError('full candidates silently discarded')
    encode(packet)
    return packet


class SequenceGuard:
    """Only one live perception epoch; late packets cannot resurrect retired sources."""
    def __init__(self):
        self.epoch = None
        self.sequence = self.stamp = -1
        self.keys = deque(maxlen=600)
        self.retired = deque(maxlen=32)

    def accept(self, packet, now):
        validate(packet, now)
        epoch = packet['source_epoch']
        changed = epoch != self.epoch
        if epoch in self.retired:
            raise ContractError('retired perception source')
        if not changed and (packet['sequence'] <= self.sequence or packet['stamp_ns'] <= self.stamp):
            raise ContractError('duplicate or reordered perception packet')
        key = tuple(packet['rgbd_frame_key'])
        if not changed and key in self.keys:
            raise ContractError('duplicate RGB-D pair')
        if changed:
            if self.epoch is not None:
                self.retired.append(self.epoch)
            self.keys.clear()
        self.epoch, self.sequence, self.stamp = epoch, packet['sequence'], packet['stamp_ns']
        self.keys.append(key)
        return changed


class RoiHint:
    """A scene hint can save work but never becomes a perception prerequisite."""
    def __init__(self):
        self.hint = None

    def receive(self, hint, now, source_epoch, published):
        try:
            if not isinstance(hint, dict) or hint.get('schema') != HINT_SCHEMA:
                raise ContractError('invalid hint schema')
            if hint.get('source_epoch') != source_epoch:
                raise ContractError('hint refers to another perception source')
            sequence = hint.get('based_on_sequence'); stamp = hint.get('based_on_stamp_ns')
            if not integer(sequence) or (sequence, stamp) not in published:
                raise ContractError('hint does not refer to a published observation')
            if not integer(hint.get('issued_ns'), 1) or not 0 <= (now-hint['issued_ns'])*1e-9 <= .3:
                raise ContractError('hint expired or future stamped')
            if self.hint and hint['issued_ns'] < self.hint['issued_ns']:
                raise ContractError('reordered ROI hint')
            if not 0 <= (now-stamp)*1e-9 <= MAX_IMAGE_AGE_S or type(hint.get('full_image')) is not bool:
                raise ContractError('hint source observation expired')
            roi = hint.get('roi_xyxy')
            if not hint['full_image'] and not (isinstance(roi, list) and len(roi) == 4
                    and all(integer(v) for v in roi) and 0 <= roi[0] < roi[2] <= 1280
                    and 0 <= roi[1] < roi[3] <= 720 and min(roi[2]-roi[0], roi[3]-roi[1]) >= 480):
                raise ContractError('invalid bounded ROI hint')
            self.hint = copy.deepcopy(hint)
            return True
        except (ContractError, KeyError, TypeError):
            return False

    def roi(self, now, source_epoch):
        h = self.hint
        if (not h or h['source_epoch'] != source_epoch or h['full_image']
                or not 0 <= (now-h['issued_ns'])*1e-9 <= .3
                or not 0 <= (now-h['based_on_stamp_ns'])*1e-9 <= MAX_IMAGE_AGE_S):
            return None
        return h['roi_xyxy']
