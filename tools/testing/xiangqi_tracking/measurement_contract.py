"""Bounded camera measurement and optional ROI-hint wire contracts; no ROS imports."""
import copy
import json
import math
import re
from collections import deque

CAMERA_FRAME = 'd405_color_optical_frame'
SCHEMA = 'xiangqi_camera_measurements/v1'
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
    for d in detections:
        if not isinstance(d, dict) or not integer(d.get('instance_index')):
            raise ContractError('invalid frame-local instance')
        if any(k.startswith(('base_', 'track_')) or k in ('mask', 'masks', 'color', 'depth') for k in d) or any(d.get(k, False) for k in ('motion_ready', 'grasp_ready')):
            raise ContractError('camera instance carries forbidden qualification')
        if not isinstance(d.get('class_name'), str) or not 1 <= len(d['class_name']) <= 128:
            raise ContractError('invalid class label')
        confidence = d.get('confidence')
        if (not vector(d.get('box_xyxy'), 4) or not number(confidence) or not 0 < confidence <= 1):
            raise ContractError('invalid box/confidence')
        if type(d.get('camera_geometry_valid')) is not bool or not d.get('detection_valid'):
            raise ContractError('invalid instance qualification')
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
