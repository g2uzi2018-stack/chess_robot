"""Same-image candidate coverage and ROI-only class evidence.

The existing segmentation model still computes class scores in its full-image
pass. Those scores admit proposals at the unchanged threshold; their labels
are deliberately not class evidence. This is not a new objectness detector.
"""
import hashlib
import json
import math
import time
from pathlib import Path

import numpy as np

POLICY = 'full_candidates_roi_classes/v1'
DEPTH_POLICY = 'depth_candidates_roi_classes/shadow-v1'
MAX_CROPS = 8
MAX_INSTANCES = 300


def overlap(a, b):
    lo = np.maximum(a[:2], b[:2]); hi = np.minimum(a[2:], b[2:])
    intersection = np.maximum(hi-lo, 0).prod()
    return float(intersection/max(1e-9, np.prod(a[2:]-a[:2])+np.prod(b[2:]-b[:2])-intersection))


def crop_for(boxes, width, height):
    """Existing observation ROI shape, with a per-candidate fallback for wide scenes."""
    boxes = np.asarray(boxes, dtype=float)
    lo = boxes[:, :2].min(0); hi = boxes[:, 2:4].max(0)
    size = max(480, int(math.ceil((max(hi-lo)+160)/32))*32)
    if size > min(width, height):
        return None
    cx, cy = np.round((lo+hi)/2/32)*32
    x = max(0, min(width-size, int(cx-size/2)))
    y = max(0, min(height-size, int(cy-size/2)))
    return [x, y, x+size, y+size]


def plan_crops(boxes, width, height, hint=None):
    if not (isinstance(hint, (list, tuple)) and len(hint) == 4
            and all(type(x) is int for x in hint)
            and 0 <= hint[0] < hint[2] <= width and 0 <= hint[1] < hint[3] <= height
            and min(hint[2]-hint[0],hint[3]-hint[1]) >= 480
            and list(hint) != [0,0,width,height]):
        hint=None
    if hint is not None and boxes and all(
            hint[0]+2 < b[0] and hint[1]+2 < b[1] and
            b[2] < hint[2]-2 and b[3] < hint[3]-2 for b in boxes):
        return [list(hint)]
    if not boxes:
        return [list(hint)] if hint is not None else []
    combined = crop_for(boxes, width, height)
    if combined is not None:
        return [combined]
    crops = []
    for box in boxes:
        crop = crop_for([box], width, height)
        if crop is not None and crop not in crops:
            crops.append(crop)
    return crops


def items(packet, source, crop=None):
    """Return geometry in the original image plane; do not expand full masks yet."""
    offset = np.array(packet['roi_xyxy'][:2]*2)
    predictions = packet['predictions']
    x1,y1,x2,y2=packet['roi_xyxy']
    if packet['masks'] is not None and packet['masks'].shape != (len(predictions),y2-y1,x2-x1):
        raise ValueError('ROI mask/image ownership mismatch')
    groups = []
    for i in np.argsort(-predictions[:, -2]):
        box = predictions[i, :4]+offset
        # ROI observations are fused only after their crop completeness is
        # known. Pre-deduplicating here would conflate an edge fragment with
        # complete conflicting class evidence in the same crop.
        old = next((g for g in groups if overlap(box, g['box']) > .65), None) if source != 'roi' else None
        name = packet['names'][int(predictions[i, -1])]
        if old is not None:
            continue
        groups.append(dict(box=box, confidence=float(predictions[i, -2]),
                           class_id=int(predictions[i,-1]),
                           name=name if source == 'roi' else 'unknown',
                           valid=source == 'roi', crop=crop,
                           mask=None if packet['masks'] is None else packet['masks'][i],
                           mask_roi=packet['roi_xyxy'], source=source))
    return groups


def merge_roi(observations):
    """Complete observations supersede edge fragments, never true conflicts.

    Sorting makes the result independent of crop execution order. Incomplete
    edge proposals remain visible unless a complete observation covers them.
    Two complete contradictory labels stay unresolved even if another crop
    later agrees with one of them.
    """
    def order(d):
        b=d['box']
        return (not d['complete'], -d['confidence'], -float(np.prod(b[2:]-b[:2])),
                tuple(b), tuple(d['crop']), d['class_id'])
    merged=[]
    for item in sorted(observations,key=order):
        item['conflict']=False
        # Retain all complete semantic views. A representative ROI box can
        # overlap a full candidate while its crop does not cover that full
        # candidate's geometry; provenance is selected after association.
        item['views']=[{k:item[k] for k in ('name','class_id','confidence','crop')}]
        if not item['complete']:item['views']=[]
        matches=[d for d in merged if overlap(d['box'],item['box'])>.65]
        if not matches:
            merged.append(item);continue
        if len(matches)>1:
            # A proposal joining two distinct current objects cannot decide
            # their identity. Keep all proposals explicitly unresolved.
            for d in matches:d['valid']=False;d['conflict']=True
            item['valid']=False;item['conflict']=True;merged.append(item)
            continue
        old=matches[0]
        if old['complete'] and item['complete']:
            old['views'].extend(item['views'])
            if old['name']!=item['name'] or old['conflict']:
                old['valid']=False;old['conflict']=True
        # An edge fragment does not add or veto class evidence when the
        # geometrically matching complete proposal has already been retained.
    return merged


def recognize(color, detect, hint=None):
    """Full scan and bounded ROI passes on this exact image, never cached pixels."""
    started = time.monotonic()
    height, width = color.shape[:2]
    full = detect(color, None)
    candidates = items(full, 'full_candidate')
    return _recognize_candidates(color, detect, full, candidates, hint, started)


def _recognize_candidates(color, detect, full, candidates, hint, started,
                          policy=POLICY, scan_complete=True):
    """Shared ROI fusion; candidate geometry always belongs to this frame."""
    height, width = color.shape[:2]
    classifiable = candidates if policy == POLICY else [d for d in candidates if d.get('roi_eligible')]
    planned = plan_crops([d['box'] for d in classifiable], width, height,
                         hint if classifiable or policy == POLICY else None)
    crops = planned[:MAX_CROPS]
    observations = []
    packets = [full]
    for crop in crops:
        packet = detect(color, crop)
        if packet['names'] != full['names']:
            raise ValueError('ROI/full model class mapping changed within one frame')
        packets.append(packet)
        for item in items(packet, 'roi', crop):
            # Keep crop-edge objects visible, but do not qualify truncated labels.
            b = item['box']
            item['complete'] = bool(b[0] >= crop[0]+2 and b[1] >= crop[1]+2
                                    and b[2] <= crop[2]-2 and b[3] <= crop[3]-2)
            item['valid'] = item['complete']
            observations.append(item)
    classified=merge_roi(observations)
    # Link proposals one-to-one by overlap. Ambiguous links are unresolved,
    # rather than inheriting a full-image class or dropping a candidate.
    links = [[j for j, r in enumerate(classified) if overlap(c['box'], r['box']) > .3]
             if policy == POLICY or c.get('roi_eligible') else []
             for c in candidates]
    used = set()
    output = []
    for c, choices in zip(candidates, links):
        unique = len(choices) == 1 and sum(choices[0] in row for row in links) == 1
        if unique:
            j = choices[0]; used.add(j)
            # The full pass guards scene position. Taking the ROI box/mask
            # here could hide a full-image movement behind a stale-looking
            # overlapping ROI prediction. Only semantic evidence is borrowed.
            r=classified[j]
            b=c['box']
            compatible=[v for v in r['views'] if v['crop'][0]+2 <= b[0]
                        and v['crop'][1]+2 <= b[1] and b[2] <= v['crop'][2]-2
                        and b[3] <= v['crop'][3]-2]
            if r['valid'] and compatible:
                view=min(compatible,key=lambda v:(-v['confidence'],tuple(v['crop']),v['class_id']))
                output.append(dict(c,name=view['name'],class_id=view['class_id'],
                                   confidence=view['confidence'],valid=True,crop=view['crop']))
            else:
                output.append(c)
        else:
            output.append(c)
            for j in choices:
                classified[j]['valid'] = False
    # ROI-only detections matter too: full-image misses must not silently erase
    # a target that a classification crop sees.
    extras = [r for j, r in enumerate(classified) if j not in used]
    if policy == DEPTH_POLICY:
        # A model proposal absent from the depth scan cannot establish its own
        # workspace admission. Preserve it as unknown, including table-exterior
        # observations; do not silently call this a repaired depth scan.
        extras = [dict(r, valid=False) for r in extras]
    output.extend(extras)
    overflow = len(output) > MAX_INSTANCES
    if overflow:
        # No partial set is publishable as complete. The event remains bounded.
        output = []
    predictions = []
    masks = []
    evidence = []
    names = dict(full['names']);names[-1] = 'unknown'
    for i, d in enumerate(output):
        predictions.append([*d['box'], d['confidence'], d['class_id'] if d['valid'] else -1])
        mask = np.zeros((height, width), np.uint8)
        if d['mask'] is not None:
            x1, y1, x2, y2 = d['mask_roi']
            if d['mask'].shape != (y2-y1, x2-x1):
                raise ValueError('ROI mask/image ownership mismatch')
            mask[y1:y2, x1:x2] = d['mask']
        masks.append(mask)
        evidence.append(dict(classification_valid=d['valid'],
                             classification_source='roi' if d['valid'] else 'unresolved',
                             classification_roi_xyxy=d['crop'] if d['valid'] else None))
        if policy == DEPTH_POLICY:
            evidence[-1].update(candidate_geometry_valid=bool(d.get('candidate_complete',False)),
                                candidate_geometry_source='aligned_depth' if d['source']=='depth_candidate' else 'unadmitted_roi',
                                candidate_geometry_reasons=d.get('candidate_reasons',['ROI proposal lacks depth workspace admission']),
                                roi_eligible=bool(d.get('roi_eligible',False)))
    coverage = bool(scan_complete and not overflow and len(planned) <= MAX_CROPS)
    complete = bool(coverage and all(d['valid'] for d in output))
    timings = {}
    for packet in packets:
        for key, value in packet['inference_speed_ms'].items():
            timings[key] = timings.get(key, 0.)+value
    compute=full['compute']
    if policy == DEPTH_POLICY:
        compute=dict(compute, network_executed=len(packets)>1)
        if len(packets)>1:
            compute.update(packets[-1]['compute'])
        compute['candidate_backend']='cpu_depth'
    return dict(full, predictions=np.asarray(predictions, dtype=float).reshape(-1, 6),
                masks=np.asarray(masks, dtype=np.uint8) if masks else None, names=names,
                compute=compute,
                roi_xyxy=[0, 0, width, height], classification_evidence=evidence,
                recognition_policy=policy, full_scene_coverage=coverage,
                full_candidate_count=len(candidates), classification_complete=complete,
                classification_rois=crops, inference_speed_ms=timings,
                detector_s=time.monotonic()-started)


def load_depth_configuration(path):
    """Explicit, immutable current-view experiment configuration; no fallback."""
    if not path:
        raise ValueError('depth_shadow requires XIANGQI_DEPTH_CONFIG')
    try:
        raw = Path(path).read_bytes()
    except OSError as error:
        raise ValueError('depth configuration cannot be read') from error
    try:
        config = json.loads(raw)
    except (ValueError, UnicodeError) as error:
        raise ValueError('invalid depth configuration JSON') from error
    if not isinstance(config, dict) or config.get('qualification_mode') != 'shadow':
        raise ValueError('depth configuration must explicitly select qualification_mode=shadow')
    view = config.get('view_binding')
    if (not isinstance(view, dict) or not isinstance(view.get('view_id'), str)
            or not 1 <= len(view['view_id']) <= 128
            or not isinstance(view.get('camera_epoch'), str) or not view['camera_epoch']
            or view.get('image_size') != [1280, 720]):
        raise ValueError('depth configuration requires a current-view/camera/image-size binding')
    K = view.get('K')
    if (not isinstance(K, list) or len(K) != 9
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in K)
            or K[0] <= 0 or K[4] <= 0 or K[8] != 1):
        raise ValueError('invalid bound depth camera intrinsics')
    detector = config.get('detector')
    if not isinstance(detector, dict) or detector.get('image_size') != view['image_size']:
        raise ValueError('depth detector configuration/image size missing')
    # Instantiate now so malformed parameters fail startup instead of silently
    # reverting to a full-image model. The core owns detector parameter checks.
    from depth_candidates import DepthCandidateConfig
    try:
        detector_config = DepthCandidateConfig(**detector)
    except (TypeError, ValueError) as error:
        raise ValueError('invalid depth detector configuration') from error
    import cv2
    masks=[]
    for key in ('workspace_polygon_xy','table_seed_polygon_xy'):
        try:
            polygon=np.asarray(detector.get(key),dtype=float)
        except (TypeError,ValueError) as error:
            raise ValueError('invalid '+key) from error
        if (polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 2
                or not np.isfinite(polygon).all() or np.any(polygon < 0)
                or np.any(polygon[:,0] >= 1280) or np.any(polygon[:,1] >= 720)
                or abs(cv2.contourArea(polygon.astype(np.float32))) < 1):
            raise ValueError('invalid '+key)
        mask=np.zeros((720,1280),np.uint8)
        cv2.fillPoly(mask,[np.round(polygon).astype(np.int32)],1)
        masks.append(mask.astype(bool))
    if int(masks[1].sum()) < detector_config.min_seed_pixels or np.any(masks[1] & ~masks[0]):
        raise ValueError('bare-table seed must have sufficient area inside workspace')
    return dict(config=config, detector_config=detector_config,
                config_sha256=hashlib.sha256(raw).hexdigest(), path=str(Path(path).resolve()))


def recognize_depth(color, depth, metadata, detect, names, identity, compute,
                    configuration, hint=None):
    """Depth-only candidate scan + ROI network calls, explicitly shadow-only.

    Pixel workspaces are view-dependent. Camera epoch/K checks catch some
    changes but cannot prove an unchanged wrist camera pose; therefore this
    experimental policy never grants scene/snapshot coverage qualification.
    """
    from depth_candidates import detect_depth_candidates
    started = time.monotonic()
    height, width = color.shape[:2]
    if depth.shape != (height, width) or depth.dtype != np.float32:
        raise ValueError('depth candidates require same-frame aligned float32 metre depth')
    view = configuration['config']['view_binding']
    K = np.asarray(metadata['K'], dtype=float).reshape(3, 3)
    bound_K = np.asarray(view['K'], dtype=float).reshape(3, 3)
    binding_valid = bool(view['image_size'] == [width, height]
                         and metadata.get('camera_epoch') == view['camera_epoch']
                         and np.allclose(K, bound_K, rtol=0, atol=1e-8))
    if binding_valid:
        scan = detect_depth_candidates(depth, K, configuration['detector_config'])
    else:
        scan = dict(valid=False, coverage_valid=False, candidates=[], board_regions=[],
                    reasons=['current-view camera/image/intrinsics binding mismatch'], diagnostics={})
    scan_elapsed_ms=(time.monotonic()-started)*1000
    candidates = []
    for item in scan['candidates']:
        mask = item['mask']
        if mask.shape != (height, width):
            raise ValueError('depth candidate mask/frame shape mismatch')
        candidates.append(dict(box=np.asarray(item['box_xyxy'], dtype=float),
                               confidence=1., class_id=-1, name='unknown', valid=False,
                               crop=None, mask=mask, mask_roi=[0, 0, width, height],
                               source='depth_candidate',
                               candidate_complete=bool(item['candidate_complete']),
                               roi_eligible=bool(scan['valid'] and item.get('roi_eligible',item['candidate_complete'])),
                               candidate_reasons=[str(r)[:256] for r in item.get('reasons',[])][:32]))
    full = dict(names=dict(names), compute=compute, identity=identity,
                roi_xyxy=[0, 0, width, height], image_size=[width, height],
                detector_s=0., inference_speed_ms={})
    result = _recognize_candidates(color, detect, full, candidates,
                                   hint if scan['valid'] else None, started,
                                   policy=DEPTH_POLICY, scan_complete=False)
    # Copy only bounded small diagnostic values; full depth/support/masks stay
    # in the worker, never enter the inter-node JSON wire packet.
    result['depth_scan'] = dict(source='aligned_depth_table_plane', units='m',
        stamp_ns=metadata['stamp_ns'], config_sha256=configuration['config_sha256'],
        qualification_mode='shadow', view_id=view['view_id'], view_binding_valid=binding_valid,
        pose_binding_verified=False, coverage_scope='configured_workspace_current_view',
        valid=bool(scan['valid']), coverage_valid=bool(scan['coverage_valid']),
        reasons=[str(reason)[:256] for reason in scan.get('reasons', [])][:32],
        table_plane=scan.get('table_plane'), diagnostics=scan.get('diagnostics', {}),
        board_count=len(scan.get('board_regions', [])),
        candidate_count=len(candidates), roi_extra_count=max(0, len(result['predictions'])-len(candidates)),
        candidate_network_calls=0, roi_network_calls=len(result['classification_rois']),
        elapsed_ms=scan_elapsed_ms)
    return result
