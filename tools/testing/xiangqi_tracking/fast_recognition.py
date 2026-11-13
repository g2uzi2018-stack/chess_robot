"""One full mask pass, batched glyphs, and same-frame missing-mask repair."""
import time
import numpy as np
from candidate_roi import MAX_CROPS, MAX_INSTANCES
from piece_classifier import crop_window

FAST_POLICY = 'full_masks_batched_glyphs/v1'

def overlap(a, b):
    ax1,ay1,ax2,ay2=map(float,a);bx1,by1,bx2,by2=map(float,b)
    inter=max(0.,min(ax2,bx2)-max(ax1,bx1))*max(0.,min(ay2,by2)-max(ay1,by1))
    return inter/max(1e-9,(ax2-ax1)*(ay2-ay1)+(bx2-bx1)*(by2-by1)-inter)

def overlap_matrix(a,b):
    a=np.asarray(a,dtype=float).reshape(-1,4);b=np.asarray(b,dtype=float).reshape(-1,4)
    wh=np.maximum(np.minimum(a[:,None,2:],b[None,:,2:])-np.maximum(a[:,None,:2],b[None,:,:2]),0.)
    inter=wh[:,:,0]*wh[:,:,1]
    aa=np.prod(a[:,2:]-a[:,:2],axis=1);bb=np.prod(b[:,2:]-b[:,:2],axis=1)
    return inter/np.maximum(aa[:,None]+bb[None,:]-inter,1e-9)

def unique_indices(predictions):
    keep=[]
    pairwise=overlap_matrix(predictions[:,:4],predictions[:,:4])
    for i in np.argsort(-predictions[:,4]):
        if not any(pairwise[i,k]>.65 for k in keep):
            keep.append(int(i))
    return keep


def unique_masks(packet):
    return [dict(box=packet['predictions'][i,:4],confidence=float(packet['predictions'][i,4]),
                 mask=packet['masks'][i] if packet['masks'] is not None else None,
                 mask_index=i,mask_roi=packet['roi_xyxy'],mask_bounds=packet['mask_bounds_xyxy'][i],source='full_mask')
            for i in unique_indices(packet['predictions'])]

def recognize_fast(color, experiment, classifier, min_conf=.5):
    started = time.monotonic()
    height, width = color.shape[:2]
    full = experiment.detect(color,conf=.1)
    objects = unique_masks(full)
    candidates = experiment.detect_lite(color)
    missing = []
    assignments = []
    initial_count=len(objects)
    links_to_masks=overlap_matrix(candidates['predictions'][:,:4],[o['box'] for o in objects])
    for ci,prediction in enumerate(candidates['predictions']):
        box = prediction[:4]
        linked = np.flatnonzero(links_to_masks[ci]>.3).tolist()
        linked.extend(i for i in range(initial_count,len(objects)) if overlap(box,objects[i]['box'])>.3)
        if linked:
            assignments.append(linked)
            continue
        assignments.append([len(objects)])
        item = dict(box=box, confidence=float(prediction[4]), mask=None,
                    mask_roi=None, source='candidate')
        objects.append(item)
        if 2 <= box[0] and 2 <= box[1] and box[2] <= width-2 and box[3] <= height-2:
            missing.append(item)
    coverage = len(objects) <= MAX_INSTANCES
    if not coverage:
        objects = []; missing = []; assignments = []
    # One bounded batch repairs missing full-view masks; no repeated class votes.
    crops = []
    for item in missing:
        cx, cy = (item['box'][:2]+item['box'][2:])/2
        size = 480
        x = int(max(0, min(width-size, cx-size/2)))
        y = int(max(0, min(height-size, cy-size/2)))
        crop = [x, y, x+size, y+size]
        if crop not in crops and len(crops) < MAX_CROPS:
            crops.append(crop)
    repair_packets = experiment.detect_rois(color, crops) if crops else []
    for item in missing:
        matches = []
        for packet in repair_packets:
            dx,dy,x2,y2 = packet['roi_xyxy']
            box=item['box']
            if not (dx+2<=box[0] and dy+2<=box[1] and box[2]<=x2-2 and box[3]<=y2-2):
                continue
            if packet['masks'] is None:
                continue
            for i,pred in enumerate(packet['predictions']):
                box = pred[:4]+[dx,dy,dx,dy]
                if overlap(box,item['box'])>.5:
                    matches.append((overlap(box,item['box']),i,packet))
        if matches:
            _,i,packet = max(matches,key=lambda item:item[0])
            item.update(mask=packet['masks'][i],mask_roi=packet['roi_xyxy'],mask_bounds=packet['mask_bounds_xyxy'][i],source='repair_mask')
    probabilities = classifier.classify(color, [o['box'] for o in objects])
    predictions=[]; evidence=[]
    reuse_full=bool(objects) and all(item['source']=='full_mask' for item in objects)
    if reuse_full:
        masks=full['masks'];mask_bounds=full['mask_bounds_xyxy']
        mask_indices=[item['mask_index'] for item in objects]
    else:
        mask_bounds=[item.get('mask_bounds') for item in objects]
        mask_height=max([b[3]-b[1] for b in mask_bounds if b is not None] or [1])
        mask_width=max([b[2]-b[0] for b in mask_bounds if b is not None] or [1])
        masks=np.zeros((len(objects),mask_height,mask_width),np.uint8)
        mask_indices=list(range(len(objects)))
    for index,(item,probs) in enumerate(zip(objects,probabilities)):
        box=item['box']; cls=int(probs.argmax()); confidence=float(probs[cls])
        complete=bool(2<=box[0] and 2<=box[1] and box[2]<=width-2 and box[3]<=height-2)
        valid=bool(complete and confidence>=min_conf)
        predictions.append([*box,confidence,cls if valid else -1])
        if not reuse_full and item['mask'] is not None:
            left,top,right,bottom=item['mask_bounds']
            masks[index,:bottom-top,:right-left]=item['mask'][:bottom-top,:right-left]
        evidence.append(dict(classification_valid=valid,
            classification_source='piece_classifier' if valid else 'unresolved',
            classification_roi_xyxy=crop_window(box) if valid else None,
            classification_reason='classified_from_current_piece_crop' if valid else
                'candidate_touches_image_edge' if not complete else 'piece_classifier_uncertain',
            geometry_source='same_frame_full_mask' if item['source']=='full_mask' else
                'same_frame_roi_mask' if item['mask'] is not None else 'missing_roi_mask',
            geometry_roi_xyxy=item['mask_roi'],candidate_box_xyxy=box.tolist(),
            piece_classifier_confidence=confidence))
    compute=dict(full['compute'],candidate_ms=candidates['detector_s']*1000,
        candidate_network_calls=1,roi_count=len(crops),piece_classifier_sha256=classifier.sha256,
        candidate_artifact_sha256=experiment.candidate_sha256,
        raw_candidate_boxes=candidates['predictions'][:,:4].tolist(),
        candidate_assignments=[[objects[i]['box'].tolist() for i in links] for links in assignments],
        classifier_min_conf=min_conf,full_mask_network_calls=1)
    return dict(full,predictions=np.asarray(predictions,dtype=float).reshape(-1,6),
        masks=masks if len(objects) else None,mask_indices=mask_indices,mask_bounds_xyxy=mask_bounds,
        names=dict(full['names'])|{-1:'unknown'},compute=compute,
        identity=dict(full['identity'],piece_classifier_sha256=classifier.sha256),
        classification_evidence=evidence,recognition_policy=FAST_POLICY,
        full_scene_coverage=coverage,full_candidate_count=len(objects),
        classification_complete=coverage and all(e['classification_valid'] for e in evidence),
        classification_rois=crops,detector_s=time.monotonic()-started)
