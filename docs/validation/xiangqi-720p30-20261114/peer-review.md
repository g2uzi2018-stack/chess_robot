DONE

# Assessment

The live limiter is **source-message delivery and stamp pairing before GPU work**, not camera FPS and not the repair/postprocess path from the first report.

Main’s dual-GPU-candidate counters: CameraInfo and FrameInfo **30.02 fps** each; color **25.98 fps**; depth **26.31 fps**; matched submitted **23.01 fps**; `complete_frames_skipped_for_latest` delta **0**. Small messages keep the 30 Hz camera clock. Large images do not. Complete 4-tuples are rarer still. Zero skip-for-latest means the 23 vs 26 gap is **incomplete stamp sets**, not the parent throwing away extra complete frames because the pipeline was busy.

Do not raise camera FPS. Transport is under main investigation; this note only records facts from that measurement plus the files already in tree.

Processing findings from the first handoff (median extra 640-square repair, native-mask postprocess, copies) remain valid **after** a frame is submitted. They cannot create 30 fps recognition while only ~23 complete RGB-D tuples are submitted.

---

# Outcome

Finished the read-only report with the new input-loss facts. No further search. No code edits.

---

# Evidence

## 1. Arrival rates (main measurement; not re-run here)

| Stream | Rate | Size class |
|---|---|---|
| CameraInfo `/vision/color/camera_info` | 30.02 fps | small |
| FrameInfo `/vision/frame_info` | 30.02 fps | small |
| color `/vision/color/image_rect` | 25.98 fps | ~2.76 MiB BGR8 1280×720 |
| depth `/vision/aligned_depth/image_rect` | 26.31 fps | ~3.52 MiB 32FC1 1280×720 |
| matched submitted | 23.01 fps | all four stamps present |

`25.98/30.02 × 26.31/30.02 × 30.02 ≈ 22.8 fps` if color and depth drops are independent. That matches **23.01** submitted. Correlated drops would have submitted ≈ min(color, depth) ≈ 26 fps. The pairing loss is therefore mostly **uncorrelated color vs depth delivery**, not GPU queueing.

`complete_frames_skipped_for_latest` delta 0 agrees: when a 4-tuple exists, it is submitted; older complete frames are not being discarded for a newer one. This workspace’s `perception_node.py` has no such named counter; the candidate added it. Equivalent behavior here is `t = max(eligible)` then `last = t` (`perception_node.py` 199–203).

## 2. Pairing code (this tree)

`perception_node.py` 75–85 caches each kind by stamp, cap 8. Submit path 199–216:

- `common = intersection` of color, depth, info, frame stamps
- `eligible = common` with `t > last` and age ≤ 0.2 s
- takes **only** `max(eligible)`
- `qos_profile_sensor_data` BEST_EFFORT on all four subscriptions (97–101)

A color frame whose depth (or vice versa) never arrives at the same stamp never enters `common`. BEST_EFFORT plus a depth-8 cache cannot recover it. That is pairing loss **preceding** `pipeline.submit`.

## 3. Fast DDS profile mismatch (this tree)

`src/chess_robot_vision/config/large_images_fastdds.xml`: SHM `maxMessageSize` 8 MiB, `segment_size` 64 MiB, UDP whitelist 127.0.0.1, `useBuiltinTransports` false.

Who exports `FASTRTPS_DEFAULT_PROFILES_FILE` to that XML:

- Camera/preview: `src/chess_robot_preview/scripts/preview_service.sh` line 8
- Tracking renderer: `tools/testing/xiangqi_tracking/start_tracking_renderer.sh` line 10
- Documented as camera/preview/renderer only: `src/chess_robot_vision/README.md` §7; `docs/validation/xiangqi-camera-30hz-20261031.md` (default participant SHM ~512 KiB vs color ~2.64 MiB / depth ~3.52 MiB; large-image profile applied to camera and overlay, **not** global DDS)

Who does **not**:

- `tools/testing/xiangqi_tracking/start_perception.sh` — ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY / thread caps only; **no** `FASTRTPS_DEFAULT_PROFILES_FILE`
- `split_supervisor.py` passes `os.environ.copy()` through; it does not add the profile either

Publisher on the large-image participant (`useBuiltinTransports=false`) and subscriber on Humble default transports is the configuration the 2026-10-31 camera note already identified as causing large-image delivery loss. Perception is still on that subscriber side.

## 4. What this does to the first report

Original live40s: camera 30.00 fps hardware 0 gaps, recognition **22.08 fps**. That is now the same shape as 30 → ~26 image arrival → ~23 matched → ~22 published, not “GPU 41 ms alone.” GPU-worker p50 41.26 ms still caps **processing** at ~24 fps **if** 30 complete frames were offered; with 23.01 submitted, processing is not the first missing 7 fps.

Repair/postprocess/copy items stay useful for latency and for headroom **after** transport delivers 30 Hz 4-tuples. They are not the next experiment while color/depth are in the mid-20s.

---

# Remaining work

- Main continues transport (perception Fast DDS profile vs camera, SHM vs UDP, whether both ends must share `large_images_fastdds.xml`). This review does not prescribe a patch beyond the file facts above.
- After color, depth, info, and FrameInfo are all ~30 fps **and** matched submitted ~30 with skip-for-latest still ~0, re-check GPU p50 / repair histogram. Only then do ranked processing fixes from the first handoff apply to the 30 fps recognition target.
- No workspace edits. Git remains with the primary.
