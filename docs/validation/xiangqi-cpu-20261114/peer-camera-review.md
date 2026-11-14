DONE

## Review status
Finished. Isolated snapshot reviewed as specified (`node.cpp`, `calibration.cpp`, `calibration.hpp` only). No edits, tests, or network.

## Correctness assessment
**No blockers.** Node-owned ROS Image reuse, source-stride wrapping, nearest-map fused depth (LUT + OOB→0→NaN), evidence Z16/timestamps, and unchanged SDK align + linear color remap are consistent with the stated `remap` + `convertTo` + `setTo` contract on packed destinations and padded sources.

## Findings

### Ownership (reusable ROS Image buffers, const-ref publish)
- Startup sizes packed ROS buffers once: `step = width*3` / `width*sizeof(float)`, `data.resize(step*height)` (`node.cpp:96-101`). `is_bigendian` left at default 0.
- Per-frame `cv::Mat` views wrap those vectors with default `AUTO_STEP` (`node.cpp:154-155`), matching the packed `step` values. `remap_depth_metres` calls `metres.create(map.size(), CV_32FC1)` (`calibration.cpp:44`); OpenCV `create` is a no-op when size/type already match, so the write stays in `depth_message_.data`. Color `cv::remap` into `bgr` is the same size-coupling.
- Publish is `color_pub_->publish(color_message_)` / `depth_pub_->publish(depth_message_)` (`node.cpp:195-201`), i.e. `publish(const Image&)`. Comment at `node.cpp:152-153` matches rclcpp: that overload copies into transport; members are not given out as shared/loaned buffers. Reuse after return is sound for this API.
- Failure path `invalid()` (`node.cpp:115-120`, `208-210`) publishes only `FrameInfo`, not the image members, so a throw mid-remap cannot emit a half-written reusable buffer.

Residual (not demonstrated on the stated 720p Jetson path): maps come from actual color intrinsics (`node.cpp:102-103`) while Mats/messages use parameter `width_`/`height_` (`node.cpp:146-147`, `96-101`). The pair check at `node.cpp:137-138` does not also require `color.get_width()==width_`. A mismatch would make OpenCV `create`/`remap` detach from the node-owned vectors. `enable_stream(..., width_, height_, ...)` plus the logged actual profile (`node.cpp:70-71`, `88-93`) is what keeps them equal.

### Stride (padded RealSense rows vs packed ROS dest)
- Sources wrap SDK pointers with `get_stride_in_bytes()` (`node.cpp:146-147`). Destinations are packed (no extra row padding).
- Fused sampler uses `source.ptr<uint16_t>(sy)[sx]` (`calibration.cpp:54`), which is row-step then pixel index, so extra SDK row bytes are skipped. `metres.ptr<float>(y)` (`calibration.cpp:49`) uses the destination step (packed ROS buffer when create is a no-op).
- This is the correct split for “padded rows match original remap”: OpenCV `remap` also honors `Mat::step`; the LUT path does too.

### Border and zero / NaN semantics (256KiB LUT + fused nearest remap)
- Depth LUT: 65536 `uint16` units → `convertTo(CV_32F, scale)` then `lookup[0] = NaN` (`calibration.cpp:35-40`). Size is 65536×4 = 262144 bytes (256KiB). Index 0 is the only special value; that is the same as `convertTo` then `setTo(NaN)` on zeros.
- Nearest map: `convertMaps(..., CV_16SC2, true)` (`node.cpp:108`), separate from linear color maps (`node.cpp:104-106`). Comment at `node.cpp:104-105` is accurate: sharing the linear table would interpolate holes.
- Fused OOB test (`calibration.cpp:51-55`) is the OpenCV 16SC2 nearest trick: negative `sx`/`sy` become large `unsigned` and miss the in-bounds load, substitute `0`, then `values[0]` is NaN. In-bounds Z16 holes (0) also hit `values[0]`. Matches `BORDER_CONSTANT` (0) + `setTo(NaN)` on zeros.
- Color remap remains `INTER_LINEAR` + `BORDER_CONSTANT` on the linear tables (`node.cpp:157`). SDK `align_(RS2_STREAM_COLOR)` still runs before remap (`node.cpp:21`, `135`).

### Evidence save
- Trigger: `count_+1==30` or last `max_frames_` frame (`node.cpp:150-151`). Color for disk uses the same linear remap into the node-owned `bgr` (`node.cpp:156-157`). Depth evidence is a separate `INTER_NEAREST` `remap` into a new Z16 `Mat`, not the metres buffer (`node.cpp:160-162`, `204`). Matches the comment that evidence keeps Z16.
- `save()` writes `color.png`, `depth_z16.png`, and JSON (`node.cpp:213-232`) after publish returns, so it reads the reusable color buffer only post-copy.
- `cv::imwrite` of 16UC1 PNG is the remapped Z16 including holes as 0, not NaN metres.

### Timestamps
- ROS `Header.stamp` is `now()` taken on host receipt, before `align_.process` (`node.cpp:133-135`, `163`). Same `header` is copied to `FrameInfo`, `CameraInfo`, and both images (`node.cpp:164-165`, `180`, `196`, `200`). Evidence JSON `stamp_sec` / `stamp_nanosec` come from that `info.header.stamp` (`node.cpp:224`). `stamp_nanosec` stored as `double` still exactly represents `uint32` nanoseconds.
- Device time is `color.get_timestamp()` plus domain string (`node.cpp:167-168`, `223`); RGB-D domains must match and capture times must be within `1500/fps` ms (`node.cpp:141-143`). Depth device time is checked, not stored.
- Stale discard uses a different clock on purpose: processing budget vs `stale_s_` from tick start (`node.cpp:176-177`) returns through `invalid()` without publish/save/`count_++`. `last_frame_` is steady time at successful completion (`node.cpp:191`).

### CameraInfo vs maps (related, not a hole/stride bug)
- Rectify maps: startup color stream intrinsics (`node.cpp:102-103`). Published `CameraInfo`: per-frame `pinhole_intrinsics(depth.get_profile()...)` after align (`node.cpp:144-145`, `179-185`). `pinhole_intrinsics` zeros coeffs and sets `RS2_DISTORTION_NONE` (`calibration.cpp:8-11`; header `calibration.hpp:6-7`). With align-to-color, the aligned depth video profile is the color plane; D=0 plumb_bob then describes the remapped images. Inverse-Brown source coeffs are not copied into ROS `d`.

## Remaining work
None for this review. Snapshot left unchanged.
