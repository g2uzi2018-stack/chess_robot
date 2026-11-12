# Pinned official C++ V4 subset

Source: https://github.com/Dobot-Arm/TCP-IP-CR-CPP-V4/tree/0ca0b1bbd3894b951976c5dbf1535a36c173aab7

MIT license is preserved. UPSTREAM.json records all 8 original file hashes;
LOCAL.patch is the complete patch against those files. No vendor demo is built
or run by this package. CDobotClient's inherited network is never connected.

Local upstream patches:
1. Virtualize both SendRecvMsg overloads so the official command methods can
   produce wire strings for the project's bounded ordered transport. The
   upstream synchronous 5-second wait cannot drive a fixed-period ServoJ stream.
2. Add the missing comma before InverseKin's first optional argument.
3. Preserve fixed decimal precision for FK/IK/ServoJ; exponent notation was
   rejected by this controller, and upstream six decimal places lose small angles.
4. Encode PositiveKin user/tool as explicit named arguments to retain the
   already validated project wire contract rather than changing conventions.

Unmodified official reuse: CFeedbackData (native Linux LP64), joint/Cartesian
point types and other API code. The live decoder asserts sizeof=1440 and offsets
24/32/48/432/480; only the validated common subset is consumed. It does not infer
firmware-dependent tail safety fields or clock synchronization.

Project adapter differences (outside this vendor patch): persistent socket,
whole-operation deadlines, semicolon framing/16 KiB cap, partial/coalesced bytes,
TCP_NODELAY/temporary QUICKACK, async bounded FIFO/echo/queue-ID verification,
fault lock without reconnect and same-session Stop. These are retained because
upstream uses a single 1023-byte receive and synchronous send/wait, and the real
controller has rejected immediate replacement sessions as occupied.
Stop acknowledgement is not physical stop; measured fresh idle/stationary dwell
remains required. No ClearError, enable, motion retry or reference reset is added.
