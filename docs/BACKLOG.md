# Backlog

Status as of 2026-11-19, branch `feature/teleop-obstacle-avoidance` (PR #1).
Nothing in this stack has commanded the real robot yet: every live run so far
was a dry run (`enable_hardware:=false`), with the robot walked only by the
Unitree remote.

## Done on the real robot (Stage A: perception, no SDK commands)

- Head Mid360 read from Unitree's onboard stream (`sensor_source:=utlidar`):
  relay restamps across the ~27 min clock offset; FAST-LIO runs at 10 Hz.
- FAST-LIO accuracy: matched a raw scan-to-scan ICP over a 4.6 m walk to 4 cm.
  Unitree's leg odometry (`/dog_odom`) read ~20% short.
- Mount measured (`config/robot_g1_head.yaml`): upside down, x forward to ~1 deg,
  3.2 deg tilt, IMU 1.259 m above the floor.
- Gamepad dry run with live perception: arm/disarm, forward 0.2, reverse -0.1,
  turn +-0.3 (right stick), deadman release all correct.

## Next

1. **Live test of obstacle memory** (implemented, unit/ROS tested, not yet run
   on the robot). With the stack running:
   - put a ~0.35 m box ~1.5 m ahead, walk the robot towards it with the Unitree
     remote; the box must stay in `/perception/obstacles` after it leaves the
     LiDAR's view (inside ~1 m), at the right position;
   - walk a person past and away; their points must clear within ~1 s once the
     LiDAR can see through where they were;
   - check `sensor_adapter` CPU and 10 Hz output with memory on.
2. **Confirm calibration on site**, then set `calibration_confirmed: true` in
   `robot_g1_head.yaml`: zones centred on the robot in RViz, walls straight
   and single in the map while walking and turning.
3. **Obstacle response test** with a person/box 0.6-1 m ahead in the dry run
   (approach slowdown and freeze were not exercised live yet).
4. ~~README: document `sensor_source:=utlidar`, `robot_g1_head.yaml`, the
   camera preview and obstacle memory.~~ Done. Bags recorded with
   `sensor_source:=utlidar` now include `/g1/lidar/*` (previously only the
   unused `/livox/*` raw topics, so those sessions could not be replayed).

## Found in simulation (sim/, 2026-11-19), to confirm on the robot

- Fixed in sim: approach-stop at the freeze boundary (`direction_margin`) and
  head-height ghosts above the FOV edge (`obstacle_memory_above_sensor_age`).
  Confirm both live (items 1 and 3).
- A late obstacle cloud (> `source_timeout` 0.3 s) is treated as no obstacles by
  Humble's collision monitor; the gate only stops after 0.75 s. Measure
  `/perception/obstacles` age on the robot (`sim/tools/cloud_age.py`) under
  load; decide on raising `source_timeout` or gating on cloud age.
- ~2.5 deg attitude change vs the calibration pose lifts floor returns 2-4 m away
  over `obstacle_min_height`. Check standing and walking with
  `sim/tools/floor_fit.py /mapping/points` during item 2.

## Stage B prerequisites (SDK velocity commands; needs explicit go-ahead)

- Verified walking-controller FSM IDs for `allowed_fsm_ids`.
- Remote-takeover behaviour: Unitree remote must override and the gate must
  disarm when it does.
- `hardware_validation_complete` and `network_interface` set deliberately.
- Low limits for the first runs, remote in hand, clear space or gantry.
- Obstacle memory live-tested (item 1); known limits understood (below).

## Known limitations

- **Near-field blind zone:** the upside-down head LiDAR cannot see the floor
  within ~0.96 m; at 0.5 m only objects taller than ~0.6 m are visible.
  Obstacle memory covers objects seen before they entered the zone, not ones
  that appear inside it (e.g. something pushed next to the feet).
- Obstacle memory can leave ghost points where something stood in the blind
  zone; they block forward motion until seen through (reverse away) or 30 s.
  Memory resets after a perception gap > 0.5 s or an odometry jump.
- Mid360 scan pattern is sparse per scan: small objects appear in only some
  scans (memory addresses this).
- Base-to-IMU horizontal offset not measured (0); the waist yaw joint moves
  the head relative to the legs.
- D435 preview: colour only, ~7 Hz observed at 640x480 (exposure or transport,
  not investigated). Started manually on PC2 with `scripts/pc2_camera.sh`; not
  launched by the stack. Unitree's `/frontvideostream` and its RTP multicast
  (230.1.1.1:1720) were idle; this PC lacks GStreamer h264parse/avdec_h264.

## Hardware notes

- Right BrainCo hand: thumb_aux reports position stuck at 0.000 while drawing
  current and making noise; likely a hardware fault (slipping transmission or
  stuck sensor). Unplugged. Firmware version unknown (Modbus input register
  3000); latest V1.0.22U, versions <= V0.0.14 cannot be flashed directly.
  Diagnostic scripts left in PC2 `/tmp` (`hand_quick.py`, `hand_backoff.py`,
  `hand_trace.py`).
