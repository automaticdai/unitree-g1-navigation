# Gamepad teleoperation with LiDAR obstacle avoidance — design

Date: 2026-11-19
Status: ready for approval

## Intent

Drive the G1 by hand with our own gamepad while the Mid360 LiDAR keeps it from
walking into obstacles, and watch the map and obstacles live on a PC connected
to the robot.

**Why:** this is groundwork for autonomous navigation. The avoidance layer that
is tuned and trusted during manual driving must be the *same* layer that later
guards Nav2's commands, so it is validated once, on real hardware, under direct
human supervision before any autonomy is enabled.

## Goals

1. A new `teleop` launch mode: gamepad → avoidance layer → command gate → G1.
2. Avoidance behaviour ("option 1"): the robot slows as it approaches an
   obstacle along its commanded motion and stops short of it. Motions that do
   not lead into the obstacle (turning away, backing off) remain possible.
3. Slow reverse walking in teleop so the operator can back out of tight spots.
4. Driving works while the map is being built live (no saved map needed). The
   drive can optionally be saved as a map bundle usable by `localization` and
   `navigation` modes.
5. A PC-side launch that runs the gamepad driver and RViz with a saved layout
   showing: live map, live obstacle points, robot footprint, avoidance zones,
   and the arm/disarm status with reason.
6. One collision-monitor configuration shared by `teleop` and `navigation`.

## Non-goals (this version)

- Steering around obstacles, sideways walking (`vy` stays 0), or dodging
  objects that approach a stationary robot.
- Teleop on top of a saved map with localization (live mapping only).
- Using the stock Unitree handheld remote as the driving input.
- Stairs, drop-off detection, people detection, full-body collision checking —
  unchanged from the existing README scope.

## Anti-goals (failure even if it "works")

- Any path by which a gamepad command reaches `SetVelocity` without passing
  through the collision monitor and the command gate.
- Losing the PC, the link, or the gamepad and the robot keeps walking.
- Teleop and navigation ending up with different avoidance tuning or code.
- Re-arming automatically after any fault.
- Weakening the existing hardware interlocks (`hardware_validation_complete`,
  FSM allowlist, Unitree-remote takeover).

## Constraints

- ROS 2 Humble / Ubuntu 22.04 on the robot computer **and** on the PC (or a
  Humble container on the PC). Same `ROS_DOMAIN_ID` (e.g. 42) on both; the
  Unitree SDK stays on domain 0 on the robot's interface.
- PC ↔ G1 over **wired Ethernet** for control. Wi-Fi is not supported for
  driving (README already restricts Wi-Fi to visualization).
- Humble's `nav2_collision_monitor` feature set: STOP, SLOWDOWN, APPROACH
  action types; APPROACH uses a footprint polygon, `time_before_collision`, and
  `simulation_time_step`. Parameter names must be checked against the installed
  Humble package version.
- Untested on a physical G1. All speeds, zones, and timeouts are commissioning
  defaults.
- Existing pure-Python modules (`safety.py`, etc.) are unit-tested without ROS;
  new logic should follow the same split.

## Approaches considered

| | Approach | Trade-off |
|---|---|---|
| **A (chosen)** | Feed gamepad velocity into Nav2's `collision_monitor`, using the APPROACH (time-to-collision) model, then the existing `command_bridge` gate | Same layer later guards Nav2; mature code; little new safety code. Behaviour limited to what the monitor offers; harder to unit-test; refuses all motion if a point is already inside the footprint. |
| B | Custom direction-aware Python filter | Full control and easy testing, but new safety-critical code, and later either two avoidance layers or Nav2 routed through ours. |

The user chose **A** because it gives one avoidance layer to validate for both
manual and autonomous driving.

## Design

### Data flow

```text
PC                                   G1 onboard computer
──                                   ───────────────────
gamepad ─USB─ joy_node ──/joy (20 Hz)──▶ gamepad_teleop ──/cmd_vel──▶ collision_monitor
                                             │ (arm/disarm svc calls)        │ /cmd_vel_safe
RViz ◀── /mapping/map_cloud, /perception/obstacles, /tf, zones, status       ▼
                                                                     command_bridge (gate)
                                                                             │ SetVelocity (bounded lease)
Unitree remote ──rt/wirelesscontroller──▶ command_bridge (takeover=disarm)   ▼
                                                                            G1
```

`joy_node` (stock ROS `joy` package) runs on the PC because the gamepad is
plugged into the PC. Everything that decides motion runs on the robot, so link
latency or loss can only cause a stop, never an unfiltered command.

### New node: `gamepad_teleop` (runs on the robot)

Pure logic in a ROS-free module (e.g. `teleop.py`), thin ROS wrapper node.

- Subscribes `/joy`. Requires `joy_node` `autorepeat_rate` ≈ 20 Hz so a held
  gamepad produces a steady message stream; `/joy` freshness is the link check.
- **Operator health:** publishes `operator` health to the gate (replacing the
  dry-run `/g1/operator_heartbeat` role in teleop) = `/joy` fresh (≤ 0.25 s)
  **and** the gamepad is connected. Stale `/joy` → operator unhealthy → gate
  latches disarm.
- **Deadman (hold-to-drive):** while the deadman button (default LB) is held,
  stick input maps to velocity; while released, publishes zero velocity at the
  same rate. Releasing the deadman stops motion but **does not disarm**.
- **Arm/disarm on the gamepad:** Start (while deadman held and sticks neutral)
  calls `/g1/arm`; Back/Select calls `/g1/arm false`. Arming still goes through
  all gate health checks and never happens automatically.
- **Mapping:** left stick vertical → `linear.x`, left stick horizontal →
  `angular.z` (configurable axes/buttons/scales in YAML; defaults for an
  Xbox-layout controller). Stick deadzone ~0.1. Forward capped at `max_vx`,
  reverse capped at `max_vx_reverse`, yaw at `max_wz`.
- **Ramping:** acceleration-limit increases in speed; decreases to zero are
  immediate.
- Publishes `/cmd_vel` at a fixed 20 Hz from the latest valid input.

### Avoidance: shared `collision_monitor.yaml`

Replace the current direction-agnostic stop/slow polygons with:

- **`approach` polygon (main behaviour):** robot footprint plus a thin safety
  margin (a few cm; see "Close-contact freeze" below),
  `action_type: approach`, `time_before_collision` ≈ 1.0–1.5 s,
  `simulation_time_step` ≈ 0.1 s. The monitor simulates the footprint along the
  commanded twist and scales speed so the robot keeps that much time before
  contact: slows as it closes in, stops short. Motion that does not sweep into
  the obstacle (turning away, backing away) is not reduced.
- **`stop` polygon (backstop):** just the padded footprint. A point inside it
  means something is touching or the self-filter failed; all motion stops.
- No direction-agnostic slowdown zone (it would also slow escape motions).

**Close-contact freeze.** If a point is inside the approach polygon, Humble's
simulation likely reports collision at t≈0 and refuses *all* motion, including
turning away and backing off. Accepted by design: the margin is kept thin to
make this rare, and the operator recovers with the Unitree remote. The builder
verifies the actual Humble behaviour and documents it.

The same file is used by `navigation` mode, replacing today's polygons there.
The collision monitor publishes the polygons for RViz.

Stale obstacle data: Humble's monitor may treat a stale source as "no
obstacles". This is covered by the gate, which requires fresh `perception`
health from `sensor_adapter`; a stalled cloud expires that health and stops
the robot. The builder should confirm Humble's stale-source behaviour and
document it.

### Command gate changes (`safety.py`, `command_bridge.py`)

- `Limits` gains `max_vx_reverse` (default 0.0). Reverse is clamped to
  `-max_vx_reverse` instead of 0. Teleop config sets ≈ 0.1 m/s; navigation
  keeps 0.0 (forward-only, as today).
- The set of required health inputs becomes configurable per mode. Teleop
  requires `perception`, `robot`, `operator`; **not** `localization` (there is
  no localizer in live-mapping teleop). Navigation keeps all four.
- In teleop, the bridge takes `operator` health from `gamepad_teleop` rather
  than `/g1/operator_heartbeat`.
- In hardware mode, Unitree-remote takeover remains: any remote button or stick
  movement disarms, and the neutral remote must keep transmitting, exactly as
  today. The remote is the independent second stop.
- Zero commands while armed (deadman released) are normal and do not trip the
  first-command timeout.

### Live map

`teleop` runs the existing sensor pipeline and `map_recorder`:

- `map_recorder` additionally publishes its accumulated voxel map as
  `/mapping/map_cloud` (PointCloud2 in `odom`, downsampled for display, ~1 Hz,
  transient-local QoS so a late-joining RViz gets the latest).
- If `session_directory` is given, `/mapping/save` writes a normal map bundle
  usable by `localization`/`navigation`. If omitted, nothing is saved.
- Fixed frame in teleop is `odom` (no `map → odom` without a localizer).

### Launch changes

- `bringup.launch.py`: add `mode:=teleop`. Starts sensors, `sensor_adapter`,
  `map_recorder` (session dir optional), `collision_monitor` + lifecycle,
  `gamepad_teleop`, `command_bridge` (teleop health set, reverse allowed).
  Does **not** start Nav2, map server, or localizer. `enable_hardware:=true` is
  allowed in `teleop` as well as `navigation`, with all existing interlocks.
- New `pc.launch.py` (PC side): `joy_node` with `autorepeat_rate: 20`,
  `deadzone`, device selection; RViz with the teleop layout. Argument to choose
  the navigation layout instead.

### RViz layouts (`rviz/teleop.rviz`, `rviz/navigation.rviz`)

Teleop layout, fixed frame `odom`:

- `/mapping/map_cloud` — live map (grey, small points)
- `/perception/obstacles` — current obstacle points (coloured by height)
- collision-monitor polygons — approach and stop zones
- robot footprint and TF axes (`base_footprint`, `mid360_imu`)
- status text: a small marker publisher (in `command_bridge` or
  `gamepad_teleop`) renders `/g1/autonomy_status` (ARMED/DISARMED + reason) and
  the current safe vs. requested speed as a text marker above the robot, since
  RViz cannot display a `String` topic directly.

Navigation layout: same plus `map` fixed frame, static map, Nav2 paths/costmaps,
goal tools.

### Tests

- Unit (no ROS): joy → twist mapping, deadzone, scaling, ramping, deadman
  zeroing, `/joy` staleness → operator unhealthy, arm/disarm button edges;
  gate reverse limits and per-mode required health.
- Launch-build tests: `teleop` graph contents (no Nav2/localizer; collision
  monitor present; hardware still rejected without interlocks).
- ROS smoke (extend `scripts/smoke_navigation.py` or add `smoke_teleop.py`):
  synthetic obstacle ahead → forward command reduced to zero before contact;
  reverse/turn-away command still passes; `/joy` interruption → latched disarm;
  deadman release → zero output while armed.
- README: teleop section, PC setup, extra hardware validation items (stopping
  distance under teleop at max speed, reverse visibility behind the robot,
  link-pull test, gamepad unplug test, and recovery from a close-contact
  freeze via Unitree-remote takeover).

## Decided by the user

- Our own gamepad, not the stock Unitree remote, for driving.
- Avoidance: slow then stop; turning away and backing off still possible.
- Purpose: groundwork for autonomous navigation.
- Approach A: Nav2 collision monitor shared with navigation.
- Close-contact freeze: keep the approach-polygon safety margin thin (a few
  centimetres) so the monitor rarely refuses all motion. When it does (a point
  inside the margin), recovery is **Unitree-remote takeover**; no escape path
  bypasses the monitor. The README documents this procedure and the hardware
  checklist tests it.

## Calls I made (correct if wrong)

- Gamepad plugged into the PC; avoidance and gating run on the robot.
- Wired Ethernet required for driving.
- Deadman release = stop but stay armed; link loss = latched disarm.
- Arm/disarm from gamepad buttons (Start / Back), still subject to health checks.
- Reverse allowed only in teleop, capped at ~0.1 m/s; default forward
  0.2 m/s, yaw 0.3 rad/s (existing limits).
- Teleop uses live mapping only, with optional map saving.
- Xbox-layout default button mapping, configurable.
- Unitree remote and gamepad are separate health inputs: in hardware teleop
  the gate requires both `remote` (neutral remote transmitting) and `operator`
  (fresh `/joy`), so a neutral remote can never mask a stale gamepad link.
- No Nav2 goal cancellation in teleop (there is no Nav2); disarm must not set
  `cancel_failed` and block re-arming.
- `map_recorder` does not require `session_directory` in teleop.
- If the installed Humble collision monitor cannot do what this design relies
  on (APPROACH missing/broken, wrong handling of negative `vx`, stale source
  not covered by the gate), the builder **stops and reports back** rather than
  switching approach mid-build.

## Left to the builder

- Exact Humble collision-monitor parameter names/values and footprint margin.
- Voxel downsampling for the display cloud and publish rate.
- Where the status-marker publisher lives.
- Whether to extend the existing smoke script or add a new one.
- RViz colours and display details.
