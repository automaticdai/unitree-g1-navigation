# Gamepad Teleop with LiDAR Obstacle Avoidance — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `teleop` mode in which a PC-attached gamepad drives the G1 through Nav2's collision monitor (approach model) and the existing command gate, with a live map and obstacles visible in RViz on the PC.

**Architecture:** The PC runs only `joy_node` and RViz. On the robot, a new `gamepad_teleop` node turns `/joy` into `/cmd_vel` plus `operator` health; the shared `collision_monitor` filters `/cmd_vel` → `/cmd_vel_safe`; `command_bridge` gates and sends bounded `SetVelocity` leases. Pure logic lives in ROS-free modules (`safety.py`, `teleop.py`, `map_io.py`) with unit tests; ROS nodes are thin wrappers.

**Tech Stack:** Python 3.10, ROS 2 Humble (rclpy, nav2_collision_monitor, joy, rviz2), NumPy, pytest.

**Spec:** `docs/superpowers/specs/2026-11-19-teleop-obstacle-avoidance-design.md`

## Global Constraints

- ROS 2 Humble / Ubuntu 22.04 on robot and PC; same `ROS_DOMAIN_ID` (e.g. 42).
- Driving requires wired Ethernet between PC and G1.
- Limits: forward `max_vx` 0.2 m/s, yaw `max_wz` 0.3 rad/s; reverse `max_vx_reverse` 0.1 m/s in teleop only, 0.0 in navigation. `vy` always 0.
- `/joy` freshness timeout 0.25 s; `joy_node` `autorepeat_rate` 20 Hz; teleop publishes `/cmd_vel` at 20 Hz.
- Approach polygon: footprint plus a thin margin (a few cm); `time_before_collision` 1.2 s; `simulation_time_step` 0.1 s. No direction-agnostic slowdown zone.
- Never re-arm automatically. No escape path that bypasses the collision monitor; close-contact freeze is recovered with the Unitree remote.
- Existing hardware interlocks (`hardware_validation_complete`, `network_interface`, `allowed_fsm_ids`, Unitree-remote takeover) unchanged.
- If the installed Humble collision monitor cannot do what the spec relies on (APPROACH missing/broken, reverse `vx` reduced by an obstacle ahead, stale source not covered by the gate), **stop and report back**; do not switch approach.

## Review Focus

1. Gamepad of the wrong layout (fewer axes/buttons than configured) → no exception, operator unhealthy, zero command. Test in Task 2.
2. Non-finite axis values from a faulty driver → zero command, operator unhealthy. Test in Task 2.
3. Start pressed without deadman, or with sticks deflected, or Start held down → no arm request (and only one per press). Test in Task 2.
4. Stick flipped from full forward to full reverse in one message → output passes through zero, then ramps; never jumps to reverse. Test in Task 2.
5. Close-contact freeze and reverse-with-obstacle-ahead behaviour of Humble's APPROACH model → asserted in the teleop smoke script (Task 7); a reverse command reduced by an obstacle ahead is a stop-and-report condition.

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `src/g1_navigation/g1_navigation/safety.py` | modify | `max_vx_reverse`; `required_health(profile, hardware)` |
| `src/g1_navigation/g1_navigation/teleop.py` | create | ROS-free joy → velocity mapping, deadman, ramp, arm/disarm edges, staleness |
| `src/g1_navigation/g1_navigation/gamepad_teleop.py` | create | ROS node wrapping `teleop.py` |
| `src/g1_navigation/g1_navigation/command_bridge.py` | modify | `profile` param, `remote` health, teleop operator health, no cancel in teleop, status marker |
| `src/g1_navigation/g1_navigation/map_io.py` | modify | `display_points()` |
| `src/g1_navigation/g1_navigation/map_recorder.py` | modify | optional session dir, `/mapping/map_cloud` |
| `src/g1_navigation/config/collision_monitor.yaml` | modify | approach + stop polygons |
| `src/g1_navigation/config/teleop.yaml` | create | `gamepad_teleop` and teleop `command_bridge` params |
| `src/g1_navigation/launch/bringup.launch.py` | modify | `mode:=teleop` |
| `src/g1_navigation/launch/pc.launch.py` | create | `joy_node` + RViz on the PC |
| `src/g1_navigation/rviz/teleop.rviz`, `navigation.rviz` | create | RViz layouts |
| `src/g1_navigation/setup.py`, `package.xml` | modify | entry point, rviz data files, deps |
| `scripts/smoke_teleop.py` | create | synthetic end-to-end teleop check |
| `.github/workflows/test.yml` | modify | run teleop smoke |
| `README.md` | modify | teleop, PC setup, controls, freeze recovery, checklist |
| `tests/test_safety.py`, `tests/test_teleop.py`, `tests/test_map_io.py`, `tests/test_configuration.py`, `tests/test_ros_smoke.py` | modify/create | tests |

---

### Task 0: Test environments

No code. Makes the two test tiers runnable on this WSL machine (it has neither NumPy nor ROS).

- [ ] **Step 1:** `python3 -m venv .venv && .venv/bin/pip install 'numpy<2' scipy pyyaml pytest` (`.venv/` is already gitignored).
- [ ] **Step 2:** Run `.venv/bin/python -m pytest -q`. Expected: all non-ROS tests pass, ROS tests skipped. Record the count.
- [ ] **Step 3:** Create the ROS tier helper at `scripts/ros_test.sh` (committed): runs `docker run --rm -v "$PWD":/ws -w /ws ros:humble-ros-base bash -c '<install deps exactly as in .github/workflows/test.yml, plus ros-humble-joy>; source /opt/ros/humble/setup.bash; colcon build --base-paths src --symlink-install; source install/setup.bash; python3 -m pytest -q "$@"'`. It must pass through extra args and allow `SMOKE=1` to also run `scripts/smoke_navigation.py` and `scripts/smoke_teleop.py` (the latter once it exists). Build artifacts go to `build/ install/ log/` (already ignored).
- [ ] **Step 4:** `bash scripts/ros_test.sh`. Expected: all tests pass, including ROS ones. If the baseline fails, stop and report.
- [ ] **Step 5:** Commit `scripts/ros_test.sh` ("Add local Humble container test runner").

Every later "ROS tier" step means `bash scripts/ros_test.sh <args>`; "fast tier" means `.venv/bin/python -m pytest -q <args>`.

---

### Task 1: Gate — reverse limit and per-profile required health

**Files:** Modify `safety.py`; Test `tests/test_safety.py`.

**Interfaces — Produces:**
- `Limits.max_vx_reverse: float = 0.0`
- `CommandGate.receive` clamps `vx` to `[-max_vx_reverse, max_vx]`
- `required_health(profile: str, hardware: bool) -> tuple[str, ...]`:
  - `('navigation', False)` → `('perception', 'localization', 'robot', 'operator')`
  - `('navigation', True)` → `('perception', 'localization', 'robot', 'remote')`
  - `('teleop', False)` → `('perception', 'robot', 'operator')`
  - `('teleop', True)` → `('perception', 'robot', 'operator', 'remote')`
  - anything else → `ValueError`

- [ ] **Step 1: Failing tests**

```python
def test_reverse_is_clamped_to_reverse_limit():
    gate = CommandGate(Limits(max_vx_reverse=0.1))
    healthy(gate, 0.); gate.arm(0.)
    gate.receive((-1., 0., 0.), 0.)
    assert gate.output(0.) == (-0.1, 0., 0.)

def test_reverse_disabled_by_default():
    gate = CommandGate(); healthy(gate, 0.); gate.arm(0.)
    gate.receive((-0.05, 0., 0.), 0.)
    assert gate.output(0.) == (0., 0., 0.)

@pytest.mark.parametrize('profile,hardware,expected', [
    ('navigation', False, ('perception', 'localization', 'robot', 'operator')),
    ('navigation', True, ('perception', 'localization', 'robot', 'remote')),
    ('teleop', False, ('perception', 'robot', 'operator')),
    ('teleop', True, ('perception', 'robot', 'operator', 'remote'))])
def test_required_health_per_profile(profile, hardware, expected):
    assert required_health(profile, hardware) == expected

def test_unknown_profile_rejected():
    with pytest.raises(ValueError):
        required_health('mapping', False)

def test_zero_commands_keep_gate_armed():
    gate = CommandGate(required=required_health('teleop', False))
    healthy(gate, 0.); gate.arm(0.)
    for t in (0.1, 0.2, 0.3):
        healthy(gate, t); gate.receive((0., 0., 0.), t)
        assert gate.output(t) == (0., 0., 0.)
    assert gate.armed
```

Existing `test_prearm_command_is_never_replayed_and_velocity_is_bounded` must keep passing unchanged (default reverse 0).

- [ ] **Step 2:** Fast tier `tests/test_safety.py` → new tests FAIL (`max_vx_reverse` unknown / `required_health` missing).
- [ ] **Step 3:** Implement in `safety.py`.
- [ ] **Step 4:** Fast tier `tests/test_safety.py` → PASS.
- [ ] **Step 5:** Commit ("Allow bounded reverse and per-profile gate health").

---

### Task 2: Pure teleop logic (`teleop.py`)

**Files:** Create `teleop.py`, `tests/test_teleop.py`.

**Interfaces — Produces:**

```python
@dataclass(frozen=True)
class GamepadConfig:
    axis_linear: int = 1        # left stick vertical (Xbox layout via joy)
    axis_angular: int = 0       # left stick horizontal
    deadman_button: int = 4     # LB
    arm_button: int = 7         # Start
    disarm_button: int = 6      # Back
    deadzone: float = 0.1
    max_vx: float = 0.2
    max_vx_reverse: float = 0.1
    max_wz: float = 0.3
    accel_vx: float = 0.4       # m/s^2, increases only
    accel_wz: float = 0.8       # rad/s^2, increases only
    joy_timeout: float = 0.25
    # __post_init__: all finite, nonnegative, timeouts/accels > 0, indices >= 0, deadzone < 1 → else ValueError

class Gamepad:
    def __init__(self, config: GamepadConfig = GamepadConfig()): ...
    def update(self, axes: Sequence[float], buttons: Sequence[int], now: float) -> list[str]:
        """Record a /joy sample; return events, each 'arm' or 'disarm'."""
    def healthy(self, now: float) -> bool: ...
    def command(self, now: float) -> tuple[float, float]:
        """(vx, wz) after deadzone, scaling, deadman, ramp. Call once per 20 Hz tick."""
```

Rules the tests pin:
- Sample invalid (too few axes/buttons for configured indices, or any used axis non-finite) → sample ignored for commands, `healthy()` False until a valid sample arrives, no events.
- `healthy(now)` = a valid sample received and `0 <= now - last_valid <= joy_timeout`.
- Unhealthy or deadman released → target `(0, 0)`, applied immediately (no ramp).
- Stick value `s` with `|s| < deadzone` → 0; otherwise rescale `(|s| - deadzone)/(1 - deadzone)` keeping sign. Positive linear → `* max_vx`; negative → `* max_vx_reverse`; angular → `* max_wz`.
- Ramp per component, with `dt = now - previous command() time` (first call: dt = 0): if target and current have opposite signs, current = 0 first; if `|target| <= |current|` current = target; else current moves toward target by at most `accel*dt`.
- Events on rising edges only: `'arm'` when the arm button goes 0→1 **and** deadman held **and** both used sticks inside deadzone; `'disarm'` when the disarm button goes 0→1 (always).

- [ ] **Step 1: Failing tests** (`tests/test_teleop.py`, helper `joy(lin=0., ang=0., deadman=0, arm=0, disarm=0)` builds 8 axes / 11 buttons):

```python
def test_no_sample_is_unhealthy_and_zero():
    g = Gamepad(); assert not g.healthy(0.); assert g.command(0.) == (0., 0.)

def test_deadman_gates_motion_and_release_stops_immediately():
    g = Gamepad()
    g.update(*joy(lin=1.), 0.); assert g.command(0.) == (0., 0.)
    g.update(*joy(lin=1., deadman=1), 0.05); g.command(0.05)
    for t in (0.1, 0.5, 1.0): g.update(*joy(lin=1., deadman=1), t); out = g.command(t)
    assert out == pytest.approx((0.2, 0.))
    g.update(*joy(lin=1.), 1.05); assert g.command(1.05) == (0., 0.)

def test_ramp_limits_increase_only():
    g = Gamepad(); g.update(*joy(lin=1., deadman=1), 0.); g.command(0.)
    g.update(*joy(lin=1., deadman=1), 0.1)
    assert g.command(0.1)[0] == pytest.approx(0.04)  # accel 0.4 * 0.1

def test_reversal_passes_through_zero():
    g = Gamepad()
    for t in (0., 0.5, 1.0): g.update(*joy(lin=1., deadman=1), t); g.command(t)
    g.update(*joy(lin=-1., deadman=1), 1.05)
    assert g.command(1.05)[0] == pytest.approx(-0.02)  # zeroed then 0.4*0.05
    assert all(g.command(t)[0] >= -0.1 for t in (1.1, 2., 3.))

def test_deadzone_and_reverse_scaling():
    g = Gamepad()
    for t in (0., 1., 2.): g.update(*joy(lin=0.05, ang=0.05, deadman=1), t)
    assert g.command(2.) == (0., 0.)
    g2 = Gamepad()
    for t in (0., 1., 2.): g2.update(*joy(lin=-1., ang=-1., deadman=1), t); out = g2.command(t)
    assert out == pytest.approx((-0.1, -0.3))

def test_stale_joy_is_unhealthy_and_zero():
    g = Gamepad()
    for t in (0., 0.5, 1.): g.update(*joy(lin=1., deadman=1), t); g.command(t)
    assert g.healthy(1.25) and not g.healthy(1.26)
    assert g.command(1.3) == (0., 0.)

def test_arm_requires_deadman_and_neutral_sticks_on_rising_edge():
    g = Gamepad()
    assert g.update(*joy(arm=1), 0.) == []                     # no deadman
    assert g.update(*joy(), 0.05) == []
    assert g.update(*joy(lin=0.5, deadman=1, arm=1), 0.1) == []  # sticks deflected
    assert g.update(*joy(deadman=1), 0.15) == []
    assert g.update(*joy(deadman=1, arm=1), 0.2) == ['arm']
    assert g.update(*joy(deadman=1, arm=1), 0.25) == []        # held, no repeat

def test_disarm_always_on_rising_edge():
    g = Gamepad()
    assert g.update(*joy(lin=1., disarm=1), 0.) == ['disarm']
    assert g.update(*joy(lin=1., disarm=1), 0.05) == []

def test_short_or_nonfinite_sample_is_unhealthy_without_exception():
    g = Gamepad()
    g.update(*joy(deadman=1), 0.)
    assert g.update([0.], [0], 0.1) == []; assert not g.healthy(0.3)
    g2 = Gamepad(); g2.update(*joy(lin=math.nan, deadman=1), 0.)
    assert not g2.healthy(0.) and g2.command(0.) == (0., 0.)

@pytest.mark.parametrize('kwargs', [{'deadzone': 1.}, {'max_vx': -1.}, {'joy_timeout': 0.}, {'accel_vx': math.inf}])
def test_bad_config_rejected(kwargs):
    with pytest.raises(ValueError): GamepadConfig(**kwargs)
```

Note `test_short_or_nonfinite…`: a short sample does not refresh `last_valid`, so health expires 0.25 s after the last valid one.

- [ ] **Step 2:** Fast tier `tests/test_teleop.py` → FAIL (module missing).
- [ ] **Step 3:** Implement `teleop.py`.
- [ ] **Step 4:** Fast tier → PASS.
- [ ] **Step 5:** Commit ("Add ROS-free gamepad teleop logic").

---

### Task 3: `gamepad_teleop` ROS node

**Files:** Create `gamepad_teleop.py`; modify `setup.py` (entry point `gamepad_teleop = g1_navigation.gamepad_teleop:main`), `package.xml` (`exec_depend` `visualization_msgs` — used in Task 4 — no `joy` dep on robot; `sensor_msgs` already present); test in `tests/test_ros_smoke.py`.

**Interfaces:**
- Consumes: `GamepadConfig`, `Gamepad` (Task 2).
- Produces node `gamepad_teleop`:
  - params: every `GamepadConfig` field by the same name.
  - sub `/joy` (`sensor_msgs/Joy`, `qos_profile_sensor_data`).
  - pub `/cmd_vel` (`Twist`, only `linear.x`, `angular.z` set) at 20 Hz.
  - pub `/health/operator` (`DiagnosticArray`, status name `operator`, OK iff `healthy(now)`) at 20 Hz, via `ros_helpers.health`.
  - client `/g1/arm` (`SetBool`): on `'arm'` → `data=True`, on `'disarm'` → `data=False`; `call_async`, log the response message; never retry.
  - 20 Hz timer on a STEADY clock (as in `command_bridge`); `time.monotonic()` for `now`.

- [ ] **Step 1: Failing ROS test** `test_ros_gamepad_teleop_wires_joy_to_command` in `tests/test_ros_smoke.py`. The node exposes `last_command: tuple[float, float]` (set by `tick()`) and `tick(now: float | None = None)` (defaults to `time.monotonic()`). Test: construct node; `node.tick(0.)` → `last_command == (0., 0.)` and `not node.gamepad.healthy(0.)`; build an 8-axis/11-button `Joy` with `buttons[4]=1`, `axes[1]=1.0`; `node.joy(msg); node.tick()`; `time.sleep(0.1)`; `node.joy(msg); node.tick()`; assert `last_command[0] > 0` (the `joy` callback stamps samples with `time.monotonic()`).
- [ ] **Step 2:** ROS tier `-k gamepad_teleop` → FAIL.
- [ ] **Step 3:** Implement node (`joy(msg)` callback, `tick()` timer method, `last_command` attribute).
- [ ] **Step 4:** ROS tier `-k gamepad_teleop` → PASS.
- [ ] **Step 5:** Commit ("Add gamepad_teleop node").

---

### Task 4: Command bridge — profiles, remote health, status marker

**Files:** Modify `command_bridge.py`; tests in `tests/test_ros_smoke.py`.

**Interfaces:**
- Consumes: `required_health`, `Limits.max_vx_reverse` (Task 1).
- New params: `profile` (default `'navigation'`; `'teleop'` allowed; else `ValueError`), `max_vx_reverse` (default 0.0; bridge raises `ValueError` if `profile == 'navigation'` and it is non-zero).
- Gate constructed with `required=required_health(profile, hardware)`.
- Health wiring:
  - Unitree remote callback (`remote_input`) now updates `'remote'` (was `'operator'`); takeover behaviour unchanged.
  - `profile == 'teleop'`: subscribe `/health/operator` through the existing `diagnostic()` path with name `'operator'`; no `/g1/operator_heartbeat`; **no** cancel clients created and no cancellation on disarm.
  - `profile == 'navigation'`, dry run: `/g1/operator_heartbeat` as today. Hardware: no heartbeat (remote feeds `'remote'`).
- Status marker: pub `/g1/status_marker` (`visualization_msgs/Marker`, `TEXT_VIEW_FACING`, frame `base_footprint`, z = 2.0 m, scale.z 0.15, green when armed, red when disarmed, `lifetime` 0.5 s). Text: `"<ARMED|DISARMED>: <reason>\nvx <out> wz <out>"` with two decimals. Published in `tick()`.

- [ ] **Step 1: Failing ROS tests**
  - `test_ros_teleop_bridge_requires_operator_health_not_localization`: init with `-p profile:=teleop`; feed fresh `perception` and `operator` diagnostics (none for localization); `arm` → success; `node.cancel_clients == []`; disarm via `arm(False)` then `arm(True)` with fresh health → success (no `cancel_failed` latch).
  - `test_ros_teleop_bridge_allows_bounded_reverse`: `-p profile:=teleop -p max_vx_reverse:=0.1`; arm; `node.command(Twist(linear.x=-1.0))`; `node.gate.output(time.monotonic())[0] == -0.1`.
  - `test_ros_navigation_bridge_rejects_reverse_config`: `-p max_vx_reverse:=0.1` (navigation) → `ValueError` on construction.
  - Existing `test_ros_dry_bridge_never_imports_sdk_and_rejects_missing_health` must pass unchanged.
- [ ] **Step 2:** ROS tier `-k bridge` → new tests FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** ROS tier `-k bridge` → PASS.
- [ ] **Step 5:** Commit ("Add teleop profile, separate remote health, and status marker to bridge").

---

### Task 5: Live map cloud

**Files:** Modify `map_io.py`, `map_recorder.py`; tests `tests/test_map_io.py`, `tests/test_ros_smoke.py`.

**Interfaces — Produces:**
- `display_points(points: np.ndarray, resolution: float = 0.1, max_points: int = 200_000) -> np.ndarray` in `map_io.py`: one point per `resolution` voxel (first occurrence), then if still over `max_points`, evenly strided down to `≤ max_points`. Raises `ValueError` for non-finite/≤0 resolution or `max_points <= 0`.
- `map_recorder`:
  - `session_directory` optional: empty → node runs; `/mapping/save` responds `success=False, message='No session_directory configured; map will not be saved'`. Non-empty must still be absolute.
  - params `display_resolution` (0.1), `display_max_points` (200000).
  - pub `/mapping/map_cloud` (`PointCloud2`, frame `odom`, QoS depth 1 `TRANSIENT_LOCAL` reliable) from a 1 Hz timer, only when new points were recorded since the last publish.

- [ ] **Step 1: Failing tests**
  - `test_display_points_voxelizes_and_caps` (fast): 10 000 points in a 1 m cube at resolution 0.1 → ≤ 1000 rows; with `max_points=50` → ≤ 50 rows; invalid args raise.
  - `test_ros_map_recorder_without_session_publishes_but_refuses_save` (ROS): construct with no params; `record(...)`; call `node.publish_map()` (the timer method) without error; `save()` → `success False` and message contains `No session_directory`.
- [ ] **Step 2:** Run both tiers → FAIL.
- [ ] **Step 3:** Implement.
- [ ] **Step 4:** Both tiers → PASS (existing `test_ros_map_save_service` unchanged).
- [ ] **Step 5:** Commit ("Publish live map cloud; make map saving optional").

---

### Task 6: Shared collision-monitor config, teleop launch, PC launch, RViz

**Files:** Modify `config/collision_monitor.yaml`, `launch/bringup.launch.py`, `setup.py` (install `rviz/*.rviz` to `share/g1_navigation/rviz`); create `config/teleop.yaml`, `launch/pc.launch.py`, `rviz/teleop.rviz`, `rviz/navigation.rviz`; tests `tests/test_configuration.py`, `tests/test_ros_smoke.py`.

**Decisions:**
- `collision_monitor.yaml`: `polygons: [stop_zone, approach_zone]`.
  - `stop_zone`: `type: polygon`, points = Nav2 footprint `[0.35, 0.4, 0.35, -0.4, -0.35, -0.4, -0.35, 0.4]`, `action_type: stop`, `max_points: 0`, `visualize: true`, `polygon_pub_topic: stop_zone`.
  - `approach_zone`: `type: polygon`, points = footprint + 0.05 m margin `[0.40, 0.45, 0.40, -0.45, -0.40, -0.45, -0.40, 0.45]`, `action_type: approach`, `time_before_collision: 1.2`, `simulation_time_step: 0.1`, `max_points: 0`, `visualize: true`, `polygon_pub_topic: approach_zone`.
  - Remove `slow_zone`. Keep topics (`cmd_vel` → `cmd_vel_safe`), frames, timeouts and the `cloud` source.
  - Check every key against the Humble `nav2_collision_monitor` installed in the container (`ros2 param list /collision_monitor` and `ros2 param get` after launch, or the package's installed README/params). If APPROACH is unsupported, **stop and report**.
- `config/teleop.yaml`:
  ```yaml
  gamepad_teleop:
    ros__parameters: {axis_linear: 1, axis_angular: 0, deadman_button: 4, arm_button: 7,
                      disarm_button: 6, deadzone: 0.1, max_vx: 0.2, max_vx_reverse: 0.1,
                      max_wz: 0.3, accel_vx: 0.4, accel_wz: 0.8, joy_timeout: 0.25}
  command_bridge:
    ros__parameters: {profile: teleop, max_vx_reverse: 0.1}
  ```
- `bringup.launch.py`:
  - `mode` choices add `teleop`; new arg `teleop_config` (default share `config/teleop.yaml`).
  - Hardware allowed for `navigation` or `teleop` live (not replay); otherwise existing `ValueError`.
  - `session_directory` in teleop: optional; if given must be absolute and not exist (same rule as mapping).
  - Teleop graph, in this order: sensors (if `start_sensors`), `sensor_adapter`, `map_recorder` (with `session_directory` possibly empty), `collision_monitor` + `collision_lifecycle`, `gamepad_teleop` (`common + [teleop_config]`), `command_bridge` (`common + [teleop_config, {'enable_hardware': hardware}]`). Bag recording follows `record_bag` as in mapping. No Nav2, map server, localizer.
  - Navigation passes `{'profile': 'navigation'}` to the bridge explicitly.
- `pc.launch.py`: args `layout` (`teleop`|`navigation`, default `teleop`), `joy_device_id` (default 0). Nodes: `joy` `joy_node` with `{device_id, autorepeat_rate: 20.0, deadzone: 0.05}`; `rviz2` `-d share/rviz/<layout>.rviz`.
- `teleop.rviz`: fixed frame `odom`; displays: Grid; TF (`base_footprint`, `mid360_imu`); PointCloud2 `/mapping/map_cloud` (flat grey, size 0.03, transient-local durability); PointCloud2 `/perception/obstacles` (AxisColor Z, size 0.05, best-effort); Polygon `/stop_zone` (red) and `/approach_zone` (orange); Marker `/g1/status_marker`.
- `navigation.rviz`: same with fixed frame `map`, plus Map `/map`, Path `/plan`, local costmap `/local_costmap/costmap`, tools 2D Pose Estimate and Nav2 Goal.

- [ ] **Step 1: Failing tests**
  - (fast) `test_collision_monitor_uses_approach_without_slowdown`: load yaml; `approach_zone.action_type == 'approach'`; `time_before_collision == 1.2`; margin = max |approach point| − max |stop point| per axis is in `(0, 0.1]`; no polygon has `action_type: slowdown`.
  - (fast) `test_teleop_config_limits`: teleop.yaml `max_vx_reverse == 0.1`, `joy_timeout == 0.25`, `command_bridge.profile == 'teleop'`, gamepad limits equal `robot.yaml` `max_vx`/`max_wz`.
  - (fast) `test_rviz_layouts_reference_topics`: parse both `.rviz` YAML; teleop contains `/mapping/map_cloud`, `/perception/obstacles`, `/approach_zone`, `/stop_zone`, `/g1/status_marker` and `Fixed Frame: odom`.
  - (ROS) `test_teleop_launch_graph` (pattern of `test_mapping_launch_has_no_velocity_or_navigation_nodes`): mode teleop, `start_sensors=false`, `session_directory=''` → executables `['sensor_adapter', 'map_recorder', 'collision_monitor', 'lifecycle_manager', 'gamepad_teleop', 'command_bridge']`; no include of Nav2.
  - (ROS) `test_teleop_hardware_replay_rejected`: teleop + `replay=true` + `enable_hardware=true` → `ValueError`; mapping + hardware still `ValueError`.
- [ ] **Step 2:** Both tiers → FAIL.
- [ ] **Step 3:** Implement files above.
- [ ] **Step 4:** Both tiers → PASS; existing navigation smoke `SMOKE=1 bash scripts/ros_test.sh` still passes with the new polygons.
- [ ] **Step 5:** Commit ("Add teleop launch mode, shared approach-based avoidance, PC launch and RViz layouts").

---

### Task 7: Teleop end-to-end smoke script

**Files:** Create `scripts/smoke_teleop.py`; modify `.github/workflows/test.yml` (add `timeout 90s python3 scripts/smoke_teleop.py`), `scripts/ros_test.sh` (run it under `SMOKE=1`).

Structure follows `scripts/smoke_navigation.py` (temp dir, calibrated temp `robot.yaml`, `ros2 launch … mode:=teleop start_sensors:=false enable_hardware:=false`, probe node, log dump on failure, process-group teardown). The probe publishes synthetic `/Odometry` + `/cloud_registered_body` at 10 Hz (floor plus an obstacle wall, body frame = base frame − 1 m z) and `/joy` (8 axes, 11 buttons) at 20 Hz, watches `/cmd_vel`, `/cmd_vel_safe`, `/g1/command_preview`, `/g1/autonomy_status`.

Phases (each with a timeout; failures raise with state):
1. Wall at x = 0.55 m (y ∈ [−0.6, 0.6], z ∈ [0.2, 1.5]). Wait for perception OK; hold deadman, press Start → status `ARMED`.
2. Full forward stick: after ramp, `/cmd_vel` vx ≈ 0.2 and `/cmd_vel_safe` vx strictly between 0 and `/cmd_vel` vx − 0.02 (approach slowed it).
3. Full reverse stick: `/command_preview` vx ≈ −0.1 within ±0.01 (backing away not reduced). If reduced → raise `"STOP AND REPORT: Humble APPROACH reduces reverse motion away from obstacle"`.
4. Release deadman: preview = 0 and status still `ARMED` for 1 s.
5. Move wall to x = 0.38 m (inside approach margin, outside stop zone/self-filter): forward and reverse both give preview vx = 0 (documented close-contact freeze). Print the observed behaviour either way; fail only if forward is non-zero.
6. Stop publishing `/joy`: within 1 s status starts with `DISARMED` and preview is zero.
Print `PASS: …` summarising observed approach, reverse, freeze behaviour.

- [ ] **Step 1:** Write the script.
- [ ] **Step 2:** `SMOKE=1 bash scripts/ros_test.sh` → expected PASS for both smoke scripts. Any stop-and-report condition: stop and report to the user with the output.
- [ ] **Step 3:** Commit ("Add teleop end-to-end smoke test").

---

### Task 8: README

**Files:** Modify `README.md`.

- [ ] **Step 1:** Add, matching the README's existing tone:
  - Modes table row for `teleop` (gamepad through collision monitor and gate; FAST-LIO live mapping, optional save).
  - "Manual driving with obstacle avoidance" section: robot command (`ros2 launch g1_navigation bringup.launch.py mode:=teleop robot_config:=… [session_directory:=…]`), PC command (`ros2 launch g1_navigation pc.launch.py`), PC prerequisites (`ros-humble-joy ros-humble-rviz2`, same `ROS_DOMAIN_ID`, wired Ethernet), controls table (LB hold-to-drive, left stick, Start arm with LB held and sticks neutral, Back disarm), what slow/stop looks like, saving via `ros2 service call /mapping/save std_srvs/srv/Trigger '{}'`.
  - **Close-contact freeze**: when something is inside the approach margin all gamepad motion is refused; recover with Unitree-remote takeover; then re-arm.
  - Safety-contract diagram: add the teleop chain `joy → gamepad_teleop → /cmd_vel → collision monitor → /cmd_vel_safe → gate`; replace "slow/stop zones" wording with approach + stop; note reverse only in teleop; `remote` vs `operator` health.
  - Hardware checklist additions: stopping distance under teleop at max speed; reverse visibility behind the robot; Ethernet pull; gamepad unplug (robot stops ≤ 0.25 s + lease); close-contact freeze recovery via remote.
  - Tests section: `scripts/ros_test.sh`, `SMOKE=1`, `scripts/smoke_teleop.py`.
- [ ] **Step 2:** Fast tier + ROS tier full runs → all PASS.
- [ ] **Step 3:** Commit ("Document teleop mode, PC setup, and freeze recovery").
