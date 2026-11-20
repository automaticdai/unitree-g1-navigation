# G1 simulator (MuJoCo)

A simulated G1 with a simulated head Mid360, for testing this stack without the
robot. The stack runs **unchanged**, as on the real G1 with
`sensor_source:=utlidar`: the simulator plays Unitree's onboard software and
publishes `/utlidar/cloud_livox_mid360` and `/utlidar/imu_livox_mid360`.
`utlidar_relay`, FAST-LIO, `sensor_adapter` (including obstacle memory), the
collision monitor, `gamepad_teleop` and the dry-run command gate all run as on
the robot. The simulated robot walks according to `/g1/command_preview`, the gate's output in dry
run.

| Piece | Source |
| --- | --- |
| G1 model (12-DoF legs; waist, arms and head fixed) | [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym) `g1_12dof.xml`, BSD-3-Clause, fetched by `fetch_assets.sh` |
| Walking | unitree_rl_gym's pretrained G1 velocity policy (`deploy/pre_train/g1/motion.pt`), run in NumPy from `assets/g1_motion_policy.npz` |
| Mid360 scan pattern | Livox's recorded non-repetitive Mid360 table from [MuJoCo-LiDAR](https://github.com/discoverse-dev/MuJoCo-LiDAR) (MIT) |
| Physics and ray casting | [MuJoCo](https://github.com/google-deepmind/mujoco) 3.3 (`mj_multiRay`) |
| Stack runtime | ROS 2 Humble, Nav2, pinned FAST-LIO, in Docker (`docker/Dockerfile`) |

## Run

```bash
sim/run.sh                      # room scenario: stack + simulated robot, in Docker
sim/pc.sh                       # second terminal: RViz (+ joy_node if a gamepad is attached)
```

The first run fetches the G1 model into `vendor/sim/` and builds the
`g1-nav-sim` image (FAST-LIO, Livox SDK and driver, Nav2, MuJoCo), which takes
several minutes. Drive with the gamepad exactly as in the main README's teleop
section (hold LT, Start to arm). RViz uses `sim/rviz/sim.rviz`, which is the
teleop layout plus ground truth:
- green translucent shapes: the true obstacles;
- green arrow: the robot's true pose;
- red arrow: FAST-LIO's `/odom`.

Options:

```bash
sim/run.sh --scenario person    # a person-sized cylinder walks across 1.5 m ahead
sim/run.sh --scenario empty     # empty room
sim/run.sh --check              # scripted check instead of a gamepad (see below)
sim/run.sh --check --scenario person
sim/run.sh session_directory:=/ws/maps/sim_room   # extra args go to bringup.launch.py
```

DDS stays on this PC (`ROS_LOCALHOST_ONLY=1`). The stack uses `ROS_DOMAIN_ID`
(default 42). The simulated robot publishes on `SIM_ROBOT_DOMAIN` (default 41),
not 0, so a real G1 on the network is never confused with it. Stamps on
`/utlidar/*` are offset by +1620 s (`--clock-offset`) to emulate the real
robot's clock skew, so the relay's offset estimation is exercised too.

## Scripted checks (`--check`)

`check_teleop.py` drives `/joy` the way the gamepad does and scores the stack
against simulator ground truth (`/sim/ground_truth`, `/sim/scene`):

- **`room`**: drive at full stick towards a 0.35 m box 2 m ahead. Pass criteria:
  - the robot stops short of the box without contact;
  - the box stays in `/perception/obstacles` while it is in the LiDAR's
    near-field blind zone (obstacle memory);
  - reversing away is allowed;
  - releasing the deadman holds zero output while armed;
  - losing the gamepad disarms.
- **`person`**: stand still while a cylinder crosses 1.5 m ahead at 0.8 m/s.
  - It must be detected.
  - Space it has left must clear from memory within 1.5 s (95th percentile),
    for points that a beam can see through.
  - Points that no beam reaches are reported as a warning.

These are the sim counterparts of backlog items 1 and 3. They are not a
substitute for the live tests.

### Results (2026-11-19)

| Check | Result |
| --- | --- |
| Approach at full stick towards the 0.35 m box | Slows from ~0.8 m out, stops 0.09-0.10 m short (truth), no creep |
| Box in obstacles while inside the blind zone | ~240 points held by memory |
| Reverse after the approach stop | Allowed in one run, close-contact freeze in another (see findings) |
| Deadman release, gamepad loss | Zero output while armed; loss disarms |
| Person detection | 233/236 obstacle clouds |
| Vacated space clears | 1.3 s (95th percentile) |
| Relay clock offset | Injected +1620 s, estimated -1620.017 s |

### Findings for the real robot

1. **Approach-stop ends at the freeze boundary.** The approach model stops the
   robot right at the padded footprint, and the close-contact freeze trips when
   anything is inside it. A few centimetres of gait sway or overshoot decide
   whether you can still reverse away, or must take over with the Unitree remote.
   In two otherwise identical runs, one could reverse and one froze. A margin
   would fix this: give the approach polygon a larger footprint than the freeze
   zone.
2. **Head-height ghosts above the FOV edge.** The upside-down head Mid360 sees at
   most about +7 deg above horizontal (+4 to +10 deg with the mount's roll).
   Parts of a person above that, such as the head and shoulders at 1.5-2.5 m,
   are hit only during pitch sway, and no later beam can see through them.
   Memory then holds them until `obstacle_memory_max_age` (30 s), blocking forward
   driving. This is the top-edge twin of the known near-field blind-zone ghosts.
   Options:
   - forget points outside the current FOV after a shorter age;
   - remember only points below the sensor height, since memory exists for the
     low blind zone.
3. **Posture sensitivity of the floor.** If the robot's attitude relative to the
   calibration pose differs by about 2.5 deg, floor returns 2-4 m away rise over
   `obstacle_min_height` (0.10 m) and memory accumulates them. The sim showed
   this before its mount was matched to the measured one. Walking at 0.2 m/s
   changes the policy's posture by about 0.9 deg. Check the real robot with
   `tools/floor_fit.py` (standing and walking).

## Fidelity: what this does and does not test

Tested as on the robot: the `/utlidar` message format, clock offset and IMU
units, FAST-LIO on a moving, upside-down head Mid360 (with frame distortion
while walking), the measured mount, obstacle memory, the collision monitor, and
the gamepad and gate logic.

Not the robot:
- **Gait.** Walking comes from an RL policy, not Unitree's walking controller,
  so head sway differs.
  - On its own, the policy drifts 2-3 cm/s sideways on a zero command.
  - A leaky-integral velocity loop (`engine.policy_command`) brings that to
    about 0.6 cm/s and tracks commands to within a few mm/s.
  - Stopping from 0.2 m/s overshoots by about 2.5 cm.
- **Mount matched to the measurement.** At startup `engine.calibrate_mount()`
  steps in place and sets the IMU's tilt and height relative to gravity to the
  measured values (the real mount was measured against gravity on the standing
  robot, including its posture).
- **Simplified sensor.**
  - Every surface returns.
  - No reflectivity, glass or multipath.
  - Range noise 2 cm, angle noise 0.15°.
  - The robot's own body is excluded from rays, matching the real stream
    having no body returns beyond 0.3 m.
- **Mount.** The mount is exactly the configured one, so `calibration_confirmed`
  is set true inside the container only. Mount errors are not exercised.
- **Command path.**
  - SDK transport, `LocoClient`, FSM IDs and Unitree remote takeover are not
    simulated.
  - The robot follows the dry-run preview with a 0.15 s command lease.
  - Stage B still needs the real robot.
- **No camera.**

## Layout

| Path | Purpose |
| --- | --- |
| `g1_sim/world.py` | Scene: G1 model, room, scenario obstacles, head IMU/LiDAR sites |
| `g1_sim/policy.py` | Walking policy (NumPy LSTM) and PD control |
| `g1_sim/mid360.py` | Mid360 scan, noise, frame assembly, IMU |
| `g1_sim/engine.py` | Headless simulation loop (no ROS) |
| `g1_sim/node.py` | ROS front end: `/utlidar/*`, `/g1/command_preview`, ground truth |
| `check_teleop.py` | Scripted checks |
| `tools/floor_fit.py` | Floor plane in odom from `/mapping/points`: mount check, also on the real robot |
| `tools/ghosts.py` | Where obstacle points far from any true object are (simulator) |
| `tools/export_policy.py` | Re-export the policy weights (needs PyTorch) |
| `docker/` | Image and in-container start script |

`tests/test_sim.py` covers the policy, the frame layout, walking and scanning.
It skips without MuJoCo or the fetched model.
