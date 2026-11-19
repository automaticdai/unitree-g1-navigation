# Unitree G1 navigation

ROS 2 Humble / Ubuntu 22.04 prototype for **G1-EDU, 29 DoF**, using a Livox
Mid360, FAST-LIO2, and Nav2. Run estimation and command supervision onboard;
use a laptop for RViz, goal selection, and offline replay.

Implemented operating modes:

| Mode | Walking commands | Estimation |
| --- | --- | --- |
| `mapping` | Unitree handheld remote only; no SDK bridge is launched | FAST-LIO plus bounded map recording |
| `localization` | Unitree handheld remote only; no SDK bridge is launched | FAST-LIO plus initialized saved-map matching |
| `navigation` | Nav2 through collision monitoring and an explicitly armed bridge | Saved-map matching plus live obstacle observations |
| `teleop` | PC gamepad through collision monitoring and an explicitly armed bridge | FAST-LIO live map (optional save) plus live obstacle observations |

**Default navigation and teleop are dry runs.** It publishes `/g1/command_preview` and
does not import or connect to the Unitree SDK. Hardware mode also requires a
separate validation interlock, a network interface, and verified walking FSM
IDs. This repository has not been validated on a physical G1.

## Scope and assumptions

The first version supports a single, level indoor floor with a rigid sensor
mount and fixed arm configuration. Local registration matches 3D points while
estimating x/y/yaw only. Set a nearby initial pose in RViz; it is **not** global
relocalization. It requires three accepted matches before reporting healthy.
Repeated geometry can still produce an incorrect match; overlap and residual
thresholds are not a proof of correct localization.

Loop closure, stairs, uneven terrain, drop-off detection, moving sensor mounts,
full-body collision checking, and semantic person detection are not implemented.
Mapping records the FAST-LIO trajectory as estimated, including any drift.
Use short mapping routes and inspect alignment before navigation. The occupancy
export only marks observed ground free; unobserved cells remain unknown and the
planner is configured not to traverse unknown space. Sparse ground observations
can therefore leave a map unplannable; inspect it rather than filling gaps as free.

## Build on the robot or an Ubuntu 22.04 host

Install ROS 2 Humble first. From the repository root:

```bash
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install python3-vcstool python3-colcon-common-extensions \
  python3-rosdep python3-numpy python3-scipy python3-yaml python3-pytest \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-collision-monitor \
  ros-humble-message-filters ros-humble-sensor-msgs-py \
  libpcl-dev libeigen3-dev
bash scripts/prepare_dependencies.sh
```

The manifests pin source revisions. `vendor/` is ignored by Git. The preparation
script selects the driver's ROS 2 manifest without running Livox's `build.sh`,
which deletes workspace build/install directories. Run preparation in a fresh
checkout; inspect existing vendor modifications before rerunning it.

Build and install the pinned Livox SDK2, then the ROS packages:

```bash
cmake -S vendor/hardware/Livox-SDK2 -B build/livox-sdk2 -DCMAKE_BUILD_TYPE=Release
cmake --build build/livox-sdk2 -j2
sudo cmake --install build/livox-sdk2
sudo ldconfig
rosdep install --from-paths src vendor/ros --ignore-src -r -y
colcon build --base-paths src vendor/ros --symlink-install \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Initialize rosdep using its standard setup if this is a fresh ROS installation.
The Python Unitree SDK is optional for mapping, localization, and dry-run
navigation. For hardware commands, install the pinned
`vendor/hardware/unitree_sdk2_python` following its README, including CycloneDDS
requirements. Do not install a second robot walking controller.

Use a dedicated ROS DDS domain for this application, for example:

```bash
export ROS_DOMAIN_ID=42
```

Set the same ROS domain on the visualization laptop. The Unitree SDK connection
uses domain 0 and the explicitly configured robot Ethernet interface. Keep raw
LiDAR processing onboard or on wired Ethernet; Wi-Fi is for visualization.

## Configure sensor and robot geometry

Copy `src/g1_navigation/config/robot.yaml` and `fastlio.yaml` to deployment
configuration paths. The supplied robot dimensions are examples, not measured
calibration. In particular, measure:

- `base_to_imu_xyz` / `base_to_imu_xyzw`: pose of the Mid360 **internal IMU**
  in the chosen rigid robot control frame. `base_link` is a virtual rigid frame
  at nominal floor height while the robot is upright.
- `initial_imu_height`: IMU height above the floor at FAST-LIO initialization.
  Start stationary and upright. This puts the initial floor at `odom.z = 0`.
- Self-filter bounds, obstacle height bands, footprint, collision zones, and
  stopping margins. Keep arms in the calibrated configuration.
- LiDAR-to-internal-IMU extrinsics and timestamps in `fastlio.yaml`.

Only set `calibration_confirmed: true` after checking these. If the waist or
head moves the sensor relative to the control frame, this rigid-mount adapter
is insufficient: lock that articulation for this prototype or implement
joint-dependent transforms before enabling motion.

Prepare a Mid360 JSON using the pinned driver's `config/MID360_config.json`
and configure the actual sensor/host Ethernet addresses. Pass its absolute path
as `livox_config`; no sensor addresses are guessed by this repository.

## Manual mapping with the handheld controller

```bash
ros2 launch g1_navigation bringup.launch.py mode:=mapping \
  livox_config:=/absolute/path/MID360_config.json \
  robot_config:=/absolute/path/robot.yaml \
  fastlio_config:=/absolute/path/fastlio.yaml \
  session_directory:=/absolute/path/maps/room_01
```

`session_directory` must not already exist. A raw bag is recorded alongside it
at `room_01_bag`; use `record_bag:=false` to disable recording. Check disk space
before long sessions. Launch RViz separately, use fixed frame `odom`, and add
`/mapping/points`, `/perception/obstacles`, `/odom`, and TF displays.

Initialize while standing still, walk using the Unitree handheld controller,
and inspect the point cloud. This mode never creates a velocity command path.
Stop walking, then save **while the sensor stack is still running**:

```bash
ros2 service call /mapping/save std_srvs/srv/Trigger '{}'
```

A successful response gives the new bundle directory containing:

- `map.pcd`: binary XYZ point cloud for inspection/export.
- `points.npy`: the same points for fast loading by the localizer.
- `map.pgm` / `map.yaml`: conservative 2D occupancy map for Nav2.
- `metadata.json`: session dates, robot identifier, config snapshots, and
  an explicit `loop_closed: false` flag.

Existing bundles are not overwritten. After saving, stop the launch gracefully
to finalize the rosbag. Restart with a new session name to record another map.
The recorder stops accepting additions beyond its voxel cap; an overflow
invalidates saving that session rather than silently exporting a truncated map.

## Manual driving with a gamepad and obstacle avoidance

Teleop drives the G1 from a gamepad attached to a PC, through the same collision
monitor and command gate as navigation. It also builds a live FAST-LIO map that
can optionally be saved. **Drive only over wired Ethernet between the PC and the
G1**; both machines need the same `ROS_DOMAIN_ID`.

On the robot:

```bash
ros2 launch g1_navigation bringup.launch.py mode:=teleop \
  livox_config:=/absolute/path/MID360_config.json \
  robot_config:=/absolute/path/robot.yaml \
  fastlio_config:=/absolute/path/fastlio.yaml \
  [session_directory:=/absolute/path/maps/room_02]
```

On the PC (`sudo apt-get install ros-humble-joy ros-humble-rviz2`, plus this
package built so the RViz layouts are installed):

```bash
ros2 launch g1_navigation pc.launch.py            # joy_node + RViz teleop layout
ros2 launch g1_navigation pc.launch.py layout:=navigation joy_device_id:=1
```

The PC runs only `joy_node` and RViz. On the robot, `gamepad_teleop` turns
`/joy` into `/cmd_vel` and an `operator` health signal. RViz (fixed frame
`odom`) shows the live map `/mapping/map_cloud`, current obstacles, the stop
and approach polygons, and a text marker with the gate state and output.

| Control (Xbox layout) | Action |
| --- | --- |
| LB (hold) | Deadman: no motion unless held; releasing it stops immediately |
| Left stick up/down | Forward up to 0.2 m/s, reverse up to 0.1 m/s |
| Left stick left/right | Turn up to 0.3 rad/s |
| Start, with LB held and sticks neutral | Arm (one request per press; never re-arms automatically) |
| Back | Disarm |

Speed increases are ramped; decreases and stops are immediate, and a stick
reversal passes through zero. Releasing LB keeps the gate armed with zero
output. If `/joy` stops for 0.25 s (gamepad unplugged, PC or link lost), the
operator health fails and the gate disarms; re-arm with Start.

Obstacle avoidance uses Nav2's **approach** model: the padded footprint is
simulated along the command and speed is reduced to keep 1.2 s before contact,
so driving towards an obstacle slows and then stops at the padded footprint,
while turning or reversing away is not reduced. There is no direction-agnostic
slowdown zone.

**Close-contact freeze.** If anything is inside the padded footprint (5 cm
around the Nav2 footprint) the collision monitor refuses **all** gamepad
motion, including reversing and turning away. There is deliberately no escape
path around the monitor: take over with the Unitree handheld remote, walk the
robot clear, then re-arm with the gamepad.

With `session_directory` set (absolute, must not exist), save the live map as
in mapping mode with `ros2 service call /mapping/save std_srvs/srv/Trigger '{}'`.
Without it, saving is refused and the bag (unless `record_bag:=false`) is
written to a timestamped `teleop_*_bag` directory in the launch's working
directory.

Teleop is a dry run unless `enable_hardware:=true` with the same interlocks as
navigation. In hardware mode the gate additionally requires a fresh, neutral
Unitree remote; touching it takes over and disarms.

## Check localization before navigating

```bash
ros2 launch g1_navigation bringup.launch.py mode:=localization \
  livox_config:=/absolute/path/MID360_config.json \
  robot_config:=/absolute/path/robot.yaml \
  map_directory:=/absolute/path/maps/room_01
```

Use RViz fixed frame `map` and **2D Pose Estimate** near the robot's actual
position and heading while stationary. Inspect `/health/localization` and map
alignment while walking manually. FAST-LIO remains continuous in `odom`;
accepted matching updates `map -> odom`. There is no AMCL or second publisher
of that transform.

## Autonomous navigation, initially dry-run

Run the same launch with `mode:=navigation`. Keep `enable_hardware:=false`.
Provide an initial pose and inspect both health topics. For dry-run only,
provide an operator heartbeat from a separate terminal:

```bash
ros2 topic pub -r 10 /g1/operator_heartbeat std_msgs/msg/Bool '{data: true}'
ros2 service call /g1/arm std_srvs/srv/SetBool '{data: true}'
```

The first command must arrive within 10 seconds of arming. Submit an RViz Nav2
goal promptly; subsequent commands must stay fresh. If health isn't valid,
arming fails with a reason. Inspect `/g1/autonomy_status` and
`/g1/command_preview`. A dry run computes commands but cannot make the robot
follow a route. Walk manually or replay data to exercise localization.

```bash
ros2 service call /g1/stop std_srvs/srv/Trigger '{}'
```

Faults latch disarmed and request cancellation of Nav2 goals. They never re-arm
automatically. Failed cancellation blocks re-arming until navigation is
restarted after resolving the fault. There are no autonomous reverse or spin
recovery behaviors; path following may rotate to align with the route.

To enable real commands after physical validation, set
`hardware_validation_complete: true`, `network_interface`, and verified
`allowed_fsm_ids` in the deployment YAML, then launch with
`enable_hardware:=true`. The bridge does not stand the robot up, change its
controller mode, or disable torque. Hardware mode requires fresh Unitree
wireless-controller messages; any button or significant stick movement
disarms autonomy. The neutral remote must keep transmitting. Verify the topic,
message fields, and native controller priority on your exact firmware.

## Safety contract and remaining validation

```text
navigation: Nav2 controller -> /cmd_vel_nav -> velocity smoother -> /cmd_vel
teleop:     joy_node (PC) -> /joy -> gamepad_teleop -> /cmd_vel
  -> collision monitor (stop + approach polygons) -> /cmd_vel_safe
  -> command gate -> bounded SDK velocity
```

The gate checks perception, robot FSM, and command freshness using a
monotonic watchdog, plus localization in navigation. `operator` health is the
dry-run heartbeat in navigation and the gamepad in teleop; `remote` health is
the Unitree handheld remote in hardware mode (both profiles). Reverse is only
allowed in teleop (0.1 m/s); navigation clamps it to zero. The collision
monitor keeps forwarding zero commands so a stationary operator keeps the gate
armed; it publishes only on input, so input loss still expires the command. Hardware mode rejects replay
time. Fresh perception requires synchronized clouds/poses and confirmed geometry;
it also rejects implausible odometry motion and excessive tilt. Missing IMU or
LiDAR eventually stops synchronized output and expires that health input.

Velocity is capped to forward/yaw motion. Zero commands bypass acceleration
smoothing after collision monitoring. On disarm the bridge sends a bounded
zero once, then releases command ownership so it does not continuously fight
the handheld remote. Every SDK command has a short duration, including stops.
A computer/process/network failure relies on the **robot honoring command
expiry**; the software cannot enforce this after it has lost communication.

Before enabling hardware, measure and verify on the robot:

1. Rigid mounting, floor reference, time alignment, and self-filter behavior.
2. Obstacle detection at foot, torso, and head heights; glass and occlusion limits.
3. Stopping distance at configured speeds, including processing/network latency.
4. Remote takeover and the robot-specific emergency procedure.
5. Command expiry after killing the bridge or disconnecting Ethernet.
6. Stops for stale clouds, rejected localization, excessive tilt, and unavailable FSM.
7. Map completeness, unknown-space blocking, and physical exclusion of drop-offs.
8. Teleop: stopping distance towards an obstacle at full gamepad speed; what the
   LiDAR sees behind the robot when reversing; pulling the PC Ethernet and
   unplugging the gamepad (robot stops within 0.25 s plus the command lease);
   close-contact freeze recovery with the Unitree remote.

The collision polygons, speeds, and timeout values are commissioning defaults,
not a certified safety system. A flat-floor map is not a drop-off detector.
FSM allowlisting is not complete robot health monitoring; battery/thermal/fault
telemetry and an independent emergency-stop mechanism still require integration
appropriate to the installed G1 firmware and test environment.

## Replay and tests

Replay uses sensor inputs only. Do not replay recorded TF, odometry, or command
topics into a running estimator; it would create duplicate publishers.

```bash
ros2 launch g1_navigation bringup.launch.py mode:=mapping replay:=true \
  record_bag:=false robot_config:=/absolute/path/robot.yaml \
  session_directory:=/absolute/path/maps/replayed_room
ros2 bag play /absolute/path/maps/room_01_bag --clock \
  --topics /livox/lidar /livox/imu
```

Lightweight tests need Python, NumPy, SciPy, PyYAML, and pytest:

```bash
python3 -m pytest -q
```

ROS tests skip outside a ROS environment. After sourcing Humble and building,
the same command also constructs and exercises ROS nodes, cloud conversions,
map saving, and the dry-run bridge. CI runs both Python 3.10 and Humble checks.
For synthetic end-to-end checks using the real Nav2 and collision-monitor nodes:

```bash
python3 scripts/smoke_navigation.py
python3 scripts/smoke_teleop.py
```

Without a local Humble install, `bash scripts/ros_test.sh [pytest args]` runs the
ROS tier in a `ros:humble-ros-base` container; `SMOKE=1` also runs both scripts.

This creates a temporary room map, supplies synthetic FAST-LIO outputs, requests
a goal, checks that a guarded velocity is produced, and interrupts sensor data
to check the latched stop. The teleop smoke drives `/joy` towards a synthetic wall and checks the approach
slowdown, unreduced reverse, deadman release, close-contact freeze, and joystick
loss. Neither connects to hardware or models G1 dynamics.
It uses `start_sensors:=false`, which can also integrate separately launched
driver/FAST-LIO nodes; those nodes must provide the documented input topics and
keep their native TF isolated. Run tests on a separate ROS domain from a robot.

Live driver operation, full FAST-LIO dependency builds, SDK transport, physical
stopping, and route completion need testing on the deployment machine.

## Source layout and interfaces

The initial implementation keeps related modules in one `ament_python` package:

| Module | Responsibility |
| --- | --- |
| `sensor_adapter.py` | Exact-stamp cloud/pose pairing, rigid transforms, odometry, body/height filtering |
| `map_recorder.py`, `map_io.py` | Bounded voxel accumulation and map bundle export |
| `localizer.py`, `registration.py` | Initialized planar registration using 3D nearest neighbors |
| `command_bridge.py`, `safety.py` | Dry-run/hardware transport, command gate, watchdog, manual takeover |
| `gamepad_teleop.py`, `teleop.py` | `/joy` to deadman-gated, ramped `/cmd_vel`, operator health, arm/disarm |
| `footprint_publisher.py` | Padded footprint for the approach polygon in teleop (no costmap) |
| `launch/pc.launch.py`, `rviz/` | PC-side `joy_node` and RViz layouts |
| `launch/bringup.launch.py` | Mode isolation, dependency nodes, Nav2 configuration assembly |

TF: `map -> odom -> base_footprint -> base_link -> mid360_imu`.
Mapping omits `map -> odom` and uses `odom` as the saved map frame.
FAST-LIO's native `camera_init -> body` TF is isolated on `/fastlio/tf`.
`base_footprint` carries planar motion; `base_link` retains vertical motion and
roll/pitch for sensor geometry. `/odom` includes child-frame velocity derived
from successive poses, with nominal (not calibrated) covariance.

Upstream interfaces: [FAST-LIO ROS 2](https://github.com/Ericsii/FAST_LIO_ROS2),
[Livox driver](https://github.com/Livox-SDK/livox_ros_driver2),
[Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python),
[Nav2](https://docs.nav2.org/). Third-party dependencies retain their respective
licenses; inspect them before redistributing a combined system.
