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

**Default navigation is a dry run.** It publishes `/g1/command_preview` and
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
  ros-humble-message-filters ros-humble-sensor-msgs-py libpcl-dev libeigen3-dev
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
Nav2 controller -> /cmd_vel_nav -> velocity smoother -> /cmd_vel
  -> collision monitor -> /cmd_vel_safe -> command gate -> bounded SDK velocity
```

The gate checks perception, localization, robot FSM, operator connectivity,
and command freshness using a monotonic watchdog. Hardware mode rejects replay
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

The collision zones, speeds, and timeout values are commissioning defaults,
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
For a synthetic end-to-end check using the real Nav2 nodes:

```bash
python3 scripts/smoke_navigation.py
```

This creates a temporary room map, supplies synthetic FAST-LIO outputs, requests
a goal, checks that a guarded velocity is produced, and interrupts sensor data
to check the latched stop. It does not connect to hardware or model G1 dynamics.
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
