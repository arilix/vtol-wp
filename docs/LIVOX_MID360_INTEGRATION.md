# Livox MID360s — real-hardware integration

Status: driver installed and verified against the physical sensor on this
device (including a live RViz2 point-cloud check). A new **`gate_pass`**
mission mode now exists (`start_mode:=gate_pass`) that actually flies
through a gate using live Livox correction — implemented and built
successfully, but **not yet arm/flight tested** (that step needs the user
physically present with the drone; see §10).

## 1. Device recon (this machine)

- ROS 2: Jazzy, on Ubuntu 24.04 (arm64).
- Before this change, `ros2 node list` only showed `px4_micro_xrce_dds` and
  `tfmini_i2c_node` — **no MAVROS**, no Livox driver. The flight stack talks
  to PX4 directly over uXRCE-DDS (`px4_msgs`, topics `/fmu/in/*` /
  `/fmu/out/*`), driven by `wayfix_ws/src/px4` (`MissionManager` +
  `ControlModule`). `centering_ws` (MAVROS + Gazebo `/livox/points` bridge)
  is SITL-only and was not touched by this work.
- Network: `eth0` on this host is statically `192.168.1.50/24`. A device
  answered ARP at `192.168.1.141` with a DJI/Livox-vendor MAC.
- **The physical sensor identifies itself as a Livox `Mid360s`** (dev_type
  35, serial `ARMCP150033441`), not a plain MID360 (dev_type 9) — confirmed
  from the SDK's own detection log. This matters: the vendor config file is
  keyed by device type (`"MID360"` vs `"Mid360s"` JSON object), and using
  the wrong key makes broadcast auto-discovery silently never match, even
  though the sensor is actively streaming UDP data to the host.

## 2. What was installed

1. **Livox-SDK2** (vendor C++ SDK, no rosdep key) — cloned to
   `~/Livox-SDK2`, built with CMake, `sudo make install` into
   `/usr/local/{lib,include}`.
   - Needed one build fix for this GCC/Ubuntu combo: `sdk_core/comm/define.h`
     and others use `std::uint8_t`/`uint64_t` without including `<cstdint>`.
     Fixed by adding `-include cstdint` to `CMAKE_CXX_FLAGS` in the SDK's
     top-level `CMakeLists.txt` rather than patching every vendor file.
2. **`livox_ros_driver2`** — cloned into `wayfix_ws/src/livox_ros_driver2`
   (has native ROS 2 Jazzy support). Built as a normal colcon package
   (`package_ROS2.xml` copied to `package.xml`, `launch_ROS2/` copied to
   `launch/`, matching what the vendor's own `build.sh` does). Needed
   `libapr1-dev` (`apr-1` pkg-config dependency, not preinstalled).

## 3. Network / device configuration

`wayfix_ws/src/livox_ros_driver2/config/MID360_config.json` — top-level key
is **`Mid360s`** (not `MID360`), using the SDK2 "new" config format so the
lidar is pinned by static IP instead of relying on broadcast device-type
matching:

```json
"Mid360s": {
  "lidar_net_info": { "cmd_data_port": 56100, "push_msg_port": 56200,
                       "point_data_port": 56300, "imu_data_port": 56400,
                       "log_data_port": 56500 },
  "host_net_info": [
    {
      "lidar_ip": ["192.168.1.141"],
      "host_ip": "192.168.1.50",
      "cmd_data_port": 56101, "push_msg_port": 56201,
      "point_data_port": 56301, "imu_data_port": 56401,
      "log_data_port": 56501
    }
  ]
}
```

- **Host** (this machine): `192.168.1.50` (existing static `eth0` address).
- **Lidar**: `192.168.1.141`.
- Point cloud output format is **PointCloud2** (`xfer_format=0`, set as a
  launch param on `livox_ros_driver2_node`) — matches what the ported
  centering code already expects, so there's no dependency on the driver's
  custom `CustomMsg` message type.
- Default publish topic is `livox/lidar`; it's remapped to `/livox/points`
  in `px4.launch.xml` to match the topic name used by the ROI/clustering
  code (both the SITL-era `centering_ws` code and the new `GateCenteringLock`).

### Why the first config attempt silently did nothing

The vendor template's "old" format (`host_net_info` as a single JSON object,
`lidar_configs[].ip` as a separate top-level array) is **not read by
`Livox-SDK2`'s parser for static-IP pinning** — `lidar_configs[].ip` is
parsed but never actually applied to filter discovery. With that format the
SDK falls back to broadcast auto-discovery keyed by `device_type`, and since
this sensor broadcasts as `Mid360s` (35) while the template's device key was
`"MID360"` (9), the type never matched — the lidar kept streaming UDP data to
the host (visible in `tcpdump`) but the driver never opened a receiving
socket for it (`ss -ulnp` showed nothing bound; the kernel ICMP
"port unreachable" was firing every second). Switching to the "new" array
format with an explicit `Mid360s` key + `lidar_ip` fixed it immediately.

## 4. Code changes (wayfix_ws/src/px4)

`centering_ws` itself was **not copied or moved** — it's untouched, still
MAVROS/SITL-only, still lives at `~/centering_ws`. Only its gate-detection
*math* (ROI filter + lateral histogram + two-peak detection) was ported —
rewritten from scratch as a new, ROS-free class — into `wayfix_ws/src/px4`.
Everything already in `wayfix_ws` (the ArUco/YOLO/gripper mission state
machine in `runMission()`, `WaypointHandler`, etc.) is **unmodified** —
this is a pure addition, not a rewrite of existing logic. It follows the
existing `VisionLock`/`GroundLock` pattern exactly: a pure C++ class with
no ROS dependency, fed via a `ControlModule` callback, gated by a
kill-switch parameter in `MissionManager`.

**Sensor mounting**: confirmed with the user — the MID360s is mounted
facing forward, aligned with the Pixhawk/GPS forward reference, and the
mount is physically level (no roll/pitch tilt). So the body-frame
assumption `GateCenteringLock` uses (`forward=+X`, `lateral=+Y` right,
`height=Z` up, all aligned to the drone body frame) holds as-is — **no
extrinsic mount-yaw/roll/pitch correction needed** for this sensor (unlike
the camera, which has `camera_mount_yaw_deg` in `VisionLock`/`GroundLock`
precisely because it *isn't* assumed body-aligned).

- **`src/utils/gate_centering_lock.h` / `src/core/gate_centering_lock.cpp`**
  (new) — `GateCenteringLock`. Ported from
  `centering_ws/src/control/utils/control_.cpp`'s `centeringGateLivoxSimple()`
  (ROI filter + lateral histogram binning + two-peak/one-peak gate-pole
  detection). Pure `update(points) -> Result` per point-cloud frame; no
  internal control-loop/publish logic (unlike the SITL original, which drove
  MAVROS velocity commands directly).
- **`src/utils/control_module.h` / `src/core/control_module.cpp`** (edited,
  additive) — new `LivoxSample` struct, `LivoxCallback`, and a
  `/livox/points` `PointCloud2` subscriber (`SensorDataQoS`, matching the
  driver's publisher QoS) that extracts raw x/y/z into `LivoxSample::points`.
- **`src/utils/mission_manager.h` / `src/core/mission_manager.cpp`** (edited,
  additive) — `gate_centering_enable_` parameter, **default `false`**
  (kill switch, same convention as `vision_lock_enable_`), a
  `GateCenteringLock` instance, and `onLivoxUpdate()` which — only when
  enabled — runs the detector and logs the result
  (`RCLCPP_INFO_THROTTLE`). It does **not** feed any correction into the
  mission state machine yet; that's a deliberate follow-up once this has
  been validated against a real gate.
- **`CMakeLists.txt` / `package.xml`** — new source file, `config/`
  install, `livox_ros_driver2` exec dependency.

## 5. Launch (XML) and config (YAML)

`wayfix_ws/src/px4/launch/px4.launch.xml` — new args:

| Arg | Default | Purpose |
|---|---|---|
| `start_livox_lidar` | `false` | Start `livox_ros_driver2_node` |
| `gate_centering_enable` | `false` | Kill switch for `GateCenteringLock` logging |
| `livox_user_config_path` | `.../livox_ros_driver2/config/MID360_config.json` | Vendor JSON (network config) |
| `livox_publish_freq` | `10.0` | Hz |
| `livox_frame_id` | `livox_frame` | TF frame |

`wayfix_ws/src/px4/config/gate_centering.yaml` (new — first YAML config file
in this package; the rest of `px4.launch.xml` uses inline `<arg>`/`<param>`
values) holds the `GateCenteringLock` tuning params
(`gate_centering.detection_range_min/max`, `roi_lateral`, `gate_width`,
`centering_tolerance`, `target_gate_distance`, `min_cluster_points`),
loaded via `<param from="...">` in the launch file.

No Python anywhere in this addition. `centering_ws`'s one Python file
(`mavros_pose_tf_broadcaster.py`) was MAVROS-specific and has no counterpart
here — `wayfix_ws` doesn't use MAVROS at all, so it wasn't ported.

## 6. Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/wayfix_ws
colcon build --packages-select livox_ros_driver2 px4
source install/setup.bash
```

## 7. How to run

**Sanity-check the sensor alone first** (no PX4/mission involved), useful
after any reboot or cable change:

```bash
source /opt/ros/jazzy/setup.bash
source ~/wayfix_ws/install/setup.bash
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 \
  -p output_data_type:=0 -p frame_id:=livox_frame \
  -p user_config_path:=$(ros2 pkg prefix livox_ros_driver2)/share/livox_ros_driver2/config/MID360_config.json \
  -p cmdline_input_bd_code:=livox0000000001 \
  -r livox/lidar:=/livox/points
```

In another terminal: `ros2 topic hz /livox/points` should hold ~10 Hz.
Ctrl+C to stop — this alone does **not** touch PX4/arming, safe to run
anytime the lidar is powered.

**Full stack** (PX4 connection required — `px4_micro_xrce_dds` /
`MicroXRCEAgent` must already be running, same as any normal `px4.launch.xml`
run):

```bash
source /opt/ros/jazzy/setup.bash
source ~/wayfix_ws/install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_livox_lidar:=true \
  gate_centering_enable:=true \
  <...your usual other args: start_mode, start_mission_after_hover, dst>
```

- `start_livox_lidar:=false` (default) — Livox driver node doesn't start at
  all, nothing changes vs. before this work.
- `start_livox_lidar:=true`, `gate_centering_enable:=false` (default) —
  driver runs and publishes `/livox/points`, but `MissionManager` ignores it
  (kill switch off, exactly the pre-existing behavior).
- `gate_centering_enable:=true` — `MissionManager` additionally logs
  `Gate centering (Livox): ...` lines from `GateCenteringLock` at ~10 Hz.
  Still **read-only** — no setpoint/command is derived from it yet (see
  §4 and "Not yet done" below), so this flag is safe to leave on even
  mid-mission; it won't change how the drone flies.

Tuning knobs live in `wayfix_ws/src/px4/config/gate_centering.yaml`
(`gate_centering.*` — ROI size, expected gate width, tolerance, etc.) — edit
and rebuild (`colcon build --packages-select px4`), no launch-arg needed for
those.

## 8. Verification performed

All of the following were run against the real MID360s at `192.168.1.141`
(drone **disarmed** the whole time; the mission node was killed manually
before PX4's origin-lock could complete, as a precaution — this was a
communications test, not a flight test):

1. **SDK-level**: `~/Livox-SDK2/build/samples/livox_lidar_quick_start/livox_lidar_quick_start <config.json>`
   → printed live `point cloud handle: ... data_num: 96 ...` and
   `Imu data callback ...` lines, plus `Handle detection data ... dev_type:35
   sn:ARMCP150033441`.
2. **Driver alone**:
   `ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args ...`
   → `/livox/points` and `/livox/imu` appeared in `ros2 topic list`;
   `ros2 topic hz /livox/points` held a steady **~10.0 Hz**;
   `ros2 topic echo /livox/points --field width --once` → `19968` (non-zero
   point count per frame).
3. **Full launch file**:
   `ros2 launch px4 px4.launch.xml start_tfmini_lidar:=false start_livox_lidar:=true`
   → same steady 10 Hz on `/livox/points` through the XML-launched node +
   remap, `mission_manager` started normally alongside it.
4. **Gate-centering path**:
   same launch with `gate_centering_enable:=true` → `mission_manager` log
   showed live `Gate centering (Livox): one-side lat_err=... forward=...
   width=... centered=no` lines reacting to the sensor in real time, no
   crash, `px4` process stayed alive.
5. **Visual (RViz2)**: driver launched standalone
   (`ros2 run livox_ros_driver2 livox_ros_driver2_node ...`, no PX4 involved)
   alongside `rviz2 -d
   wayfix_ws/src/livox_ros_driver2/config/display_point_cloud_ROS2.rviz`
   on the desktop session (`DISPLAY=:0`, GNOME/Wayland). Point cloud
   rendered live in RViz2, confirmed visually by the user, steady ~10 Hz on
   `/livox/lidar`. Both processes killed afterward — this was a one-off
   visual check, not something wired into any launch arg.

## 9. Next steps

Everything above is communications/data verification only — sensor talks to
the driver, driver publishes correct data, point cloud looks right in RViz2.
None of it has moved the drone. In order:

1. **Bench-test `GateCenteringLock` against a real gate mockup.**
   Point the MID360s at two poles/objects spaced ~`gate_width` apart (see
   `gate_centering.yaml`), run with `gate_centering_enable:=true`, and watch
   the `Gate centering (Livox): ...` log — confirm `lateral_error_m` sign
   and magnitude make sense as you physically move the sensor left/right,
   and that `detected_width_m`/`width_valid` land in the expected range.
   Tune `roi_lateral`, `detection_range_min/max`, `min_cluster_points` in
   `config/gate_centering.yaml` if detection is flaky. This is the
   prerequisite for step 2 — no point wiring control to a detector that
   hasn't been checked against a real target yet.
2. ~~**Wire `gate_centering_latest_` into the mission.**~~ **Done** — see
   §10. New `start_mode:=gate_pass` mode, built successfully, not yet
   arm/flight tested.
3. **Ground test with props off / tethered**, arming with
   `start_mode:=gate_pass gate_centering_enable:=true`, confirm CENTER
   stage's position-setpoint direction is correct (drone should visibly
   lean toward gate center) and ADVANCE stage's velocity command direction
   is correct, before any free flight.
4. **Flight test** at low altitude in open space first (no real gate —
   verifies ADVANCE's timeout/`pass_distance_m` fail-safes trigger LAND
   correctly with no detector engaged), then against a real gate mockup.
5. **Revisit scan settings if needed**: `MID360_config.json`'s
   `lidar_configs[].pcl_data_type` / `pattern_mode` are left at vendor
   defaults (Cartesian 32-bit, non-repeating scan). Only touch these if
   step 1 shows point density/coverage is insufficient at the gate's
   detection range.
6. **Optional**: add a static TF (`livox_frame` → `base_link`) if any other
   consumer besides `GateCenteringLock` ever needs the point cloud in body
   frame via `tf2` instead of raw sensor-frame x/y/z — not needed for
   `GateCenteringLock` itself since it reads raw fields directly and the
   sensor is confirmed body-aligned (see §4).
7. **Multi-gate sequencing**: this pass only supports one gate
   (CENTER→ADVANCE→LAND). Chaining multiple gates (like the original
   `centering_ws` 3-gate mission) is a natural follow-up once single-gate
   behavior is flight-proven — not built speculatively now.

## 10. `gate_pass` mission mode (implemented, not yet flight tested)

New `StartMode::GATE_PASS` (`start_mode:=gate_pass`), fully isolated from
the existing ArUco/YOLO/gripper mission — after takeoff+hover it bypasses
`TAKEOFF_MARKER`/`runMission()` entirely and enters `Phase::GATE_PASS`
(`runGatePassMission()`, `mission_manager.cpp`). Nothing about the existing
mission flow changed; this is purely additive and only reachable when
`start_mode:=gate_pass` is explicitly selected.

**Two stages** (`GatePassStage`):

- **CENTER**: holds position, corrects N/E toward
  `gate_centering_debounce_->lockedTarget()` (a `VisionLock` instance reused
  purely for its proven debounce/freeze/clamp behavior — fed
  `body_right = lateral_error_m`, `body_up = 0`, `camera_mount_yaw_deg = 0`
  since the sensor is body-aligned) with the same 15-tick ramp-in
  `applyVisionLock()` uses for ArUco. Advances to ADVANCE once `centered`
  has held for `gate_pass.required_centered_ticks` consecutive ticks.
  Times out to `LAND_CMD` after `gate_pass.center_timeout_s` if the gate
  never stabilizes.
- **ADVANCE**: `sendVelocitySetpoint` — constant forward velocity
  (`gate_pass.forward_velocity_m_s`) plus a live proportional lateral
  correction from `gate_centering_latest_.lateral_error_m` (not frozen —
  corrects continuously while moving, same idea as the original
  `advanceThroughLivoxCorridor`), altitude held via
  `WaypointHandler::computeVerticalVelocity`. Completes (→ `LAND_CMD`) once
  travelled distance ≥ `gate_pass.pass_distance_m`; times out to `LAND_CMD`
  after `gate_pass.advance_timeout_s`.

**Kill switch**: `runGatePassMission()` refuses to run at all if
`gate_centering_enable_` is false when entered — logs an error and goes
straight to `LAND_CMD` rather than flying with no lateral correction
available. Both `start_mode:=gate_pass` **and** `gate_centering_enable:=true`
must be set for this mode to actually move the drone.

**Config**: `gate_pass.*` block in `config/gate_centering.yaml` (forward
velocity, proportional gain, max lateral velocity, pass distance, required
centered ticks, center/advance timeouts) — loaded automatically via the
existing `<param from="...gate_centering.yaml">` in `px4.launch.xml`, no
new launch args needed (same convention as `gate_centering.*`).

**Run it**:

```bash
ros2 launch px4 px4.launch.xml \
  start_mode:=gate_pass \
  start_mission_after_hover:=true \
  gate_centering_enable:=true \
  start_livox_lidar:=true
```

`start_mission_after_hover:=true` is required (same flag the ArUco path
uses) — without it the drone just hovers indefinitely after takeoff and
never enters `GATE_PASS`, regardless of `start_mode`.

**Verified so far**: `colcon build --packages-select px4` succeeds with no
warnings from the new code; `ros2 launch px4 px4.launch.xml --show-args`
parses cleanly. **Not verified**: actual CENTER/ADVANCE behavior against a
real gate — that requires arming the drone, which wasn't done here (see
step 3/4 above — that's the user's next hands-on step).
