# Livox MID360 — disconnect / point-cloud flicker troubleshooting

Status: resolved on this device (2026-07-27). Point cloud publish rate
confirmed rock-solid at 10.00 Hz (±3 ms jitter) over a sustained 15s window
after all three fixes below were applied.

Symptoms reported, in the order they were chased down:

1. Ethernet connection to the lidar dropped every ~5s; had to click
   "Connect" manually on the NetworkManager applet to get it back.
2. After fixing (1), the point cloud in RViz kept appearing/disappearing
   ("muncul hilang muncul hilang").
3. After fixing (2), the flicker was still there, and got progressively
   worse the longer the driver ran — reproducible on every restart
   regardless of network/buffer changes.

Each symptom had a **different, unrelated root cause**. All three had to be
fixed together.

## 1. `eth0` kept dropping — DHCP requested against a lidar with no DHCP server

`nmcli connection show "Livox-360s"` showed `ipv4.method: auto` (DHCP),
while `ipv4.addresses` was set to `192.168.1.50/24` but ignored (that field
only applies under `manual` method). The MID360 has no DHCP server — it's a
fixed-IP device (`192.168.1.141` per
[`config/MID360_config.json`](../src/livox_ros_driver2/config/MID360_config.json)).
NetworkManager kept requesting a DHCP lease, timing out (`ip-config ->
failed -> disconnected -> ip-config -> ...`), which is what looked like
"disconnects every few seconds."

Editing the connection live with `nmcli connection modify` didn't stick —
this host provisions `eth0` via **netplan** (cloud-init renderer for
NetworkManager, `/etc/netplan/50-cloud-init.yaml`), which regenerates the
NetworkManager profile back to DHCP on every interface state change,
clobbering live `nmcli` edits.

**Fix:** [`scripts/fix_livox_network.sh`](../scripts/fix_livox_network.sh)
writes a higher-priority netplan drop-in
(`/etc/netplan/99-livox-static.yaml`) pinning `eth0` to
`192.168.1.50/24` with `dhcp4: no`, then runs `netplan apply`. Persists
across reboots. Needs `sudo`.

```bash
sudo bash scripts/fix_livox_network.sh
```

## 2. Point cloud flickering in RViz — Decay Time too short for the actual jitter

RViz's `PointCloud2` display had `Decay Time: 0.2s`
([`config/display_point_cloud_ROS2.rviz`](../src/livox_ros_driver2/config/display_point_cloud_ROS2.rviz)).
The driver's nominal publish rate is 10 Hz (`publish_freq = 10.0` in the
launch files), i.e. ~100ms between messages — but under any jitter (CPU
load, DDS backpressure, etc.) gaps occasionally exceeded 200ms, so
previously-rendered points decayed and vanished before the next message
arrived.

**Fix:** raised `Decay Time` to `1` (1 second) in the same rviz config
file. This masks moderate jitter without making the display noticeably
stale. It's a mitigation for symptom, not a fix for the underlying jitter —
see §3 for that.

Also increased the kernel UDP receive buffer
(`net.core.rmem_max` / `net.core.rmem_default` → 32 MiB, via
`/etc/sysctl.d/99-livox-udp-buffer.conf`) after finding
`UdpRcvbufErrors`/`UdpInErrors` non-zero in `nstat -az`. This gives the
socket more headroom for the MID360's bursty UDP delivery and is a
reasonable permanent tweak even though it wasn't the dominant cause of the
remaining flicker (§3 was).

## 3. Progressive degradation every run — ROS2 graph merging with a remote mavros/PX4/Gazebo system

This was the real, dominant cause of the "keeps getting worse the longer
it runs" pattern, independent of network/RViz settings.

This host has **two active network interfaces**:
- `eth0` — `192.168.1.50/24`, the lidar, isolated point-to-point.
- `enx00e04c364d05` (USB ethernet) — `192.168.10.248/24`, a real LAN with a
  gateway.

`ROS_AUTOMATIC_DISCOVERY_RANGE` defaults to `SUBNET`. That means this
host's ROS2 graph automatically merges with **any other ROS2 system
reachable on any UP interface** — and there genuinely is one, on the
`192.168.10.0/24` LAN: a `mavros`/PX4/`ros_gz_bridge` stack (~70 topics,
high-rate mavlink + tf) running on another machine. `ros2 node list` showed
all of `/mavros/*`, `/ros_gz_bridge`, extra `/rviz`/`static_transform_publisher`
entries even with zero local processes running (`ps aux` clean, no
docker) — confirmed genuinely remote, not stale/ghost DDS state.

That remote system's DDS traffic (RELIABLE QoS, discovery + high-rate
topics) shares the same domain and competes with the lidar driver's own
DDS traffic and the local network stack, causing the publish gaps that
grew worse the longer both systems' discovery kept exchanging state.

**Fix:** run the lidar driver+rviz with discovery restricted to localhost,
via [`scripts/run_livox_isolated.sh`](../scripts/run_livox_isolated.sh):

```bash
./scripts/run_livox_isolated.sh
```

This sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` **only for the process
launched by that script** — it does not change the default for any other
terminal/session, so normal mavros/PX4 connectivity to the other machine is
unaffected elsewhere. Deliberately not made a global default in `~/.bashrc`
for that reason.

## Quick reference

| Symptom | Fix | Script/config |
|---|---|---|
| `eth0` disconnects every ~5s, manual "Connect" needed | Static IP via netplan | `scripts/fix_livox_network.sh` |
| Point cloud flickers in RViz | Decay Time 0.2s → 1s, UDP rmem → 32MiB | `config/display_point_cloud_ROS2.rviz`, `/etc/sysctl.d/99-livox-udp-buffer.conf` |
| Flicker still there, worsens over the session | Isolate ROS2 discovery from remote mavros/Gazebo host | `scripts/run_livox_isolated.sh` |

Normal launch going forward: use `scripts/run_livox_isolated.sh` instead of
calling `ros2 launch livox_ros_driver2 rviz_MID360_launch.py` directly.
