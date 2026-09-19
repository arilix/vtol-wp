# wayfix_ws

Workspace ROS 2 Jazzy untuk kontrol misi VTOL PX4 secara offboard, plus
deteksi marker ArUco (kamera nadir) untuk koreksi posisi ("vision lock").

## Isi workspace

| Package | Lokasi | Fungsi |
|---|---|---|
| `px4` | `src/px4` | State machine misi (`mission_manager`) — arm, takeoff, hover, kunjungi waypoint, land. Satu-satunya node yang bicara ke PX4 lewat uXRCE-DDS. |
| `fiducial_detector` | `src/fiducial_detector` | Deteksi marker ArUco (`aruco_node`) dari kamera nadir, publish `/fiducial/pose` yang dipakai `px4` untuk vision lock. Opsional — misi tetap jalan tanpa ini (murni NED). |
| `general_box_detector_ros` | `src/yolobox/ros2_ws` (workspace colcon terpisah, `activate.bash` sendiri) | `yolo_camera_node` — deteksi box YOLO (Hailo AI HAT) dari kamera nadir yang sama, publish `/general_box/target_center` yang dipakai `px4` untuk "ground lock" — sumber centering khusus WP1. Lihat `src/px4/PROGRAM_OVERVIEW.md` bagian "Ground Lock YOLO (WP1)". |
| `lla_sampler.py` | root | Skrip Python berdiri sendiri untuk mengambil rata-rata koordinat GPS (lat/lon/alt AMSL) sebagai acuan waypoint. Lihat `README_LLA_SAMPLER.md`. |
| `livox_ros_driver2` | `src/livox_ros_driver2` | Driver Livox MID360s (real hardware), publish PointCloud2 di `/livox/points`. Dipakai `px4` untuk `GateCenteringLock` (deteksi gerbang, masih kill-switch OFF default). Lihat `docs/LIVOX_MID360_INTEGRATION.md`. |

**Di luar workspace ini** tapi wajib untuk terbang:

| Komponen | Lokasi | Fungsi |
|---|---|---|
| `MicroXRCEAgent` | terpasang global (`/usr/local/bin`) | Jembatan uXRCE-DDS antara PX4 dan topic ROS 2 (`/fmu/...`). |

`tfmini_i2c_ros` (driver lidar, publish `/range`) **bukan lagi eksternal** —
sudah jadi package di `src/tfmini_i2c_ros` workspace ini, dan `px4.launch.xml`
otomatis menjalankannya (`start_tfmini_lidar:=true` default) sebagai bagian
dari Terminal 3 di bawah. Tidak perlu terminal terpisah untuk lidar.

## Dokumen lain di workspace ini

- [PROGRAM_OVERVIEW.md](src/px4/PROGRAM_OVERVIEW.md) — rangkuman lengkap alur program, state machine, topic ROS2, vision lock ArUco, ground lock YOLO (WP1), dan semua parameter launch.
- [LAUNCH_TUTORIAL.md](src/px4/LAUNCH_TUTORIAL.md) — contoh command launch per skenario (ArUco saja, tanpa ArUco, handoff, gripper, dll).
- [LIVOX_MID360_INTEGRATION.md](docs/LIVOX_MID360_INTEGRATION.md) — instalasi driver Livox MID360s real hardware, setup IP jaringan, `GateCenteringLock` (deteksi gerbang, masih kill-switch OFF default), cara jalanin, dan hasil verifikasi (termasuk cek visual RViz2).

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/wayfix_ws
colcon build --packages-select px4 px4_msgs fiducial_detector tfmini_i2c_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`fiducial_detector` opsional untuk di-build kalau vision lock (ArUco) tidak
dipakai sama sekali — tapi tetap aman kalau ikut dibuild meski tidak
dijalankan.

`general_box_detector_ros` (deteksi box YOLO untuk ground lock WP1) ada di
**workspace colcon terpisah** (`src/yolobox/ros2_ws`), tidak ikut ter-build
oleh perintah di atas. Build sendiri kalau mau pakai YOLO:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/wayfix_ws/src/yolobox/ros2_ws
colcon build --paths src/general_box_detector_ros \
  --build-base build_merged --install-base install_merged --merge-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Opsional kalau tidak pakai WP1/YOLO sama sekali — misi tetap jalan penuh
dengan ArUco saja (`ground_lock_enable:=false` di Terminal 3).

---

## Urutan Launch — Terbang Dari Awal Sampai Akhir

Tiga terminal terpisah, sesuai urutan. Lidar TFmini **tidak perlu terminal
sendiri** — sudah otomatis ikut Terminal 3 (`px4.launch.xml`). Urutan ini
penting: `mission_manager` akan menahan misi (tidak arm/takeoff) kalau
prasyaratnya (link PX4, data lidar) belum siap — lihat guard di `loop()`
(`mission_manager.cpp`).

### Terminal 1 — Jembatan PX4 <-> ROS2

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

Ganti `/dev/ttyAMA0` sesuai port serial companion computer ke flight
controller. **Baud rate 921600 di kedua sisi** (companion & parameter baud
FC di QGroundControl) — baud lebih rendah (mis. 115200) terbukti membuat
topic posisi macet/jitter begitu offboard mulai streaming.

Cek link hidup:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /fmu/out/vehicle_local_position --once --qos-reliability best_effort
```

### Terminal 2 — Kamera + ArUco + YOLO (opsional, untuk vision/ground lock)

Pilih salah satu sesuai hardware kamera. Kalau kameranya **Intel RealSense**
(cek dengan `v4l2-ctl --list-devices` — device video RealSense tidak bisa
dibuka lewat `webcam*.launch.xml`, harus lewat driver `realsense2_camera`):

```bash
source /opt/ros/jazzy/setup.bash
source ~/wayfix_ws/install/setup.bash
source ~/wayfix_ws/src/yolobox/ros2_ws/install_merged/setup.bash  # wajib kalau mau pakai YOLO WP1

# RealSense — ArUco + YOLO sekaligus (satu kamera, aman bersamaan karena
# dua-duanya cuma subscribe /camera/camera/color/image_raw, bukan buka device sendiri)
ros2 launch fiducial_detector realsense_with_yolo.launch.xml

# Webcam / kamera USB biasa — ArUco + YOLO sekaligus (subscribe /camera/image_raw)
ros2 launch fiducial_detector webcam_with_yolo.launch.xml device_id:=0

# Matikan YOLO di salah satu launch di atas kalau cuma mau ArUco:
#   ... yolo_enable:=false

# Varian lama tanpa YOLO sama sekali (kalau general_box_detector_ros tidak dibuild)
ros2 launch fiducial_detector realsense.launch.xml
ros2 launch fiducial_detector webcam.launch.xml device_id:=0

# Kamera yang sudah publish topic ROS sendiri
ros2 launch fiducial_detector ros_topic.launch.xml camera_topic:=<topic>
```

Cek marker/box terdeteksi:

```bash
ros2 topic echo /fiducial/pose
ros2 topic echo /general_box/target_center
```

Lewati langkah ini kalau belum mau pakai vision/ground lock sama sekali —
`mission_manager` tetap jalan normal (murni NED) selama
`vision_lock_enable:=false` dan `ground_lock_enable:=false`. **Jangan
percaya vision lock sebelum kalibrasi `camera_mount_yaw_deg` selesai** —
lihat `PROGRAM_OVERVIEW.md` bagian "Vision Lock ArUco".

### Terminal 3 — Mission Manager (kontrol misi + lidar TFmini otomatis)

```bash
source /opt/ros/jazzy/setup.bash
source ~/wayfix_ws/install/setup.bash
ros2 launch px4 px4.launch.xml \
  use_lidar_altitude:=true \
  start_mission_after_hover:=true \
  vision_lock_enable:=true \
  camera_mount_yaw_deg:=0.0 \
  ground_lock_enable:=true \
  yolo_camera_fx_px:=640.0 \
  yolo_camera_fy_px:=640.0 \
  gripper_drop_enable:=true
```

Cek lidar hidup (otomatis dijalankan launch ini, `start_tfmini_lidar` default
`true`) sebelum drone arm — kalau `/range` belum valid, node menahan misi
dan log `Menunggu data altitude TFmini /range yang valid...`:

```bash
ros2 topic echo /range --qos-reliability best_effort
```

Set `vision_lock_enable:=false`/`ground_lock_enable:=false` kalau Terminal 2
tidak dijalankan (misi murni NED). Parameter lengkap ada di
`PROGRAM_OVERVIEW.md`, ringkasan yang paling sering dipakai:

| Parameter | Default | Keterangan |
|---|---|---|
| `altitude_command_bias_m` | `0.0` | Kompensasi bias altitude command (lihat komentar di `mission_manager.h`). |
| `use_lidar_altitude` | `true` | `true` = altitude dari `/range` lidar, `false` = dari `vehicle_local_position.z` PX4. |
| `start_tfmini_lidar` | `true` | Nyalakan node TF Mini dari launch ini — matikan (`false`) hanya kalau lidar dijalankan manual dari tempat lain, jangan sampai dobel. |
| `vision_lock_enable` | `true` | Kill-switch vision lock ArUco (semua WP kecuali WP1). |
| `camera_mount_yaw_deg` | `0.0` | Kalibrasi mounting kamera — **wajib** kalau vision/ground lock dipakai. |
| `vision_max_correction_m` | `0.4` | Batas koreksi vision lock (meter). |
| `ground_lock_enable` | `true` | Kill-switch ground lock YOLO, khusus WP1. |
| `yolo_camera_fx_px` / `yolo_camera_fy_px` | `640.0` | Fokal panjang kamera (piksel) untuk proyeksi ground-plane YOLO — isi sama dengan `camera_matrix` di `fiducial_detector/config/params.yaml` kalau kamera YOLO sama dengan kamera ArUco. |
| `gripper_drop_enable` | `true` | Aktifkan drop barang setelah WP1. |

Sejak titik ini **node berjalan otomatis** mengikuti state machine:

```text
INIT → WAIT_ARM → TAKEOFF → HOVER → TAKEOFF_MARKER → MISSION → LAND_CMD → WAIT_DISARM
```

Pantau log di terminal ini. Ringkasan tiap fase:

- **INIT** — kirim setpoint diam 5 tick, lalu kirim OFFBOARD + ARM.
- **WAIT_ARM** — jeda singkat menunggu PX4 memproses arm.
- **TAKEOFF** — naik vertikal ke altitude waypoint pertama.
- **HOVER** — stabilisasi 3 detik, menahan posisi terhadap marker ArUco kalau vision lock aktif.
- **TAKEOFF_MARKER** — kalau vision lock aktif, marker ArUco di titik takeoff wajib ditemukan & di-center dulu sebelum origin misi direbase.
- **MISSION** — kunjungi waypoint berurutan, log `[WPx] ... REACHED!` tiap sampai. **WP1 pakai YOLO (ground lock) untuk centering ke box sebelum gripper, waypoint lain (termasuk WP2) pakai ArUco (vision lock)** — lihat `PROGRAM_OVERVIEW.md` bagian "Ground Lock YOLO (WP1)". Log per-frame menunjukkan `[ARUCO]`/`[YOLO]` sesuai sumber yang sedang aktif.
- **LAND_CMD** — kirim command LAND ke PX4.
- **WAIT_DISARM** — tunggu konfirmasi disarm, lalu log `MISSION COMPLETE` dan node berhenti sendiri.

Selama fase di atas, `mission_manager` juga publish `/mission/vision_source_active`
(`ARUCO`/`YOLO`/`NONE`) — `aruco_node` dan `yolo_camera_node` di Terminal 2
subscribe sinyal ini dan **skip inferensi** (bukan matikan node) kalau bukan
giliran mereka, supaya CPU/NPU tidak kerja dobel sepanjang misi. Cek sinyal
ini kalau curiga salah satu sumber tidak jalan di waktu yang seharusnya:

```bash
ros2 topic echo /mission/vision_source_active
```

Failsafe otomatis (LAND) aktif sepanjang misi kalau: posisi PX4 stale
>1.5 detik, altitude tidak masuk akal (>8m dari origin), atau data lidar
stale saat `use_lidar_altitude:=true`. Detail lengkap ada di komentar
`loop()` pada `mission_manager.cpp`.

### Selesai / darurat

- Berhenti normal: node `mission_manager` otomatis exit setelah `MISSION
  COMPLETE`. Terminal 1-2 boleh `Ctrl+C` setelahnya.
- Berhenti darurat: `Ctrl+C` di Terminal 3 menghentikan proses ROS, **tapi
  tidak mengirim LAND** — gunakan RC/QGroundControl untuk override manual
  kalau situasi darurat, jangan andalkan Ctrl+C terminal sebagai prosedur
  darurat utama.

## Sebelum edit waypoint atau parameter baru

- Waypoint misi: `defaultMissionWaypoints()` di `src/px4/src/core/mission_manager.cpp` — dibangun pakai `RelativePath` (`forward()`/`turnLeft()`/`strafeRight()`/`climbTo()`, relatif arah hidung drone saat origin dikunci, tidak perlu tahu kompas). Waypoint LLA/GPS **tidak dipakai lagi** — lihat `PROGRAM_OVERVIEW.md` bagian "Waypoint Dan Frame Koordinat".
