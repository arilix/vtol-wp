# wayfix_ws

Workspace ROS 2 Jazzy untuk kontrol misi VTOL PX4 secara offboard, plus
deteksi marker ArUco (kamera nadir) untuk koreksi posisi ("vision lock").

## Isi workspace

| Package | Lokasi | Fungsi |
|---|---|---|
| `px4` | `src/px4` | State machine misi (`mission_manager`) — arm, takeoff, hover, kunjungi waypoint, land. Satu-satunya node yang bicara ke PX4 lewat uXRCE-DDS. |
| `fiducial_detector` | `src/fiducial_detector` | Deteksi marker ArUco (`aruco_node`) dari kamera nadir, publish `/fiducial/pose` yang dipakai `px4` untuk vision lock. Opsional — misi tetap jalan tanpa ini (murni NED). |
| `lla_sampler.py` | root | Skrip Python berdiri sendiri untuk mengambil rata-rata koordinat GPS (lat/lon/alt AMSL) sebagai acuan waypoint. Lihat `README_LLA_SAMPLER.md`. |

**Di luar workspace ini** tapi wajib untuk terbang:

| Komponen | Lokasi | Fungsi |
|---|---|---|
| `MicroXRCEAgent` | terpasang global (`/usr/local/bin`) | Jembatan uXRCE-DDS antara PX4 dan topic ROS 2 (`/fmu/...`). |
| `tfmini_i2c_ros` | `~/tfmini_ws` | Driver TFmini lidar, publish `/range` (`sensor_msgs/Range`) — sumber altitude utama misi. |

## Dokumen lain di workspace ini

- [LLA_WAYPOINT_GUIDE.md](src/px4/LLA_WAYPOINT_GUIDE.md) — cara mengisi/mengedit waypoint misi (`mission_manager.cpp`).
- [VISION_LOCK_GUIDE.md](src/px4/VISION_LOCK_GUIDE.md) — cara kerja vision lock, kalibrasi wajib `camera_mount_yaw_deg`, kill-switch.
- [GAZEBO_LIDAR_TEST.md](src/px4/GAZEBO_LIDAR_TEST.md) — memilih sumber altitude (lidar vs local position PX4), uji tanpa lidar fisik.
- [BUGFIX_ALTITUDE_STUCK.md](src/px4/BUGFIX_ALTITUDE_STUCK.md) — riwayat lengkap sesi debugging lapangan (referensi kalau ada gejala serupa muncul lagi).
- [README_LLA_SAMPLER.md](README_LLA_SAMPLER.md) — cara ambil referensi koordinat LLA.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/wayfix_ws
colcon build --packages-select px4 fiducial_detector --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`fiducial_detector` opsional untuk di-build kalau vision lock tidak dipakai
sama sekali — tapi tetap aman kalau ikut dibuild meski tidak dijalankan.

---

## Urutan Launch — Terbang Dari Awal Sampai Akhir

Jalankan tiap langkah di **terminal terpisah**, sesuai urutan. Urutan ini
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
topic posisi macet/jitter begitu offboard mulai streaming (lihat
`BUGFIX_ALTITUDE_STUCK.md` Percobaan #2).

Cek link hidup:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /fmu/out/vehicle_local_position --once --qos-reliability best_effort
```

### Terminal 2 — Lidar TFmini (sumber altitude)

```bash
source /opt/ros/jazzy/setup.bash
source ~/tfmini_ws/install/setup.bash
cd ~/tfmini_ws
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml
```

Cek data valid sebelum lanjut:

```bash
ros2 topic echo /range --qos-reliability best_effort
```

Kalau tidak pakai lidar fisik (misal uji tanpa lidar), lihat
`GAZEBO_LIDAR_TEST.md` untuk mode fallback (`use_lidar_altitude:=false`) —
tapi ini mengurangi presisi altitude, jangan dipakai untuk terbang nyata
tanpa alasan kuat.

### Terminal 3 — Kamera + ArUco (opsional, untuk vision lock)

Pilih salah satu sesuai hardware kamera (detail argumen di
`src/fiducial_detector/launch.md`):

```bash
source /opt/ros/jazzy/setup.bash
source ~/wayfix_ws/install/setup.bash

# Webcam / kamera USB
ros2 launch fiducial_detector webcam.launch.xml device_id:=0

# Intel RealSense
ros2 launch fiducial_detector realsense.launch.xml

# Kamera yang sudah publish topic ROS sendiri
ros2 launch fiducial_detector ros_topic.launch.xml camera_topic:=<topic>
```

Cek marker terdeteksi:

```bash
ros2 topic echo /fiducial/pose
```

Lewati langkah ini kalau belum mau pakai vision lock — `mission_manager`
akan tetap jalan normal (murni NED) selama `vision_lock_enable:=false` atau
kalau `/fiducial/pose` memang tidak ada datanya sama sekali. **Jangan
percaya vision lock sebelum kalibrasi `camera_mount_yaw_deg` selesai** —
lihat `VISION_LOCK_GUIDE.md`.

### Terminal 4 — Mission Manager (kontrol misi)

```bash
source /opt/ros/jazzy/setup.bash
source ~/wayfix_ws/install/setup.bash
ros2 launch px4 px4.launch.xml \
  use_lidar_altitude:=true \
  vision_lock_enable:=false \
  camera_mount_yaw_deg:=0.0
```

Set `vision_lock_enable:=true` (+ `camera_mount_yaw_deg` hasil kalibrasi)
kalau Terminal 3 sudah jalan dan sudah dikalibrasi. Parameter lengkap:

| Parameter | Default | Keterangan |
|---|---|---|
| `altitude_command_bias_m` | `0.0` | Kompensasi bias altitude command (lihat komentar di `mission_manager.h`). |
| `use_lidar_altitude` | `true` | `true` = altitude dari `/range` lidar, `false` = dari `vehicle_local_position.z` PX4. |
| `vision_lock_enable` | `true` | Kill-switch vision lock. |
| `camera_mount_yaw_deg` | `0.0` | Kalibrasi mounting kamera — **wajib** kalau vision lock dipakai. |
| `vision_max_correction_m` | `0.4` | Batas koreksi vision lock (meter). |

Sejak titik ini **node berjalan otomatis** mengikuti state machine:

```text
INIT → WAIT_ARM → TAKEOFF → HOVER → MISSION → LAND_CMD → WAIT_DISARM
```

Pantau log di terminal ini. Ringkasan tiap fase:

- **INIT** — kirim setpoint diam 5 tick, lalu kirim OFFBOARD + ARM.
- **WAIT_ARM** — jeda singkat menunggu PX4 memproses arm (lihat catatan panjang di `mission_manager.cpp` soal kenapa fase ini tidak blokir keras).
- **TAKEOFF** — naik vertikal ke altitude waypoint pertama.
- **HOVER** — stabilisasi 3 detik. Kalau vision lock engaged, log `VISION LOCK ENGAGED` muncul dan posisi hover dikoreksi ke marker.
- **MISSION** — kunjungi waypoint berurutan, log `[WPx] ... REACHED!` tiap sampai.
- **LAND_CMD** — kirim command LAND ke PX4.
- **WAIT_DISARM** — tunggu konfirmasi disarm, lalu log `MISSION COMPLETE` dan node berhenti sendiri.

Failsafe otomatis (LAND) aktif sepanjang misi kalau: posisi PX4 stale
>1.5 detik, altitude tidak masuk akal (>8m dari origin), atau data lidar
stale saat `use_lidar_altitude:=true`. Detail lengkap ada di komentar
`loop()` pada `mission_manager.cpp`.

### Selesai / darurat

- Berhenti normal: node `mission_manager` otomatis exit setelah `MISSION
  COMPLETE`. Terminal 1-3 boleh `Ctrl+C` setelahnya.
- Berhenti darurat: `Ctrl+C` di Terminal 4 menghentikan proses ROS, **tapi
  tidak mengirim LAND** — gunakan RC/QGroundControl untuk override manual
  kalau situasi darurat, jangan andalkan Ctrl+C terminal sebagai prosedur
  darurat utama.

## Sebelum edit waypoint atau parameter baru

- Waypoint misi: `defaultMissionWaypoints()` di `src/px4/src/core/mission_manager.cpp` — lihat `LLA_WAYPOINT_GUIDE.md` untuk cara pakai `RelativePath` (tidak perlu tahu kompas) atau NED manual.
- Ambil referensi koordinat LLA (kalau perlu titik acuan absolut): `README_LLA_SAMPLER.md`.
