# Tutorial Launch PX4 Mission Manager

Dokumen ini adalah panduan run package `px4` terbaru di `wayfix_ws`. Launch utama sekarang juga menyalakan driver TF Mini I2C, jadi lidar tidak perlu diluncurkan dari `tfmini_ws` lagi.

## Build Dan Setup

Jalankan setelah ada perubahan kode atau setelah package baru masuk workspace:

```bash
cd ~/wayfix_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Untuk setiap terminal baru:

```bash
cd ~/wayfix_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Komponen Yang Perlu Jalan

Sebelum launch misi, pastikan komponen dasar ini sudah berjalan:

- PX4 SITL atau flight controller fisik.
- `MicroXRCEAgent`, sebagai bridge PX4 ke ROS 2.
- Sumber posisi lokal ke `/fmu/out/vehicle_local_position`, misalnya EKF PX4 atau `pose_republisher` jika memakai FastLIO.
- TF Mini I2C lidar di `/range`; default sudah otomatis dijalankan oleh `px4.launch.xml`.
- `fiducial_detector` hanya diperlukan kalau mode ArUco/vision lock dipakai.
- Controller gripper hanya diperlukan kalau `gripper_drop_enable:=true`; package `px4` publish command ke `/gripper_cmd`, sedangkan node/hardware gripper harus subscribe topic itu.

Contoh `MicroXRCEAgent` serial, sesuaikan device dan baud dengan setup:

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

Contoh cek topic penting:

```bash
ros2 topic list
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic echo /range --qos-reliability best_effort
```

## Pilihan Start Mode

`px4.launch.xml` punya dua mode awal:

- `start_mode:=takeoff`: mode lama/default. Program arm, masuk offboard, takeoff vertikal, hover, lalu misi.
- `start_mode:=airborne_handoff`: drone sudah terbang dan ditahan dengan RC Hold. Program ambil alih dari posisi sekarang, tahan N/E, sejajarkan altitude lidar, lalu cari ArUco awal.

Mode handoff tidak mengubah urutan waypoint. Setelah heading awal terkunci, WP1 tetap maju sesuai `RelativePath`, yaitu `forward(4.9)`.

## Mode Lengkap: Lidar + ArUco + YOLO + Gripper

Ini mode utama untuk terbang lengkap: altitude memakai TF Mini lidar, WP1
(titik box, sebelum gripper) dikunci pakai **YOLO** (`ground_lock`), WP
lainnya (termasuk WP2) tetap dikunci **ArUco** (`vision_lock`) — lihat
`PROGRAM_OVERVIEW.md` bagian "Ground Lock YOLO (WP1)". Gripper aktif untuk
drop barang setelah WP1 (box) centered.

Karena WP1 butuh kamera ArUco DAN kamera YOLO sekaligus (satu device fisik,
dua subscriber — lihat README), pakai launch gabungan `*_with_yolo`, bukan
`webcam.launch.xml`/`realsense.launch.xml` biasa. **Wajib source dua
workspace**: `wayfix_ws/install` DAN `yolobox/ros2_ws/install_merged`
(package `general_box_detector_ros` ada di workspace colcon terpisah,
lihat README bagian build YOLO).

Terminal kamera ArUco + YOLO, pilih salah satu sesuai hardware:

```bash
cd ~/wayfix_ws
source install/setup.bash
source src/yolobox/ros2_ws/install_merged/setup.bash
ros2 launch fiducial_detector webcam_with_yolo.launch.xml device_id:=0 marker_size:=0.05
```

Atau RealSense:

```bash
cd ~/wayfix_ws
source install/setup.bash
source src/yolobox/ros2_ws/install_merged/setup.bash
ros2 launch fiducial_detector realsense_with_yolo.launch.xml marker_size:=0.05 width:=640 height:=480 fps_limit:=15
```

Cek marker, box YOLO, dan lidar sebelum arm:

```bash
ros2 topic echo /fiducial/pose
ros2 topic echo /fiducial/marker_centers
ros2 topic echo /general_box/target_center
ros2 topic echo /range --qos-reliability best_effort
```

Jika gripper fisik dipakai, pastikan node controller gripper sudah berjalan dan subscribe `/gripper_cmd`. Untuk cek command yang dikirim program saat test tanpa hardware:

```bash
ros2 topic echo /gripper_cmd
```

Launch misi lengkap:

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=takeoff \
  start_mission_after_hover:=true \
  use_lidar_altitude:=true \
  start_tfmini_lidar:=true \
  vision_lock_enable:=true \
  camera_mount_yaw_deg:=0.0 \
  marker_center_tolerance_m:=0.10 \
  marker_search_timeout_s:=20.0 \
  ground_lock_enable:=true \
  yolo_camera_fx_px:=640.0 \
  yolo_camera_fy_px:=640.0 \
  gripper_drop_enable:=true \
  gripper_cmd_topic:=/gripper_cmd \
  gripper_open_wait_ticks:=15 \
  gripper_close_wait_ticks:=15
```

Jika TF Mini perlu offset, tambahkan:

```bash
tfmini_range_offset:=0.05
```

## Mode Handoff: Manual RC Hold Ke Autonomous

Mode ini dipakai saat operator menerbangkan drone manual, lalu menahan posisi dengan switch Hold di RC. Setelah drone stabil di udara, jalankan mission manager dengan `start_mode:=airborne_handoff`.

Alur handoff:

```text
RC Hold -> Offboard takeover -> altitude align lidar -> center ArUco awal
-> align heading dari marker besar->kecil -> mulai WP1 maju 4.9m
```

Untuk heading otomatis, titik start perlu punya dua marker ArUco yang terlihat kamera:

- `marker_heading_back_id`: marker belakang/besar.
- `marker_heading_front_id`: marker depan/kecil.

Mission manager memakai garis pixel dari marker belakang ke marker depan. Jika garis itu miring di gambar kamera, drone yaw pelan sampai lurus. Fase ini hanya aktif di mode `airborne_handoff` dan hanya di ArUco awal setelah switch, bukan di waypoint-waypoint berikutnya.

Cek dua marker terlihat:

```bash
ros2 topic echo /fiducial/marker_centers
```

Launch handoff:

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=airborne_handoff \
  start_mission_after_hover:=true \
  use_lidar_altitude:=true \
  start_tfmini_lidar:=true \
  vision_lock_enable:=true \
  marker_heading_align_enable:=true \
  marker_heading_back_id:=0 \
  marker_heading_front_id:=1 \
  marker_heading_tolerance_deg:=5.0 \
  marker_heading_timeout_s:=8.0 \
  gripper_drop_enable:=true
```

Ganti `marker_heading_back_id` dan `marker_heading_front_id` sesuai ID fisik marker di lapangan. Jika saat log `[ARUCO-HEADING]` error heading makin besar ketika drone yaw, balik arah koreksi:

```bash
marker_heading_yaw_sign:=-1.0
```

Jika marker pair tidak terlihat atau tidak pernah stabil, mode ini timeout setelah `marker_heading_timeout_s` dan lanjut memakai yaw terbaik terakhir.

## Mode 1: Misi Dengan ArUco (Tanpa YOLO)

Mode ini memakai marker ArUco di titik takeoff dan setiap waypoint **kecuali WP1** (WP1 defaultnya YOLO — lihat "Ground Lock YOLO (WP1)" di `PROGRAM_OVERVIEW.md`). Kalau mau uji coba murni ArUco tanpa perlu jalankan `yolo_camera_node`/box fisik sama sekali, wajib set `ground_lock_enable:=false` juga — begitu dimatikan, WP1 otomatis jatuh ke final-position-hold biasa (sama seperti `vision_lock_enable:=false` di waypoint lain), bukan menunggu deteksi box yang tidak akan pernah datang. `mission_manager` akan takeoff, hover 3 detik, mencari marker, melakukan centering, lalu mulai misi.

Terminal kamera, pilih salah satu sesuai hardware:

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch fiducial_detector webcam.launch.xml device_id:=0 marker_size:=0.05
```

Atau RealSense:

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch fiducial_detector realsense.launch.xml marker_size:=0.05 width:=640 height:=480 fps_limit:=15
```

Atau jika image topic sudah ada:

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch fiducial_detector ros_topic.launch.xml camera_topic:=/camera/image_raw marker_size:=0.05
```

Cek marker sebelum arm:

```bash
ros2 topic echo /fiducial/pose
ros2 topic echo /fiducial/marker_centers
```

Launch misi penuh dengan ArUco, tanpa YOLO:

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=takeoff \
  start_mission_after_hover:=true \
  vision_lock_enable:=true \
  ground_lock_enable:=false \
  gripper_drop_enable:=false \
  camera_mount_yaw_deg:=0.0 \
  marker_center_tolerance_m:=0.10 \
  marker_search_timeout_s:=20.0
```

Command di atas mematikan gripper DAN ground-lock YOLO supaya mode ini fokus untuk validasi ArUco saja (WP1 jatuh ke final-position-hold, bukan menunggu box). Jika ingin ArUco sekaligus YOLO+gripper, pakai command "Mode Lengkap" di atas atau ubah `ground_lock_enable:=true`/`gripper_drop_enable:=true`.

Kalibrasi `camera_mount_yaw_deg` wajib dilakukan di lapangan. Default `0.0` berarti atas gambar kamera dianggap searah hidung drone. Jika koreksi menjauh dari marker, hentikan test dan ubah nilai ini, biasanya mulai dari `90.0`, `180.0`, atau `270.0`.

## Mode 2: Misi Tanpa ArUco Dan Tanpa YOLO

Mode ini tidak membutuhkan kamera, marker, atau box. Setelah takeoff dan hover 3 detik, misi langsung mengikuti waypoint NED/RelativePath. Di setiap waypoint (termasuk WP1) tetap ada final position hold ke pusat waypoint sebelum lanjut — **wajib matikan `vision_lock_enable` DAN `ground_lock_enable` sekaligus**, karena WP1 pakai `ground_lock_enable` (bukan `vision_lock_enable`) sebagai kill-switch-nya.

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=takeoff \
  start_mission_after_hover:=true \
  vision_lock_enable:=false \
  ground_lock_enable:=false \
  gripper_drop_enable:=false
```

Jika ingin menguji di Gazebo tanpa topic `/range`, pakai altitude lokal PX4 dan jangan jalankan driver TF Mini:

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=takeoff \
  start_mission_after_hover:=true \
  vision_lock_enable:=false \
  ground_lock_enable:=false \
  use_lidar_altitude:=false \
  start_tfmini_lidar:=false \
  gripper_drop_enable:=false
```

## Mode 3: Tes Hover Saja

Mode default saat ini adalah takeoff lalu hover terus. Ini cocok untuk cek arm, offboard, altitude lidar, dan kestabilan posisi tanpa menjalankan waypoint.

Dengan lidar fisik:

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=takeoff \
  start_mission_after_hover:=false \
  use_lidar_altitude:=true
```

Tanpa lidar atau untuk Gazebo:

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=takeoff \
  start_mission_after_hover:=false \
  use_lidar_altitude:=false \
  start_tfmini_lidar:=false
```

Log yang dicari:

```text
Start mission after hover: false (takeoff + hover only)
HOVER HOLD: anchor takeoff + koreksi ArUco terbatas; misi dinonaktifkan.
```

## Lidar TF Mini

Default `px4.launch.xml` menjalankan:

```text
pkg=tfmini_i2c_ros exec=tfmini_i2c_node topic=/range
```

Parameter umum:

```bash
ros2 launch px4 px4.launch.xml \
  tfmini_bus_device:=/dev/i2c-1 \
  tfmini_i2c_address:=16 \
  tfmini_i2c_read_mode:=command \
  tfmini_range_offset:=0.05
```

Cek sensor:

```bash
i2cdetect -y 1
ros2 topic echo /range --qos-reliability best_effort
```

Jika permission `/dev/i2c-1` ditolak:

```bash
sudo usermod -aG i2c $USER
```

Lalu login ulang.

## Gripper

Program publish command string ke topic gripper. Default drop aktif setelah waypoint pertama selesai, yaitu setelah drone sampai WP1 dan fase centering box YOLO (`ground_lock`, atau final hold biasa kalau `ground_lock_enable:=false`) selesai. Program menahan posisi, publish `open`, menunggu beberapa tick, publish `close`, menunggu lagi, lalu baru lanjut ke waypoint berikutnya.

Yang diaktifkan/dinonaktifkan dari `px4.launch.xml` adalah logika drop di `mission_manager`. Node gripper fisik tetap harus ada sendiri sebagai subscriber `/gripper_cmd`, misalnya controller servo/GPIO di companion computer.

Alur gripper:

```text
WP1 selesai -> hold posisi -> publish "open" -> tunggu open_wait -> publish "close" -> tunggu close_wait -> lanjut misi
```

Topic default:

```text
/gripper_cmd
```

Command yang dikirim:

```text
open
close
```

Aktifkan gripper:

```bash
ros2 launch px4 px4.launch.xml \
  start_mission_after_hover:=true \
  gripper_drop_enable:=true \
  gripper_cmd_topic:=/gripper_cmd \
  gripper_open_wait_ticks:=15 \
  gripper_close_wait_ticks:=15
```

`gripper_open_wait_ticks` dan `gripper_close_wait_ticks` dihitung pada loop 10 Hz. Nilai `15` berarti sekitar 1.5 detik.

Untuk mematikan drop barang:

```bash
ros2 launch px4 px4.launch.xml \
  start_mission_after_hover:=true \
  gripper_drop_enable:=false
```

## Override Parameter Cepat

Lihat semua argumen launch:

```bash
ros2 launch px4 px4.launch.xml --show-args
```

Contoh lengkap misi ArUco saja (tanpa YOLO) dengan offset lidar dan gripper nonaktif:

```bash
ros2 launch px4 px4.launch.xml \
  start_mode:=takeoff \
  start_mission_after_hover:=true \
  vision_lock_enable:=true \
  ground_lock_enable:=false \
  camera_mount_yaw_deg:=90.0 \
  tfmini_range_offset:=0.05 \
  gripper_drop_enable:=false
```

Contoh lengkap misi ArUco + YOLO + lidar + gripper aktif (butuh kamera `*_with_yolo` dan sourcing `yolobox/ros2_ws/install_merged`, lihat "Mode Lengkap" di atas):

```bash
ros2 launch px4 px4.launch.xml \
  start_mode:=takeoff \
  start_mission_after_hover:=true \
  use_lidar_altitude:=true \
  start_tfmini_lidar:=true \
  vision_lock_enable:=true \
  ground_lock_enable:=true \
  yolo_camera_fx_px:=640.0 \
  yolo_camera_fy_px:=640.0 \
  camera_mount_yaw_deg:=90.0 \
  tfmini_range_offset:=0.05 \
  gripper_drop_enable:=true
```
