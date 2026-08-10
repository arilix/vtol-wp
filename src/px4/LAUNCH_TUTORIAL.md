# Tutorial Launch PX4 Mission Manager

Dokumen ini adalah panduan run package `px4` terbaru di `wayfix_ws`. Launch utama sekarang juga menyalakan driver TF Mini I2C dan (opsional) driver Livox MID360, jadi tidak ada proses lain yang perlu diluncurkan dari workspace lain untuk menerbangkan misi.

**Gate centering (Livox) tidak butuh `centering_ws`.** Matematika deteksi gerbangnya sudah diporting penuh ke `src/px4/src/utils/gate_centering_lock.h` dan `src/px4/src/core/gate_centering_lock.cpp` di workspace ini — `centering_ws` di Raspberry Pi hanya jadi referensi sumber, bukan dependency runtime. Driver Livox (`livox_ros_driver2`) juga sudah ada sebagai package di `wayfix_ws` dan dinyalakan langsung oleh `px4.launch.xml` lewat argumen `start_livox_lidar`. Jadi untuk terbang, **cukup satu launch**: `ros2 launch px4 px4.launch.xml ...` — tidak perlu buka terminal terpisah untuk `centering_ws`.

## Build Dan Setup

Jalankan setelah ada perubahan kode atau setelah package baru masuk workspace:

```bash
cd ~/wayfix_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Jika yang berubah hanya package `src/px4` dan dependency workspace sebelumnya sudah pernah berhasil dibangun, cukup:

```bash
cd ~/wayfix_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select px4 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Pada workspace baru/bersih, atau jika `px4_msgs`, `tfmini_i2c_ros`, `livox_ros_driver2`, atau dependency lain belum tersedia di `install`, gunakan `colcon build --packages-up-to px4 --symlink-install` atau build seluruh workspace satu kali.

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
- Sumber posisi lokal ke `/fmu/out/vehicle_local_position`. Untuk validasi non-Livox, gunakan estimator PX4 yang sebelumnya stabil dan pastikan FastLIO/`pose_republisher` tidak berjalan.
- TF Mini I2C lidar di `/range`; default sudah otomatis dijalankan oleh `px4.launch.xml`.
- `fiducial_detector` (+ `yolo_camera_node` untuk WP1) hanya diperlukan kalau ArUco/YOLO dipakai — lihat "Kamera ArUco + YOLO" di bawah.
- Livox MID360 hanya diperlukan kalau `gate_centering_enable:=true`; `px4.launch.xml` sudah bisa menyalakan drivernya sendiri lewat `start_livox_lidar:=true`.
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

## Kamera ArUco + YOLO

WP1 (titik box, sebelum gripper) dikunci pakai **YOLO** (`ground_lock`), WP lainnya (termasuk WP2 dan marker awal handoff) dikunci **ArUco** (`vision_lock`) — lihat `PROGRAM_OVERVIEW.md` bagian "Ground Lock YOLO (WP1)". Karena satu kamera fisik dipakai dua detector sekaligus, pakai launch gabungan `*_with_yolo`. **Wajib source dua workspace**: `wayfix_ws/install` DAN `yolobox/ros2_ws/install_merged`.

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

Kalibrasi `camera_mount_yaw_deg` wajib dilakukan di lapangan (default `0.0` = atas gambar kamera dianggap searah hidung drone). Jika koreksi menjauh dari marker, hentikan test dan ubah nilai ini, biasanya mulai dari `90.0`, `180.0`, atau `270.0`.

Untuk mode `airborne_handoff`, siapkan juga dua marker di titik start:

- `marker_heading_back_id` (default `0`): marker belakang/besar, dipakai untuk centering posisi.
- `marker_heading_front_id` (default `1`): marker depan/kecil, dipakai untuk koreksi heading.

Setelah posisi terkunci ke marker belakang, program menghitung garis pixel marker-belakang → marker-depan dan membandingkannya dengan heading pilot. Kalau selisihnya sudah kecil (≤ `marker_heading_tolerance_deg`), heading pilot **tidak diubah sama sekali**. Kalau selisihnya lebih besar, drone berputar tegas satu kali (bukan mengoreksi pelan/berulang) lalu lanjut. Log yang dicari: `[ARUCO-HEADING]`. Jika arah koreksi terbalik (error makin besar saat drone berputar), balik tandanya:

```bash
marker_heading_yaw_sign:=-1.0
```

## Gate Centering (Livox MID360)

Gate assist opt-in per-leg: kalau drone mendeteksi gerbang saat maju ke suatu waypoint, `APPROACH` dijeda otomatis, drone `GATE CENTER` (mengunci posisi tengah gerbang), lalu `GATE ADVANCE` (maju lurus **sejauh sisa jarak leg yang sebenarnya**, dikoreksi lateral terus dari Livox selama masih terlihat, lalu odometry+heading terkunci begitu gerbang keluar jangkauan sensor). Sesudah gerbang terlewati, sisa jalur RelativePath digeser mengikuti titik tengah gerbang terakhir dan `APPROACH` normal (ArUco/YOLO) lanjut seperti biasa.

Fitur ini murni kill-switch tunggal: `gate_centering_enable:=false` (default) → tidak ada subscriber `/livox/points` sama sekali, nol pengaruh ke jalur ArUco/YOLO manapun.

Cek point cloud Livox sebelum arm (kalau driver dinyalakan lewat `start_livox_lidar:=true`):

```bash
ros2 topic echo /livox/points --once
```

## Tiga Mode Launch

Semua argumen numerik (`camera_mount_yaw_deg`, `marker_center_tolerance_m`, `tfmini_range_offset`, `yolo_camera_fx_px`, dst.) sengaja **tidak ditulis** di contoh command — kalau tidak dioverride, program otomatis pakai nilai default (lihat tabel "Semua Nilai Default Parameter" di paling bawah dokumen ini). Hanya argumen `true`/`false`/`start_mode` yang perlu diketik eksplisit tiap mode.

### 1. Mode Takeoff (arm dari darat)

Program arm, offboard, takeoff vertikal, hover, cari ArUco/YOLO sendiri, lalu jalankan misi penuh.

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=takeoff \
  start_mission_after_hover:=true \
  use_lidar_altitude:=true \
  start_tfmini_lidar:=true \
  vision_lock_enable:=true \
  ground_lock_enable:=true \
  gripper_drop_enable:=true \
  gate_centering_enable:=false \
  start_livox_lidar:=false
```

### 2. Mode Handoff Tanpa Gate Centering

Drone sudah terbang manual dan ditahan Hold di RC. Program ambil alih dari posisi sekarang: sejajarkan altitude, kunci posisi ke ArUco, koreksi heading tegas kalau perlu, lalu jalankan misi (tanpa gerbang Livox).

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=airborne_handoff \
  start_mission_after_hover:=true \
  use_lidar_altitude:=true \
  start_tfmini_lidar:=true \
  vision_lock_enable:=true \
  ground_lock_enable:=true \
  marker_heading_align_enable:=true \
  gripper_drop_enable:=true \
  gate_centering_enable:=false \
  start_livox_lidar:=false
```

### 3. Mode Handoff Dengan Gate Centering (Livox)

Sama seperti mode 2, ditambah gate assist Livox aktif di setiap leg maju yang menemukan gerbang.

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=airborne_handoff \
  start_mission_after_hover:=true \
  use_lidar_altitude:=true \
  start_tfmini_lidar:=true \
  vision_lock_enable:=true \
  ground_lock_enable:=true \
  marker_heading_align_enable:=true \
  gripper_drop_enable:=true \
  gate_centering_enable:=true \
  start_livox_lidar:=true
```

> Ada mode ke-4 khusus pengujian: `start_mode:=gate_pass` menjalankan **hanya** lintasan gerbang Livox standalone (bypass total ArUco/YOLO/gripper) — dipakai untuk validasi Livox sendirian sebelum digabung ke misi penuh, bukan untuk terbang misi asli. Lihat bagian "Mode Uji: Gate Pass Standalone" di bawah kalau perlu.

## Mode Uji: Gate Pass Standalone

Untuk menguji satu gerbang Livox secara terisolasi tanpa ArUco/YOLO/gripper:

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=gate_pass \
  start_mission_after_hover:=true \
  gate_centering_enable:=true \
  start_livox_lidar:=true \
  vision_lock_enable:=false \
  ground_lock_enable:=false \
  gripper_drop_enable:=false
```

Gate assist/gate-pass belum menggantikan atau mengaktifkan FastLIO sebagai estimator. Jika FastLIO dipakai untuk local position PX4, proses dan konfigurasi `pose_republisher` harus divalidasi terpisah.

## Mode Tanpa Kamera Sama Sekali (Opsional, Untuk Validasi Dasar)

Tidak butuh kamera/marker/box — cocok untuk validasi takeoff, yaw-hold, dan RelativePath murni sebelum kamera dipasang:

```bash
cd ~/wayfix_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml \
  start_mode:=takeoff \
  start_mission_after_hover:=true \
  vision_lock_enable:=false \
  ground_lock_enable:=false \
  gate_centering_enable:=false \
  gripper_drop_enable:=false
```

## Urutan Validasi Yang Disarankan

Selalu mulai dari yang paling sederhana sebelum menyalakan fitur berikutnya:

1. `start_mission_after_hover:=false`: pastikan takeoff naik vertikal dan N/E tetap dekat anchor awal (mode hover-saja, lihat log `HOVER HOLD`).
2. Misi tanpa kamera (`vision_lock_enable:=false`, `ground_lock_enable:=false`, `gripper_drop_enable:=false`, `gate_centering_enable:=false`): validasi yaw-hold, shift 10 cm, RelativePath.
3. Aktifkan ArUco (`vision_lock_enable:=true`) dengan YOLO/gripper/gate centering tetap mati. Validasi search dan arah koreksi `camera_mount_yaw_deg`.
4. Jalankan launch kamera gabungan (`*_with_yolo`), aktifkan `ground_lock_enable:=true` (YOLO WP1) dan `gripper_drop_enable:=true`.
5. Kalau pakai `airborne_handoff`, validasi `marker_heading_align_enable:=true` dulu di darat/hover rendah — cek log `[ARUCO-HEADING]` sebelum terbang jauh.
6. Terakhir, kalau gerbang dipasang di lintasan: `gate_centering_enable:=true start_livox_lidar:=true` — mulai dari mode 3 uji standalone (`start_mode:=gate_pass`) sebelum digabung ke misi penuh (mode Launch #3 di atas).

Log shift yang normal:

```text
POST-YAW SHIFT ... DRIVE   # sekitar 7–8 cm awal, 0.20–0.30 m/s
POST-YAW SHIFT ... BRAKE   # sisa 3 cm, position hold
POST-YAW SHIFT COMPLETE    # sisa <=2 cm, speed <=0.10 m/s selama 3 tick
```

Sesudah centering/shift, log `ANCHOR COMMIT` atau `jalur direbase dan mulai maju lurus` menandakan target berikutnya sudah mengikuti posisi aktual, bukan koordinat nominal lama.

Log gate assist yang normal:

```text
GATE DETECTED: pause APPROACH, mulai CENTER.
GATE CENTERED: mulai ADVANCE lurus X.XXm (sisa leg).
GATE PASSED: path mengikuti center terbaru (lateral shift X.XXXm), lanjut APPROACH lurus.
```

## Gripper

Program publish command string ke topic gripper. Default drop aktif setelah WP1 selesai (setelah centering box YOLO selesai, atau final-hold biasa kalau `ground_lock_enable:=false`). Program menahan posisi, publish `open`, menunggu `gripper_open_wait_ticks`, publish `close`, menunggu `gripper_close_wait_ticks`, lalu lanjut waypoint berikutnya.

Node/hardware gripper fisik tetap harus jalan sendiri sebagai subscriber topic (`gripper_cmd_topic`, default `/gripper_cmd`), misalnya controller servo/GPIO di companion computer:

```bash
ros2 topic echo /gripper_cmd
```

## Lidar TF Mini

Default `px4.launch.xml` menjalankan `tfmini_i2c_node` yang publish `/range`. Kalau perlu ganti device/alamat I2C atau offset kalibrasi, override argumen numeriknya (lihat tabel default di bawah), misalnya `tfmini_range_offset:=0.05`.

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

## Lihat Semua Argumen Launch

```bash
ros2 launch px4 px4.launch.xml --show-args
```

---

## Semua Nilai Default Parameter

Argumen di bawah **tidak perlu ditulis** di command launch kecuali mau diubah dari default-nya. Semua argumen ini didefinisikan di `launch/px4.launch.xml`.

### Umum / Start Mode

| Argumen | Default |
|---|---|
| `start_mode` | `takeoff` (pilihan: `takeoff`, `airborne_handoff`, `gate_pass`) |
| `start_mission_after_hover` | `false` |
| `altitude_command_bias_m` | `0.0` |
| `use_lidar_altitude` | `true` |
| `start_tfmini_lidar` | `true` |
| `override_mission_heading` | `false` |
| `mission_heading_deg` | `0.0` |
| `mission_heading_correction_deg` | `0.0` |

### Koreksi Heading ArUco (Handoff)

| Argumen | Default |
|---|---|
| `marker_heading_align_enable` | `true` |
| `marker_heading_back_id` | `0` |
| `marker_heading_front_id` | `1` |
| `marker_heading_tolerance_deg` | `5.0` |
| `marker_heading_timeout_s` | `8.0` |
| `marker_heading_yaw_sign` | `1.0` |

### Vision Lock (ArUco)

| Argumen | Default |
|---|---|
| `vision_lock_enable` | `true` |
| `camera_mount_yaw_deg` | `0.0` |
| `vision_max_correction_m` | `0.4` |
| `marker_center_tolerance_m` | `0.10` |
| `marker_search_timeout_s` | `20.0` |

### Ground Lock (YOLO, WP1)

| Argumen | Default |
|---|---|
| `ground_lock_enable` | `true` |
| `yolo_camera_fx_px` | `640.0` |
| `yolo_camera_fy_px` | `640.0` |

### Gripper

| Argumen | Default |
|---|---|
| `gripper_drop_enable` | `true` |
| `gripper_cmd_topic` | `/gripper_cmd` |
| `gripper_open_wait_ticks` | `15` |
| `gripper_close_wait_ticks` | `15` |

### Gate Centering (Livox)

| Argumen | Default |
|---|---|
| `gate_centering_enable` | `false` |
| `start_livox_lidar` | `false` |
| `livox_publish_freq` | `10.0` |
| `livox_frame_id` | `livox_frame` |
| `livox_user_config_path` | `$(find-pkg-share livox_ros_driver2)/config/MID360_config.json` |

### TF Mini I2C

| Argumen | Default |
|---|---|
| `tfmini_bus_device` | `/dev/i2c-1` |
| `tfmini_i2c_address` | `16` |
| `tfmini_i2c_read_mode` | `command` |
| `tfmini_frame_id` | `tfmini_link` |
| `tfmini_publish_rate` | `20.0` |
| `tfmini_min_range` | `0.03` |
| `tfmini_max_range` | `12.0` |
| `tfmini_field_of_view` | `0.04` |
| `tfmini_range_offset` | `0.0` |
| `tfmini_log_rate` | `1.0` |
| `tfmini_mavlink_enabled` | `false` |
| `tfmini_mavlink_device` | `/dev/ttyAMA0` |
| `tfmini_mavlink_system_id` | `1` |
| `tfmini_mavlink_component_id` | `191` |
| `tfmini_mavlink_sensor_id` | `0` |
| `tfmini_mavlink_orientation` | `25` |
| `tfmini_mavlink_covariance` | `0` |

### Parameter Gate Centering Lanjutan (bukan launch arg — edit `config/gate_centering.yaml`)

Nilai berikut **tidak** bisa dioverride lewat `ros2 launch ... nama:=nilai`; harus diedit langsung di `src/px4/config/gate_centering.yaml` lalu rebuild/relaunch.

| Parameter YAML | Default | Dipakai di |
|---|---|---|
| `gate_centering.detection_range_min` | `1.0` m | ROI forward minimum deteksi gerbang |
| `gate_centering.detection_range_max` | `10.0` m | ROI forward maksimum deteksi gerbang |
| `gate_centering.target_gate_distance` | `1.75` m | Jarak forward ideal saat centering |
| `gate_centering.roi_lateral` | `3.0` m | ROI lateral (±) deteksi gerbang |
| `gate_centering.gate_width` | `1.5` m | Lebar gerbang yang dicari |
| `gate_centering.centering_tolerance` | `0.2` m | Toleransi dianggap "centered" |
| `gate_centering.min_cluster_points` | `5` | Titik minimum per tiang gerbang |
| `gate_pass.forward_velocity_m_s` | `1.0` m/s | Kecepatan maju, khusus `start_mode:=gate_pass` |
| `gate_pass.proportional_gain` | `0.5` | Gain koreksi lateral, khusus `start_mode:=gate_pass` |
| `gate_pass.max_lateral_velocity_m_s` | `0.3` m/s | Batas kecepatan lateral, khusus `start_mode:=gate_pass` |
| `gate_pass.pass_distance_m` | `3.5` m | Jarak ADVANCE tetap, khusus `start_mode:=gate_pass` (gate assist di misi normal pakai sisa jarak leg, bukan nilai ini) |
| `gate_pass.required_centered_ticks` | `10` | Tick stabil sebelum ADVANCE, khusus `start_mode:=gate_pass` |
| `gate_pass.center_timeout_s` | `15.0` s | Timeout CENTER, khusus `start_mode:=gate_pass` |
| `gate_pass.advance_timeout_s` | `12.0` s | Timeout dasar ADVANCE, khusus `start_mode:=gate_pass` (gate assist di misi normal skala otomatis sesuai jarak) |
