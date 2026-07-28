# Rangkuman Program PX4 Terbaru

Dokumen ini merangkum alur kerja package `px4`, parameter launch, algoritma navigasi, integrasi TF Mini lidar, dan integrasi ArUco. Waypoint LLA/GPS sudah tidak dipakai; misi sekarang berbasis local NED relatif terhadap posisi hover.

## Struktur Utama

- `mission_manager.cpp`: node utama dan state machine misi.
- `control_module.cpp`: komunikasi ROS 2 dengan PX4, TF Mini, ArUco, dan YOLO.
- `waypoint_handler.cpp`: perhitungan jarak, yaw, kecepatan horizontal/vertikal, dan smoothing yaw.
- `vision_lock.cpp`: konversi pose marker ArUco (sudah metrik) menjadi target N/E terkoreksi.
- `ground_lock.cpp`: sama seperti `vision_lock.cpp` tapi sumbernya titik tengah box YOLO dalam piksel — proyeksi ground-plane (butuh altitude lidar) dulu sebelum masuk rotasi yang sama dengan `VisionLock`. Dipakai khusus WP1, lihat bagian "Ground Lock YOLO (WP1)".
- `tfmini_i2c_ros`: driver TF Mini I2C yang publish `/range`.
- `general_box_detector_ros` (package `yolobox/ros2_ws`, workspace terpisah): `yolo_camera_node` — inferensi YOLOv8 di Hailo AI HAT, subscribe `/camera/image_raw`, publish `/general_box/target_center`.

## Topic ROS 2

Subscriber:

- `/fmu/out/vehicle_local_position`: posisi lokal PX4, yaw, dan counter reset EKF.
- `/fmu/out/vehicle_status_v1`: status armed/disarmed jika tersedia.
- `/range`: `sensor_msgs/msg/Range` dari TF Mini atau simulator.
- `/fiducial/pose`: pose marker ArUco dari `fiducial_detector`.
- `/fiducial/marker_centers`: daftar center pixel semua marker ArUco yang terlihat, dipakai untuk heading awal mode handoff.
- `/general_box/target_center`: titik tengah box (piksel) hasil deteksi YOLO dari `yolo_camera_node`, sumber centering khusus WP1 — lihat "Ground Lock YOLO (WP1)".

Publisher:

- `/fmu/in/offboard_control_mode`: heartbeat offboard.
- `/fmu/in/trajectory_setpoint`: setpoint posisi atau velocity.
- `/fmu/in/vehicle_command`: arm, mode offboard, dan land.
- `/gripper_cmd`: string `open` dan `close` untuk drop barang.
- `/mission/vision_source_active`: string `ARUCO`/`YOLO`/`NONE`, sinyal skip-inferensi untuk `aruco_node`/`yolo_camera_node` — lihat "Skip-inferensi ArUco/YOLO saat bukan giliran".

## State Machine Misi

Alur fase:

```text
INIT -> WAIT_ARM -> TAKEOFF -> HOVER -> TAKEOFF_MARKER -> MISSION -> LAND_CMD -> WAIT_DISARM
```

- `INIT`: kirim setpoint diam 5 tick, lalu set mode OFFBOARD dan ARM.
- `WAIT_ARM`: tunggu 1 detik. Jika status PX4 tersedia, status armed dicek; jika topik status tidak pernah masuk, program lanjut takeoff karena posisi tetap tersedia.
- `TAKEOFF`: mode `takeoff` naik vertikal ke altitude waypoint pertama sambil menahan N/E dan yaw awal. Mode `airborne_handoff` menahan N/E posisi handoff dan menunggu altitude lidar stabil di target awal selama 10 tick.
- `HOVER`: tahan posisi takeoff selama 30 tick atau 3 detik. Jika `start_mission_after_hover=false`, program berhenti di hover hold.
- `TAKEOFF_MARKER`: jika vision lock aktif, marker takeoff wajib ditemukan dan drone melakukan centering sebelum origin misi direbase. Khusus `airborne_handoff`, fase ini bisa lanjut ke alignment heading dua marker sebelum misi mulai.
- `MISSION`: kunjungi waypoint berurutan, dengan fase internal `APPROACH`, `SEARCH_MARKER`, `CENTER_MARKER`, dan `ALIGN_MARKER_HEADING` khusus start marker mode handoff.
- Gripper drop berjalan di dalam `MISSION`, tepat setelah WP1 (YOLO/ground-lock centered) dinyatakan selesai dan sebelum `current_wp_` naik ke waypoint berikutnya.
- `LAND_CMD`: kirim command LAND.
- `WAIT_DISARM`: tunggu PX4 disarm lalu timer berhenti.

Ctrl+C memanggil `requestGracefulLand()`, jadi node mencoba mengirim LAND sebelum keluar.

## Waypoint Dan Frame Koordinat

Waypoint internal memakai NED:

```text
{ north_m, east_m, down_m }
```

`down_m` negatif berarti naik, misalnya `-1.15` berarti altitude 1.15 m AGL. Origin local dikunci saat posisi PX4 sudah stabil 0.5 detik dalam toleransi horizontal 0.20 m. Setelah origin terkunci, posisi misi dihitung relatif dari origin itu.

Misi default sekarang memakai `RelativePath`:

```cpp
RelativePath path(/*start_altitude_agl_m=*/1.15);
path.forward(4.9)
    .turnLeft(90.0)
    .forward(5.9);
return path.build();
```

`RelativePath` membuat jalur relatif terhadap heading misi, bukan terhadap utara kompas. Default-nya heading misi mengikuti yaw drone saat origin dikunci. Jika `override_mission_heading=true`, waypoint diputar memakai `mission_heading_deg`. Jika tidak override, `mission_heading_correction_deg` dapat memberi koreksi kecil dari yaw origin.

Method yang tersedia:

- `forward(dist_m)`: maju atau mundur sesuai heading builder.
- `strafeRight(dist_m)` dan `strafeLeft(dist_m)`: geser samping tanpa mengubah heading builder.
- `turnRight(deg)` dan `turnLeft(deg)`: ubah heading builder untuk gerakan berikutnya, tidak menambah waypoint.
- `climbTo(altitude_agl_m)`: ubah altitude di posisi N/E saat ini.

Waypoint LLA/GPS tidak digunakan lagi, jadi panduan LLA lama sudah dihapus.

## Altitude Lidar Dan Gazebo

Jika `use_lidar_altitude=true`, altitude misi memakai `/range` dari TF Mini. Driver TF Mini sekarang ikut launch dari `px4.launch.xml` dengan default:

```text
bus=/dev/i2c-1
address=16 atau 0x10
mode=command
topic=/range
rate=20 Hz
```

Jika `/range` belum valid sebelum arm/takeoff, program menahan misi dan log:

```text
Menunggu data altitude TFmini /range yang valid sebelum ARM/TAKEOFF...
```

Jika `/range` stale lebih dari 0.5 detik saat misi berjalan, program mengirim LAND failsafe. Untuk Gazebo tanpa `/range`, pakai `use_lidar_altitude=false` dan `start_tfmini_lidar=false`, sehingga altitude berasal dari `vehicle_local_position.z`.

Konversi target altitude saat lidar aktif memakai error AGL:

```text
altitude_error = target_altitude_agl - lidar_altitude_m
px4_down_target = origin_down + current_px4_relative_down - altitude_error - altitude_command_bias_m
```

Ini membuat setpoint PX4 disesuaikan berdasarkan pembacaan lidar, bukan hanya z lokal PX4.

## Vision Lock ArUco

Vision lock memakai package `fiducial_detector` dengan kamera nadir. `aruco_node` publish `/fiducial/pose`, lalu `mission_manager` mengubah offset kamera menjadi offset N/E:

```text
camera optical x/y -> body forward/right -> NED memakai yaw drone -> target marker
```

Parameter `camera_mount_yaw_deg` mengoreksi rotasi mounting kamera relatif hidung drone. Ini wajib dikalibrasi di lapangan. Test minimal di dua heading berbeda agar tidak tertipu mirror atau sumbu tertukar.

`aruco_node` juga publish `/fiducial/marker_centers` sebagai `Float32MultiArray`:

```text
[frame_width, frame_height, id1, cx1, cy1, id2, cx2, cy2, ...]
```

Topic ini tidak mengganti `/fiducial/pose`; centering tetap memakai `/fiducial/pose`. Data center semua marker hanya dipakai untuk heading awal mode handoff.

Saat vision lock aktif:

- Marker dicari setelah takeoff dan di setiap waypoint.
- Program menahan titik waypoint 5 detik, lalu melakukan pencarian kecil berbentuk lingkaran.
- Radius pencarian maksimum 0.06 m, tambahan altitude maksimum 0.08 m.
- Deteksi dianggap valid setelah 5 sample segar.
- Centering dilakukan closed-loop dengan `CENTER_KP=0.18`, step maksimum 0.03 m per tick.
- Target centering dibatasi radius 0.45 m dari waypoint.
- Lock sukses setelah offset marker berada dalam toleransi selama 6 frame.
- Toleransi efektif minimal 0.15 m, atau `marker_center_tolerance_m` jika lebih besar.

Jika vision lock nonaktif, program tetap melakukan final position hold ke pusat waypoint sampai `dist <= 0.08 m` dan altitude sudah masuk toleransi selama 10 tick.

## Ground Lock YOLO (WP1)

WP1 (waypoint pertama, `current_wp_ == 0`) pakai sumber centering yang beda
dari waypoint lain: box hasil deteksi YOLO (`general_box_detector_ros`,
model Hailo `basket_box`), bukan marker ArUco. Alasannya: WP1 adalah titik
drop/ambil payload lewat gripper, dan box fisik di titik itu tidak
dipasangi marker ArUco — jadi begitu drone sampai di WP1 (setelah leg
pertama `forward`), centering-nya diganti ke YOLO, baru gripper jalan
setelah box centered. WP2 (waypoint kedua, setelah `turnLeft(90)` +
`forward`) tetap pakai marker ArUco seperti waypoint biasa, tanpa gripper.

Mekanisme:

- `yolo_camera_node` subscribe `/camera/image_raw` (kamera nadir yang sama
  dengan `aruco_node` — lihat `webcam_with_yolo.launch.xml`, keduanya boleh
  jalan bersamaan karena cuma subscriber, tidak buka device kamera sendiri)
  dan publish `/general_box/target_center`: `[frame_width, frame_height, cx,
  cy, confidence]` (cx/cy/confidence kosong kalau tidak ada box terdeteksi).
- `mission_manager` terima topic ini lewat `ControlModule::TargetCenterSample`,
  lalu `GroundLock` mengonversi piksel ke offset N/E: proyeksi ground-plane
  (`x_m = (cx-cx0)/fx * altitude`, dst., asumsi box rata di tanah) memakai
  altitude AGL dari lidar, BUKAN `solvePnP` seperti ArUco — ukuran fisik box
  tidak diketahui pasti seperti marker. Hasil proyeksi lalu diputar
  mount-yaw + yaw drone memakai instance `VisionLock` internal yang sama
  persis dengan jalur ArUco (reuse, bukan duplikasi logic).
- Selama WP1 aktif (fase `MISSION`, `current_wp_ == 0`), `mission_manager`
  mengisi variabel closed-loop centering yang sama (`marker_latest_offset_*`,
  `marker_stable_frames_`, dst.) dari `GroundLock` alih-alih `VisionLock` —
  jadi seluruh logic `SEARCH_MARKER`/`CENTER_MARKER` (search pattern,
  closed-loop correction, toleransi, dwell, gripper drop) identik dengan
  ArUco, cuma sumber datanya beda. Log field `[ARUCO]`/`[YOLO]` di terminal
  mengikuti sumber yang aktif.
- Di luar WP1, sample YOLO diabaikan untuk centering (diamkan, tidak
  menimpa data ArUco WP lain) — begitu juga sebaliknya, sample ArUco
  diabaikan selama WP1 aktif.

Parameter:

- `ground_lock_enable` (default `true`): kill-switch khusus jalur YOLO —
  terpisah dari `vision_lock_enable` supaya dua jalur bisa dites independen.
  Kalau `false` saat WP1, drone jatuh ke final-position-hold biasa (sama
  seperti `vision_lock_enable:=false` di waypoint lain) — gripper tetap
  jalan setelah dwell 10 tick itu, lihat "Alur Gripper".
- `yolo_camera_fx_px` / `yolo_camera_fy_px` (default `640.0`): fokal
  panjang kamera (piksel) untuk proyeksi ground-plane. Kalau kamera YOLO
  sama persis dengan kamera ArUco (default saat ini), isi dengan nilai
  `camera_matrix` di `src/fiducial_detector/config/params.yaml`.

Catatan: `general_box_detector_ros` ada di workspace colcon terpisah
(`src/yolobox/ros2_ws`, di-build lewat `activate.bash`-nya sendiri) — saat
launch, `wayfix_ws/install/setup.bash` DAN
`yolobox/ros2_ws/install_merged/setup.bash` harus sama-sama di-source
supaya `ros2 launch` menemukan `yolo_camera_node`.

### Skip-inferensi ArUco/YOLO saat bukan giliran (hemat CPU/NPU)

`aruco_node` dan `yolo_camera_node` dua-duanya TETAP HIDUP dan subscribe
kamera dari awal sampai akhir misi — tidak ada start/stop proses saat
pindah waypoint (start/stop proses di tengah terbang berisiko: race
condition, gagal start ulang, dst.). Yang benar-benar "gantian" adalah
KERJA BERAT-nya (deteksi ArUco / inferensi Hailo YOLO), lewat sinyal
ringan dari `mission_manager`:

- `mission_manager` publish `/mission/vision_source_active` (`std_msgs/String`)
  tiap tick 10Hz: `"ARUCO"` (HOVER, TAKEOFF_MARKER, MISSION di WP selain
  WP1), `"YOLO"` (MISSION di WP1), atau `"NONE"` (INIT/WAIT_ARM/TAKEOFF/
  LAND_CMD/WAIT_DISARM — belum/tidak ada koreksi vision yang dipakai).
- `aruco_node` dan `yolo_camera_node` subscribe topic ini. Di awal
  callback gambar, kalau sinyal terbaru mengatakan bukan giliran mereka,
  langsung `return` sebelum masuk ke bagian berat (deteksi ArUco penuh /
  `detector.infer()` Hailo) — publish topic mereka juga otomatis berhenti
  update, yang oleh sisi `mission_manager` sudah dianggap "stale" (tidak
  dipakai) lewat cek umur sample yang sudah ada.
- **Fallback aman**: kalau sinyal belum pernah diterima ATAU sudah basi
  >1 detik (mission_manager belum/tidak jalan — mis. tes standalone salah
  satu node saja tanpa `px4`), node DEFAULT TETAP JALAN PENUH seperti
  sebelum fitur ini ada. Jadi cara tes manual (`ros2 launch
  fiducial_detector realsense_with_yolo.launch.xml` tanpa `px4`, atau
  `ros2 run` satu-satu) tetap berfungsi tanpa perlu menjalankan
  `mission_manager`.

## Mode Handoff Dan Heading Dua Marker

Parameter `start_mode` memilih cara program dimulai:

- `takeoff`: default. Program arm/offboard dan takeoff dari bawah seperti alur lama.
- `airborne_handoff`: drone sudah hover dari RC Hold. Program ambil alih dari posisi sekarang, sejajarkan altitude lidar, lalu melakukan ArUco centering awal.

Khusus `airborne_handoff`, jika `marker_heading_align_enable=true`, setelah ArUco awal centered program masuk fase `ALIGN_MARKER_HEADING`. Fase ini hanya berjalan sekali di marker awal setelah switch manual ke autonomous, bukan di marker waypoint berikutnya.

Heading dihitung dari dua marker yang terlihat kamera:

- `marker_heading_back_id`: marker belakang/besar.
- `marker_heading_front_id`: marker depan/kecil.

Program mengambil center pixel marker belakang dan depan dari `/fiducial/marker_centers`, lalu menghitung error garis marker terhadap arah depan gambar. Jika garis besar->kecil masih miring, drone yaw pelan sambil tetap hold N/E dan altitude. Heading dianggap lock jika error berada di bawah `marker_heading_tolerance_deg` selama 8 tick. Jika tidak berhasil sebelum `marker_heading_timeout_s`, program lanjut memakai yaw terbaik terakhir.

Jika arah koreksi yaw terbalik saat test, ubah:

```bash
marker_heading_yaw_sign:=-1.0
```

Setelah heading locked atau timeout, waypoint `RelativePath` direbuild memakai yaw hasil alignment, origin direbase ke posisi centered sekarang, lalu misi mulai dari WP1 `forward(4.9)`.

## Alur Gripper

Gripper dikontrol dari `mission_manager` lewat publisher `std_msgs/msg/String` ke topic `gripper_cmd_topic`, default `/gripper_cmd`. Package `px4` tidak menggerakkan servo/GPIO langsung; node gripper fisik harus subscribe topic ini dan menerjemahkan command ke hardware. Command yang dikirim hanya:

```text
open
close
```

Drop barang default terjadi setelah WP1 selesai. WP1 adalah titik box
fisik tanpa marker ArUco, jadi centering-nya dipakai YOLO/ground-lock
(lihat "Ground Lock YOLO (WP1)"), bukan ArUco seperti waypoint lain —
gripper baru jalan setelah box centered, baru sesudah itu drone lanjut
yaw 90deg + leg berikutnya ke WP2 (WP2 tetap ArUco seperti biasa, tanpa
gripper). Di kode, WP1 berarti `current_wp_ == 0`
(`gripper_drop_after_wp_ = 0`). Lokasinya ada setelah waypoint
benar-benar selesai:

- Dengan YOLO/ground-lock aktif (default WP1): setelah box ditemukan,
  centering sukses, dan log `YOLO CENTERED - waypoint selesai`.
- Tanpa ground-lock (`ground_lock_enable:=false`): setelah final position
  hold ke pusat WP1 stabil selama 10 tick.

Saat drop berjalan, drone tetap hold posisi dan altitude waypoint terakhir. State gripper:

```text
IDLE -> OPEN_SENT -> CLOSE_SENT -> COMPLETE
```

Detail alur:

- `IDLE`: publish `open`, reset counter, lalu masuk `OPEN_SENT`.
- `OPEN_SENT`: tahan posisi sambil menunggu `gripper_open_wait_ticks`; default 15 tick atau sekitar 1.5 detik.
- Setelah tunggu open selesai: publish `close`, reset counter, lalu masuk `CLOSE_SENT`.
- `CLOSE_SENT`: tahan posisi sambil menunggu `gripper_close_wait_ticks`; default 15 tick atau sekitar 1.5 detik.
- `COMPLETE`: tandai `gripper_drop_completed=true`, lalu misi lanjut ke waypoint berikutnya.

Gripper bisa dinyalakan dan dimatikan dari launch seperti ArUco:

```bash
gripper_drop_enable:=true
gripper_drop_enable:=false
```

Parameter topic dan durasi juga bisa dioverride:

```bash
gripper_cmd_topic:=/gripper_cmd
gripper_open_wait_ticks:=15
gripper_close_wait_ticks:=15
```

Catatan penting: `gripper_drop_after_wp_` masih hard-coded di kode sebagai `0` (WP1), jadi dari launch saat ini yang bisa diubah adalah aktif/nonaktif, topic command, dan durasi tunggu. Jika ingin drop di WP lain, nilai `gripper_drop_after_wp_` perlu dibuat parameter atau diubah di kode.

## Algoritma Navigasi

Origin stabilization:

- Posisi PX4 pertama tidak langsung dipercaya.
- Origin dikunci setelah N/E stabil 0.5 detik dengan toleransi 0.20 m.
- Altitude utama dapat memakai lidar, jadi noise z PX4 tidak memblokir origin.

Kompensasi reset EKF:

- `xy_reset_counter`, `z_reset_counter`, dan `heading_reset_counter` dipantau.
- Saat counter berubah, origin digeser mengikuti delta reset agar posisi relatif tidak melompat.

Yaw sebelum maju:

- Setiap leg memakai bearing dari waypoint sebelumnya ke waypoint sekarang, bukan bearing sesaat dari posisi aktual.
- Drone menahan posisi sambil yaw ke arah leg.
- Yaw command memakai profil akselerasi/deselerasi: max rate sekitar 12 deg/s, akselerasi sekitar 20 deg/s2.
- Drone baru boleh maju setelah yaw aktual stabil dalam sekitar 5 deg selama 5 tick.
- Setelah yaw besar, program melakukan post-yaw shift 10 cm ke arah samping yang dikoreksi, lalu rebase waypoint agar leg berikutnya tetap lurus.

Tracking waypoint:

- Saat yaw sudah lock, velocity utama mengikuti arah leg.
- Forward speed: `clamp(along_track_remaining * 0.35, 0.20, 1.20)` m/s.
- Cross-track correction: `clamp(cross_track_error * 0.60, -0.25, 0.25)` m/s.
- Velocity horizontal dikalikan `max(0, cos(yaw_error))` agar gerak melambat jika yaw mulai meleset.
- Saat sisa along-track <= 0.45 m, mode berubah ke final position hold.

Altitude:

- Toleransi altitude 0.15 m.
- Vertical velocity maksimum 0.4 m/s.
- Pure altitude waypoint dideteksi jika jarak horizontal < 0.3 m, lalu langsung climb/descend tanpa fase yaw.

Waypoint reached:

- Radius waypoint approach 0.30 m.
- Dengan ArUco, reached hanya memicu `SEARCH_MARKER`; waypoint baru selesai setelah marker centered.
- Tanpa ArUco, reached memicu final hold ke pusat waypoint.

## Failsafe Dan Guard

- Position stale warning jika `/fmu/out/vehicle_local_position` tidak update lebih dari 0.3 detik.
- Position stale LAND jika posisi stale lebih dari 1.5 detik.
- Lidar stale LAND jika `/range` tidak valid/stale lebih dari 0.5 detik saat misi berjalan.
- Altitude tidak masuk akal LAND jika altitude relatif melewati 8.0 m.
- Vision correction diclamp oleh `vision_max_correction_m`, default 0.4 m.
- Saat marker hilang di fase centering, target terakhir ditahan.

## Parameter Launch Utama

| Parameter | Default | Fungsi |
| --- | --- | --- |
| `altitude_command_bias_m` | `0.0` | Bias tambahan untuk target down PX4. |
| `use_lidar_altitude` | `true` | `true` memakai `/range`, `false` memakai local z PX4. |
| `start_tfmini_lidar` | `true` | Menyalakan node TF Mini dari launch PX4. |
| `start_mode` | `takeoff` | `takeoff` untuk mode lama, `airborne_handoff` untuk switch RC Hold ke autonomous. |
| `start_mission_after_hover` | `false` | `false` untuk tes hover saja, `true` untuk lanjut misi. |
| `override_mission_heading` | `false` | Jika `true`, waypoint diputar memakai `mission_heading_deg`. |
| `mission_heading_deg` | `0.0` | Heading absolut misi saat override aktif. |
| `mission_heading_correction_deg` | `0.0` | Koreksi heading dari yaw origin jika override mati. |
| `vision_lock_enable` | `true` | Aktifkan pencarian dan centering ArUco. |
| `camera_mount_yaw_deg` | `0.0` | Rotasi kamera relatif hidung drone. |
| `vision_max_correction_m` | `0.4` | Batas koreksi vision lock. |
| `marker_center_tolerance_m` | `0.10` | Toleransi centering marker. |
| `marker_search_timeout_s` | `20.0` | Waktu warning jika marker belum ditemukan. |
| `ground_lock_enable` | `true` | Kill-switch centering YOLO khusus WP1 (lihat "Ground Lock YOLO (WP1)"). |
| `yolo_camera_fx_px` | `640.0` | Fokal panjang kamera (piksel) untuk proyeksi ground-plane YOLO. |
| `yolo_camera_fy_px` | `640.0` | Sama seperti di atas, sumbu vertikal. |
| `marker_heading_align_enable` | `true` | Aktifkan alignment heading dua marker khusus `airborne_handoff`. |
| `marker_heading_back_id` | `0` | ID marker belakang/besar untuk heading handoff. |
| `marker_heading_front_id` | `1` | ID marker depan/kecil untuk heading handoff. |
| `marker_heading_tolerance_deg` | `5.0` | Error maksimum heading dua marker agar dianggap lurus. |
| `marker_heading_timeout_s` | `8.0` | Timeout alignment heading handoff. |
| `marker_heading_yaw_sign` | `1.0` | Arah koreksi yaw; ubah ke `-1.0` jika yaw menjauh saat test. |
| `gripper_drop_enable` | `true` | Aktifkan drop barang setelah WP1 (YOLO/ground-lock centered). |
| `gripper_cmd_topic` | `/gripper_cmd` | Topic command gripper. |
| `gripper_open_wait_ticks` | `15` | Durasi tunggu setelah `open`, tick 10 Hz. |
| `gripper_close_wait_ticks` | `15` | Durasi tunggu setelah `close`, tick 10 Hz. |

## Parameter TF Mini

| Parameter | Default | Fungsi |
| --- | --- | --- |
| `tfmini_bus_device` | `/dev/i2c-1` | Bus I2C Raspberry Pi. |
| `tfmini_i2c_address` | `16` | Address TF Mini desimal, sama dengan 0x10. |
| `tfmini_i2c_read_mode` | `command` | Mode baca SMBus yang sudah cocok dengan script Python. |
| `tfmini_frame_id` | `tfmini_link` | Frame ID untuk message Range. |
| `tfmini_publish_rate` | `20.0` | Frekuensi publish `/range`. |
| `tfmini_min_range` | `0.03` | Minimum range valid. |
| `tfmini_max_range` | `12.0` | Maximum range valid. |
| `tfmini_field_of_view` | `0.04` | Field of view message Range. |
| `tfmini_range_offset` | `0.0` | Offset jarak meter, misalnya `0.05` untuk tambah 5 cm. |
| `tfmini_log_rate` | `1.0` | Frekuensi log jarak. |
| `tfmini_mavlink_enabled` | `false` | Kirim DISTANCE_SENSOR MAVLink ke Pixhawk. |
| `tfmini_mavlink_device` | `/dev/ttyAMA0` | Serial MAVLink jika enabled. |
| `tfmini_mavlink_system_id` | `1` | MAVLink system id. |
| `tfmini_mavlink_component_id` | `191` | MAVLink component companion. |
| `tfmini_mavlink_sensor_id` | `0` | ID sensor rangefinder. |
| `tfmini_mavlink_orientation` | `25` | Orientasi downward. |
| `tfmini_mavlink_covariance` | `0` | Covariance DISTANCE_SENSOR. |

## Ringkasan Panduan Lama Yang Dihapus

Isi `GAZEBO_LIDAR_TEST.md` sudah dipindah ke bagian altitude lidar dan Gazebo: gunakan `/range` untuk lidar fisik, atau `use_lidar_altitude=false start_tfmini_lidar=false` untuk Gazebo tanpa range.

Isi `LLA_WAYPOINT_GUIDE.md` yang relevan diganti dengan penjelasan `RelativePath` dan NED lokal. LLA/GPS tidak dipakai lagi.

Isi `VISION_LOCK_GUIDE.md` sudah dipindah ke bagian vision lock: ArUco wajib dikalibrasi, `camera_mount_yaw_deg` harus diuji di lapangan, dan mode tanpa ArUco bisa dimatikan dengan `vision_lock_enable=false`.
