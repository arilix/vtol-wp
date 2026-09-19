# Rangkuman Program PX4 Mission Manager

Dokumen ini menjelaskan dari nol bagaimana package `px4` (node `mission_manager`) bekerja: data apa yang dipakai, dari mana asalnya, algoritma apa yang mengolahnya, dan bagaimana semuanya terangkai jadi satu state machine penerbangan otonom. Waypoint LLA/GPS tidak dipakai — semua misi berbasis **local NED** relatif terhadap posisi drone saat origin dikunci.

---

## 1. Filosofi Desain

- **Satu node ROS 2** (`mission_manager`), state machine sinkron, di-tick oleh `rclcpp::TimerBase` **10 Hz** (`timer_ = create_wall_timer(100ms, ...)`). Tidak ada thread lain, tidak ada callback yang langsung mengirim command ke PX4 — semua keputusan kontrol terjadi di `loop()` dan fungsi `run*()` yang dipanggilnya per fase.
- **Kelas matematika murni** (`WaypointHandler`, `VisionLock`, `GroundLock`, `GateCenteringLock`) sengaja tidak menyentuh ROS/rclcpp sama sekali — supaya gampang diuji dan supaya `MissionManager` tidak bercampur antara "logic" dan "I/O".
- **`ControlModule`** adalah satu-satunya kelas yang menyentuh publisher/subscriber PX4 dan sensor. `MissionManager` memanggilnya lewat command sederhana (`sendPositionSetpoint`, `sendVelocitySetpoint`, dst.) dan menerima data lewat callback.
- **Kill-switch per fitur**: `vision_lock_enable`, `ground_lock_enable`, `gate_centering_enable`, `marker_heading_align_enable`, `gripper_drop_enable` — semuanya bisa dimatikan independen dari command line tanpa mengubah kode, dan saat mati subscriber-nya (kalau ada, seperti Livox) benar-benar tidak dibuat sama sekali (bukan cuma diabaikan datanya).

## 2. Struktur File

| File | Tanggung jawab |
|---|---|
| `core/mission_manager.cpp` + `utils/mission_manager.h` | Node utama, state machine, semua keputusan misi. |
| `core/control_module.cpp` + `utils/control_module.h` | I/O murni ke PX4 (uXRCE-DDS), TF Mini, ArUco, YOLO, Livox — tidak ada logic keputusan. |
| `core/waypoint_handler.cpp` + `utils/waypoint_handler.h` | Matematika navigasi: jarak, yaw target, profil kecepatan, kriteria "reached". Tanpa ROS. |
| `core/vision_lock.cpp` + `utils/vision_lock.h` | Ubah offset marker ArUco (meter, sudah metrik) jadi target NED terkunci, dengan debounce + freeze. Tanpa ROS. |
| `core/ground_lock.cpp` + `utils/ground_lock.h` | Sama seperti `VisionLock`, tapi sumbernya titik tengah box YOLO (piksel) — proyeksi ground-plane dulu, lalu reuse `VisionLock` internal. Tanpa ROS. |
| `core/gate_centering_lock.cpp` + `utils/gate_centering_lock.h` | Deteksi dua tiang gerbang dari point cloud Livox (ROI + binning + puncak histogram). Tanpa ROS. Diporting dari `centeringGateLivoxSimple()` di `centering_ws` (referensi sumber, bukan dependency runtime — lihat `LAUNCH_TUTORIAL.md`). |
| `utils/vehicle_state.h` | Struct data (`PositionNED`, `VehicleState`) — posisi/kecepatan/yaw drone saat ini dalam frame misi. |
| `tfmini_i2c_ros` (package terpisah) | Driver TF Mini I2C, publish `/range`. |
| `general_box_detector_ros` (workspace `yolobox/ros2_ws`, terpisah) | `yolo_camera_node`: inferensi YOLOv8 di Hailo AI HAT, publish `/general_box/target_center`. |
| `livox_ros_driver2` (package di `wayfix_ws`) | Driver Livox MID360, publish `/livox/points` (`PointCloud2`). |

## 3. Data & Sensor Yang Dipakai

Ini bagian paling penting untuk dipahami: **program tidak pakai satu sumber odometry tunggal** — posisi/kecepatan/yaw datang dari estimator lokal PX4 (EKF2), sedangkan altitude bisa memakai lidar sebagai override, dan koreksi posisi horizontal memakai vision (ArUco/YOLO) atau Livox tergantung fase.

### 3.1 Posisi & odometry — `/fmu/out/vehicle_local_position` (`px4_msgs::msg::VehicleLocalPosition`)

Ini **satu-satunya sumber odometry** program (posisi X/Y/Z lokal PX4 + kecepatan + yaw + counter reset EKF). Field yang benar-benar dipakai (lihat `ControlModule::PositionSample`, `control_module.cpp::onPosition`):

- `x, y, z`: posisi lokal PX4 (frame NED PX4, meter) — **bukan** posisi GPS absolut, ini estimator EKF2 lokal (bisa berasal dari fusion GPS+baro+IMU, atau dari external vision/FastLIO kalau dikonfigurasi begitu di PX4, tergantung setup `EKF2_*` param di flight controller; `mission_manager` sendiri tidak peduli sumber aslinya, hanya konsumsi output `/fmu/out/vehicle_local_position`).
- `vx, vy, vz`: kecepatan lokal — dipakai untuk deteksi "sudah berhenti"/settle check (bukan untuk kontrol langsung).
- `heading`: yaw drone (radian) — dipakai sebagai `vehicle_.yaw`, referensi seluruh perhitungan body→NED.
- `xy_reset_counter`, `z_reset_counter`, `heading_reset_counter` + `delta_xy`, `delta_z`, `delta_heading`: penanda EKF2 melakukan reset internal (re-fusi GPS/baro/vision). **Tanpa data ini drone akan "mengejar" lompatan estimator sebagai gerakan nyata** — lihat §4.2.

QoS: `BestEffort + TransientLocal` (disamakan dengan publisher uXRCE-DDS PX4, `KeepLast(1)`).

### 3.2 Altitude — `/range` (`sensor_msgs::msg::Range`, dari TF Mini I2C)

Kalau `use_lidar_altitude:=true` (default), **altitude AGL memakai pembacaan lidar TF Mini**, bukan `z` dari EKF PX4 — baro/GPS altitude sering kurang presisi untuk operasi dekat tanah/box. Kalau `false`, altitude memakai `-vehicle_.position.down` (relatif terhadap origin lokal PX4).

- `sensor_msgs::msg::Range::range` divalidasi terhadap `min_range`/`max_range` (dari driver TF Mini) sebelum dipakai.
- Kalau `/range` stale >0.5 detik saat misi berjalan → failsafe LAND (§9).

### 3.3 ArUco — `/fiducial/pose` dan `/fiducial/marker_centers` (dari `fiducial_detector`/`aruco_node`)

- **`/fiducial/pose`** (`geometry_msgs::msg::PoseStamped`): pose marker hasil `solvePnP` (sudah metrik, satuan meter) dari kamera nadir. Dipakai sebagai sumber utama `VisionLock` untuk centering posisi (§6). Field yang dipakai cuma `x, y` (offset kamera, frame optical OpenCV: x=kanan, y=bawah) — orientasi marker (rotasi) **tidak** dipakai dari topic ini.
- **`/fiducial/marker_centers`** (`std_msgs::msg::Float32MultiArray`, layout `[frame_w, frame_h, id1, cx1, cy1, id2, cx2, cy2, ...]`): daftar titik tengah **piksel mentah** (bukan metrik) semua marker yang terlihat kamera dalam satu frame, dengan ID masing-masing. Dipakai untuk dua hal: (a) centering posisi awal saat mode `airborne_handoff` sebelum offboard takeover, (b) **koreksi heading dua-marker** (§7) — di sinilah ID marker jadi penting, karena `/fiducial/pose` tidak membawa ID/orientasi.

### 3.4 YOLO — `/general_box/target_center` (dari `yolo_camera_node`, khusus WP1)

`std_msgs::msg::Float32MultiArray`: `[frame_w, frame_h, cx, cy, confidence]` (cx/cy/confidence kosong kalau tidak ada box, atau kalau terdeteksi lebih dari satu box — sengaja ditolak karena box tidak punya ID unik seperti ArUco, jadi ambigu). Titik tengah piksel ini **bukan** hasil `solvePnP` (ukuran fisik box tidak diketahui pasti) — dikonversi ke meter lewat proyeksi ground-plane yang butuh altitude AGL (§3.2, §6.2).

### 3.5 Livox — `/livox/points` (`sensor_msgs::msg::PointCloud2`, khusus gate centering)

Point cloud 3D body-frame dari Livox MID360 (`x`=forward, `y`=lateral kanan-positif, `z`=height). **Subscriber ini hanya dibuat kalau `gate_centering_enable:=true`** — kill-switch keras di level konstruksi objek (`ControlModule(node, enable_livox)`), bukan cuma diabaikan datanya. Dipakai `GateCenteringLock` untuk deteksi dua tiang gerbang (§8).

### Ringkasan tabel topic

| Topic | Tipe | Arah | Dipakai untuk |
|---|---|---|---|
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | sub | **Odometry** — posisi, kecepatan, yaw, reset EKF |
| `/fmu/out/vehicle_status`, `/fmu/out/vehicle_status_v1` | `VehicleStatus` | sub | Status armed/disarmed (opsional, sering tidak pernah terisi — lihat §5) |
| `/range` | `sensor_msgs/Range` | sub | Altitude AGL (TF Mini) |
| `/fiducial/pose` | `geometry_msgs/PoseStamped` | sub | Centering ArUco (metrik, `VisionLock`) |
| `/fiducial/marker_centers` | `std_msgs/Float32MultiArray` | sub | Heading dua-marker + posisi awal handoff (piksel, per-ID) |
| `/general_box/target_center` | `std_msgs/Float32MultiArray` | sub | Centering box YOLO WP1 (piksel, `GroundLock`) |
| `/livox/points` | `sensor_msgs/PointCloud2` | sub | Gate centering (opt-in) |
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | pub | Heartbeat offboard (wajib >2Hz selama offboard) |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | pub | Setpoint posisi ATAU velocity (tidak pernah dua-duanya sekaligus) |
| `/fmu/in/vehicle_command` | `VehicleCommand` | pub | Arm, set mode (Offboard/Position), Land |
| `/gripper_cmd` | `std_msgs/String` | pub | `"open"` / `"close"` ke hardware gripper eksternal |
| `/mission/vision_source_active` | `std_msgs/String` | pub | `"ARUCO"`/`"YOLO"`/`"NONE"` — sinyal skip-inferensi (§6.3) |

## 4. Frame Koordinat & Origin

### 4.1 Origin locking

Sample posisi PX4 pertama **tidak langsung dipercaya** — EKF2 baru boot bisa belum konvergen (terutama Z, bisa meleset puluhan meter). Origin (`origin_north_/east_/down_/yaw_`) baru dikunci setelah posisi mentah PX4 terbukti stabil: horizontal tidak bergeser lebih dari **0.20 m** selama **0.5 detik** berturut-turut (`ORIGIN_STABLE_TOL_M`, `ORIGIN_STABLE_DUR_S` di `mission_manager.cpp`). Setelah terkunci, `vehicle_.position = {0,0,0}` relatif terhadap origin itu, dan seluruh perhitungan misi (`waypoints_`, target N/E) memakai frame relatif ini.

### 4.2 Kompensasi reset EKF

PX4 EKF2 kadang melakukan reset internal (re-fusi GPS/baro/vision) yang membuat `x/y/z/heading` melompat instan walau posisi fisik drone tidak berubah. Program memantau `xy_reset_counter`, `z_reset_counter`, `heading_reset_counter` — begitu salah satu berubah, `origin_north_/east_/down_` digeser sebesar `delta_x/delta_y/delta_z` yang sama, sehingga `vehicle_.position` (posisi RELATIF) tetap kontinu. Tanpa ini, drone akan mengira EKF jump sebagai gerakan sungguhan dan "mengejar" balik ke posisi lama sejauh beberapa meter — bug nyata yang pernah terjadi di lapangan.

### 4.3 RelativePath — mendefinisikan misi tanpa tahu kompas

Waypoint didefinisikan lewat builder `RelativePath` (mission_manager.cpp, anonymous namespace) dalam "frame lokal drone" — heading 0 = arah hidung drone saat origin dikunci, BUKAN utara kompas:

```cpp
RelativePath path(/*start_altitude_agl_m=*/1.00);
path.forward(5.1)
    .turnLeft(91.0)
    .forward(6.0);
return path.build();
```

Method yang tersedia: `forward(dist_m)` (maju/mundur, menambah 1 waypoint), `strafeRight/strafeLeft(dist_m)` (geser tegak lurus tanpa ubah heading builder), `turnRight/turnLeft(deg)` (ubah heading builder untuk `forward()` berikutnya, **tidak** menambah waypoint / tidak memerintah yaw fisik — itu murni matematika offset), `climbTo(altitude_agl_m)` (naik/turun di N/E sekarang).

Hasil `RelativePath` diputar ke NED sungguhan memakai `origin_yaw_` (yaw drone sebenarnya saat origin dikunci) lewat `rotateToTrueNed()`, baru dibandingkan dengan `vehicle_.position`. Kalau `override_mission_heading:=true`, dipakai `mission_heading_deg` absolut sebagai ganti `origin_yaw_`; kalau tidak, `mission_heading_correction_deg` bisa memberi koreksi kecil dari yaw origin.

`down_m` di semua struct `Waypoint` **negatif berarti naik** (konvensi NED): `-1.15` berarti target altitude 1.15 m AGL.

## 5. State Machine

```text
INIT -> WAIT_ARM -> TAKEOFF -> HOVER -> TAKEOFF_MARKER -> MISSION -> LAND_CMD -> WAIT_DISARM
                       ^
   (airborne_handoff)  |
INIT -> PILOT_ARUCO_SEARCH -> TAKEOFF -> TAKEOFF_MARKER -> MISSION -> LAND_CMD -> WAIT_DISARM
```

- **`INIT`**: mode `takeoff` — kirim setpoint diam 10 tick (prestream wajib PX4 sebelum Offboard diterima), kunci `takeoff_hold_north/east/yaw_`, lalu `setOffboardMode()` + `arm()`. Mode `airborne_handoff` — tick pertama paksa **Position mode** (memutus Offboard sisa run sebelumnya, pilot pegang kendali penuh via RC), tunggu 1 detik, lalu masuk `PILOT_ARUCO_SEARCH`.
- **`WAIT_ARM`**: tunggu status armed dari `/fmu/out/vehicle_status(_v1)`. **Catatan lapangan penting**: topik ini terbukti sering tidak pernah terisi sama sekali (`status_update_count_` tetap 0 — kemungkinan mismatch skema pesan versi `px4_msgs`). Guard lama yang memblokir/abort berdasarkan data ini menyebabkan false-abort. Fix: tunggu jeda singkat (~0.2–1 detik) lalu lanjut TAKEOFF apa pun kondisinya — **kecuali** status memang tersedia DAN eksplisit menyatakan bukan armed, baru misi dibatalkan aman (`timer_->cancel()`).
- **`PILOT_ARUCO_SEARCH`** (khusus `airborne_handoff`): pilot masih pegang kendali (Position mode/RC). Program menunggu **5 sample ArUco besar berturut-turut** (marker `marker_heading_back_id_`, atau satu-satunya marker kalau cuma satu yang terlihat) dengan umur sample ≤0.35 detik. Begitu 5/5, mulai prestream Offboard (10 tick, PX4 tetap Position mode, pilot tetap pegang stick), lalu ambil alih (`setOffboardMode()`) **hanya kalau marker masih fresh saat itu juga** — kalau tidak, tunda sampai marker muncul lagi (mencegah takeover berdasarkan sample basi). Begitu takeover, posisi+yaw aktual pilot langsung jadi anchor (`handoff_takeover_anchor_`, `takeoff_hold_yaw_`) — **tidak ada centering tambahan di titik ini**, itu dilakukan belakangan di `TAKEOFF_MARKER`.
- **`TAKEOFF`**: mode `takeoff` — naik vertikal (velocity-Z sampai 15 cm dari target, lalu position-capture) sambil menahan N/E/yaw di anchor yang dikunci sebelum ARM. Mode `airborne_handoff` — menahan N/E/yaw hasil takeover sambil menyejajarkan altitude lidar ke `marker_search_altitude_m_` (altitude aktual saat marker ditemukan pilot), stabil 5 tick (`|altitude_error| <= 0.15m`). Origin RelativePath baru di-rebase ke posisi aktual setelah tahap ini (lihat alur di bawah).
- **`HOVER`** (mode `takeoff` saja): tahan posisi 30 tick (~3 detik). Kalau `start_mission_after_hover:=false`, program berhenti permanen di sini (mode uji hover).
- **`TAKEOFF_MARKER`**: marker awal wajib ditemukan + centered sebelum origin misi direbase. Detail lengkap di §6 dan §7 (koreksi heading, khusus `airborne_handoff`).
- **`MISSION`**: kunjungi semua waypoint berurutan, fase internal `APPROACH` → `SEARCH_MARKER` → `CENTER_MARKER` (§6, §8). Gate assist Livox (§8) bisa menyela `APPROACH` di tengah leg kalau `gate_centering_enable:=true`. Gripper (§10) jalan di dalam `CENTER_MARKER` tepat setelah WP1 centered.
- **`LAND_CMD`**: kirim command LAND (heartbeat tetap dikirim selama menunggu).
- **`WAIT_DISARM`**: tunggu PX4 konfirmasi disarmed, lalu `timer_` berhenti.

Ctrl+C memanggil `requestGracefulLand()` — kirim LAND dulu sebelum node berhenti, tidak mengandalkan offboard-loss failsafe PX4.

## 6. Vision Lock (ArUco) & Ground Lock (YOLO WP1)

### 6.1 Vision Lock (ArUco) — algoritma

`VisionLock` (kelas murni) mengubah offset marker (meter, frame kamera) jadi target NED terkunci:

```text
camera optical x/y  →  body forward/right (rotasi camera_mount_yaw_deg)
                    →  NED (rotasi yaw drone saat ini)
                    →  target = posisi_drone_sekarang + offset_NED
```

Strategi kuncinya: target **dihitung ulang setiap sample baru** (closed-loop, bukan one-shot), dan karena `posisi_drone_sekarang` sudah dikompensasi drift/reset EKF (§4.2), penjumlahan ini otomatis menghapus drift NED yang terakumulasi. "Freeze saat marker hilang" didapat gratis: `locked_target_` cuma berubah di dalam `update()`, jadi tanpa sample baru = tidak ada perubahan.

Guard di dalam `VisionLock::update()`:
- **Debounce**: perlu `min_consecutive_samples` (default 5) sample berturut-turut dengan jeda < `max_sample_gap_s` (0.3s) sebelum `isEngaged()` true.
- **Clamp magnitude**: koreksi dibatasi `max_correction_m` (default 0.4m) — kalau sample mendekati batas ini, itu tanda kalibrasi/deteksi salah, bukan koreksi besar yang valid.
- **Jump reject**: lompatan >`max_jump_m` (0.25m) antar sample RAW berturut-turut mereset counter debounce ke 1 (bukan 0) — `/fiducial/pose` tidak membawa ID marker, jadi ini satu-satunya sinyal "loncat ke marker lain".
- Tidak ada smoothing tambahan di sini — `aruco_node` sudah menghaluskan tvec (alpha 0.4) sebelum publish; smoothing kedua cuma menambah lag.

Alur pemakaian di `MissionManager` (sama untuk marker awal takeoff dan tiap waypoint ArUco):

1. **`SEARCH_MARKER`**: drone sweep kecil (lihat tabel di bawah) sambil menunggu sample valid. Marker awal mode `takeoff`: tahan 5 detik lalu sweep maks 0.06m + naik altitude maks 0.08m. Marker awal mode `airborne_handoff`/WP2: sweep 0.03m, altitude tetap. WP1 (YOLO): sweep maks 0.50m, tumbuh 0.05 m/s. Valid setelah **5 sample segar** berturut (2 untuk YOLO WP1).
2. **`CENTER_MARKER`**: closed-loop correction, `CENTER_KP=0.18`, step maksimum **0.03 m/tick**, target dibatasi radius **0.45 m** dari waypoint nominal. Lock sukses setelah offset dalam toleransi selama **6 frame** berturut (2 untuk YOLO WP1, toleransi 0.07m). Toleransi efektif ArUco minimal 0.15m (atau `marker_center_tolerance_m` kalau lebih besar).
3. Marker hilang >3 detik saat centering → kembali ke `SEARCH_MARKER` (tidak menunggu selamanya), sebelum itu target terakhir ditahan.

Kalau `vision_lock_enable:=false`, waypoint langsung final-position-hold ke pusat nominal (`dist<=0.08m`, altitude tercapai, speed rendah selama 10 tick) — tidak menunggu marker yang tidak akan pernah datang.

### 6.2 Ground Lock (YOLO, khusus WP1)

WP1 (`current_wp_==0`) adalah titik drop payload lewat gripper — box fisiknya **tidak** dipasangi marker ArUco, jadi centering-nya diganti sumber: `GroundLock` (bukan `VisionLock`) mengonversi piksel box `(cx,cy)` ke meter lewat **proyeksi ground-plane** (asumsi box rata di tanah, jarak tegak lurus kamera-ke-tanah = altitude AGL dari lidar):

```text
x_m = (cx_px - cx0_px) / fx_px * altitude_m     (kanan gambar)
y_m = (cy_px - cy0_px) / fy_px * altitude_m     (bawah gambar)
```

Beda dari ArUco: **bukan** `solvePnP` (ukuran fisik box tidak diketahui pasti seperti marker), dan **butuh altitude AGL valid** (sample diabaikan kalau altitude ≤0 — proyeksi jadi tidak berarti). Setelah dapat `x_m/y_m`, hasilnya diputar mount-yaw + yaw drone memakai instance `VisionLock` internal yang sama persis dengan jalur ArUco (reuse, debounce/clamp/freeze identik).

Selama WP1 aktif, `mission_manager` mengisi variabel closed-loop yang sama (`marker_latest_offset_*`, `marker_stable_frames_`) dari `GroundLock` — jadi seluruh mesin `SEARCH_MARKER`/`CENTER_MARKER` (search pattern, closed-loop, toleransi, dwell, gripper) identik dengan ArUco, cuma sumber datanya beda. Di luar WP1, sample YOLO diabaikan untuk centering (dan sebaliknya).

**Kecepatan/ketegasan centering YOLO** (revisi terbaru): target centering (`marker_center_target_`) mengejar `ground_lock_->lockedTarget()` lewat **filter adaptif**, bukan gain tetap — makin jauh dari pusat box, makin tegas dikejar (setpoint langsung dipush dekat titik ukur supaya PX4 segera membangun kecepatan, tidak "merayap" saat berangin); makin dekat pusat, makin diredam (supaya jitter deteksi box tidak memicu osilasi tepat sebelum drop):

| Offset terukur | Alpha filter/tick |
|---|---|
| > 0.15 m | 0.70 (tegas) |
| 0.05 – 0.15 m | 0.55 |
| < 0.05 m | 0.35 (redam, dekat pusat) |

Guard tambahan khusus YOLO: **divergence guard** — kalau offset terukur konsisten membesar dari best-offset tercatat (margin 0.12m) selama 6 tick berturut, drone HOLD posisi total (bukan terus mengoreksi ke arah yang mungkin salah) sampai pengukuran stabil lagi — mencegah spiral menjauh dari pusat akibat lag/overshoot bbox. **Drop interlock**: begitu lock tercapai dan gripper mulai proses, kalau box ternyata bergeser >5cm dari pusat atau sample jadi basi, lock dibatalkan otomatis dan centering dilanjutkan — payload tidak pernah di-drop di titik yang salah.

Kill-switch: `ground_lock_enable:=false` → WP1 jatuh ke final-position-hold biasa (sama seperti ArUco mati di WP lain), gripper tetap jalan setelah dwell 10 tick.

### 6.3 Skip-inferensi ArUco/YOLO saat bukan giliran

`aruco_node` dan `yolo_camera_node` **tetap hidup** dan subscribe kamera dari awal sampai akhir misi (start/stop proses di tengah terbang berisiko race condition). Yang gantian cuma **kerja beratnya** (deteksi ArUco penuh / inferensi Hailo): `mission_manager` publish `/mission/vision_source_active` tiap tick (`"ARUCO"`/`"YOLO"`/`"NONE"`), kedua node subscribe topic ini dan `return` di awal callback gambar kalau bukan giliran mereka. **Fallback aman**: kalau sinyal belum pernah diterima atau basi >1 detik (mis. tes node standalone tanpa `px4`), node default tetap jalan penuh seperti sebelum fitur ini ada.

## 7. Koreksi Heading Dua-Marker (Khusus `airborne_handoff`)

Ini fitur untuk memastikan **arah hadap drone benar-benar lurus terhadap orientasi fisik ArUco/arena**, bukan cuma mengandalkan estimasi yaw pilot saat switch RC ke autonomous. Berjalan **sekali saja**, di `TAKEOFF_MARKER` fase `ALIGN_MARKER_HEADING`, tepat setelah posisi terkunci ke marker besar (bukan di tiap waypoint berikutnya).

Butuh dua marker terlihat kamera bersamaan di titik start:

- `marker_heading_back_id_` (default `0`): marker belakang/besar — juga dipakai sebagai referensi posisi (§6.1).
- `marker_heading_front_id_` (default `1`): marker depan/kecil — cuma dipakai untuk menentukan arah.

**Algoritma** (dieksekusi setelah 3 tick body-centered ke marker belakang — `position_locked`):

1. Ambil pixel-center kedua marker dari `/fiducial/marker_centers` (fresh, umur ≤0.35s).
2. Hitung vektor `(front - back)` dalam piksel, konversi ke body-frame lewat rotasi `camera_mount_yaw_deg` (transform yang sama seperti proyeksi ground-plane di §6.2, tapi di sini dipakai untuk arah bukan posisi):
   ```text
   optical_right = dx_px / fx_px
   optical_up    = -dy_px / fy_px
   body_forward  = optical_up*cos(mount) - optical_right*sin(mount)
   body_right    = optical_up*sin(mount) + optical_right*cos(mount)
   error = atan2(body_right, body_forward) * marker_heading_yaw_sign_
   ```
   `error` = 0 kalau garis marker-belakang→marker-depan persis sejajar hidung drone (heading pilot sudah pas terhadap arah depan ArUco).
3. **Kalau `|error| <= marker_heading_tolerance_deg_`** (default 5°): heading **tidak diubah sama sekali**, langsung lanjut ke altitude align — tidak ada delay, tidak ada gerakan percuma.
4. **Kalau lebih besar**: satu kali putar di tempat yang **tegas** (bukan koreksi berulang/oscillating) memakai profil accel-limited (mirip turn antar-waypoint di §11, tapi lebih cepat: rate maks 0.45 rad/s ≈26°/s, akselerasi 1.2 rad/s², dibatasi maksimum **±45°** untuk keamanan — error sebesar itu lebih mungkin salah pairing ID daripada heading nyata). Posisi tetap dikunci di titik hasil centering selama putaran. Log: `[ARUCO-HEADING]`.
5. Kalau marker depan tidak terlihat bersamaan dengan marker belakang (fresh), program **fail-open**: heading pilot dipakai apa adanya, tidak ada yang menahan misi.

Kalau arah koreksi ternyata terbalik di lapangan (error makin besar saat drone berputar), balik konvensinya lewat parameter, tanpa perlu ubah kode:

```bash
marker_heading_yaw_sign:=-1.0
```

Setelah heading final (dikoreksi atau dilewati), `takeoff_hold_yaw_`/`origin_yaw_` diperbarui, `RelativePath` di-rebuild memakai yaw ini, origin misi di-rebase ke posisi aktual, lalu lanjut ke altitude align → `MISSION`.

## 8. Gate Centering (Livox MID360, Opt-in)

### 8.1 Isolasi

`gate_centering_enable_` (default `false`) adalah kill-switch keras: saat `false`, `ControlModule` tidak membuat subscriber `/livox/points` sama sekali (bukan cuma mengabaikan data) — nol pengaruh ke jalur ArUco/YOLO. Matematika deteksinya (`GateCenteringLock`) diporting dari `centering_ws` (referensi sumber saja, lihat `LAUNCH_TUTORIAL.md`) tapi diintegrasikan ulang ke state machine misi ini.

### 8.2 Algoritma deteksi (`GateCenteringLock::update`)

Input: titik body-frame Livox (forward=+X, lateral=+Y kanan-positif, height=Z), langsung dari `/livox/points` per tick.

1. **Filter ROI**: buang titik non-finite, lalu filter `forward ∈ [roi_forward_min_m, roi_forward_max_m]`, `lateral ∈ [-roi_lateral_m, +roi_lateral_m]`, `height ∈ [roi_height_min_m, roi_height_max_m]`.
2. **Binning lateral**: titik ROI dikelompokkan ke `num_bins` (default 60) bin sepanjang sumbu lateral.
3. **Deteksi dua puncak** (dua tiang gerbang): cari bin dengan count terbanyak (`first_peak`), lalu cari puncak kedua (`second_peak`) dengan jarak minimum `min_peak_separation` (setengah `gate_width_m` dalam satuan bin) dari puncak pertama — supaya tidak salah mendeteksi dua bin dari tiang yang sama sebagai dua tiang berbeda.
4. **Kasus satu tiang saja** (gerbang di tepi ROI, `allow_one_side_centering=true`): tetap beri koreksi menuju sisi seharusnya (`desired_side = ±gate_width_m/2`), ditandai `one_side_only=true`.
5. **Kasus dua tiang valid**: `lateral_error_m = (left_y+right_y)/2` (rata-rata dari tengah gerbang), `detected_width_m = right_y-left_y` divalidasi terhadap `[gate_width_m-1.0, gate_width_m+1.0]` (`width_valid`), `forward_distance_m` = rata-rata jarak forward kedua tiang. `centered = width_valid && |lateral_error_m| < centering_tolerance_m`.
6. Kalau titik ROI/puncak kurang dari `min_cluster_points` → `Result.valid=false`, tidak ada keputusan frame ini (caller menahan posisi/menunggu, tidak menebak).

### 8.3 Integrasi ke misi normal — `runMissionGateAssist()`

Berjalan di dalam `MISSION` fase `APPROACH`, satu gerbang per leg maju:

```text
APPROACH → gerbang terdeteksi (dua tiang valid, sample fresh ≤0.35s,
           sisa jarak leg masih cukup) → GATE CENTER → GATE ADVANCE
         → shift lateral sisa RelativePath → lanjut APPROACH normal
```

- **`GATE CENTER`**: position-mode, drone diam di forward tapi dikoreksi lateral menuju tengah gerbang dengan gain **adaptif** terhadap besar error (sama filosofinya dengan tuning YOLO §6.2 — tegas kalau masih jauh, redam kalau sudah dekat toleransi supaya tidak overshoot ke tiang seberang):

  | `\|lateral_error_m\|` | Gain (KP) | Step maksimum/tick |
  |---|---|---|
  | > 0.4 m | 0.35 | 0.12 m |
  | 0.15 – 0.4 m | 0.25 | 0.06 m |
  | < 0.15 m | 0.18 | 0.03 m |

  Perlu **10 tick berturut** (`gate_pass_required_centered_ticks_`) dengan `centered && width_valid && !one_side_only` sebelum lanjut ADVANCE. Timeout 15s → failsafe LAND.
- **`GATE ADVANCE`**: velocity-mode, maju lurus sepanjang heading leg (dibekukan, tidak ikut noise yaw) sejauh **sisa jarak leg yang sesungguhnya** (`along_track_remaining` di-capture tepat saat CENTER selesai — bukan angka tetap) sambil terus dikoreksi lateral dari Livox selagi fresh (`gate_pass_proportional_gain_`, maks `gate_pass_max_lateral_velocity_m_s_`). Begitu badan sudah melewati bidang gerbang (`traveled >= target_forward_distance_m`), kehilangan deteksi Livox jadi wajar (tiang keluar ROI belakang) — drone tetap lanjut lurus murni berbasis **heading terkunci + odometry posisi** (bukan berhenti/CENTER ulang). Kalau gerbang hilang **sebelum** bidang itu terlewati, drone berhenti dan CENTER ulang (tidak menembus buta). Timeout diskalakan otomatis dari jarak+kecepatan (`expected_travel_s*1.5+3s`, minimum `gate_pass_advance_timeout_s_`) supaya leg panjang tidak LAND prematur.
- Setelah gerbang selesai, sisa `RelativePath` digeser **hanya pada sumbu lateral** menuju center gerbang terakhir (`translateWaypointsFrom`) — panjang/bearing leg berikutnya tidak berubah, lalu `APPROACH` normal (ArUco/YOLO) lanjut seperti biasa.

### 8.4 Mode uji standalone — `start_mode:=gate_pass`

`runGatePassMission()` — bypass total ArUco/YOLO/gripper, langsung CENTER→ADVANCE satu gerbang lalu berhenti. Dipakai untuk validasi Livox sendirian sebelum digabung ke misi penuh (lihat `LAUNCH_TUTORIAL.md`). Parameter `gate_pass.*` di `config/gate_centering.yaml` (`pass_distance_m`, dst.) **hanya** dipakai jalur standalone ini — gate assist di misi normal (§8.3) memakai sisa jarak leg sebenarnya, bukan nilai ini.

## 9. Failsafe & Guard

- **Position stale**: warning kalau `/fmu/out/vehicle_local_position` tidak update >0.3s; **LAND** kalau stale >1.5s.
- **Lidar stale**: LAND kalau `/range` tidak valid/stale >0.5s saat misi berjalan (hanya kalau `use_lidar_altitude:=true`).
- **Altitude tidak masuk akal**: LAND kalau altitude relatif >8.0m.
- **Vision correction clamp**: `vision_max_correction_m` (default 0.4m) di `VisionLock`.
- **Marker hilang saat centering**: target terakhir ditahan, bukan langsung reset ke 0 atau mengejar noise.
- **Gate CENTER/ADVANCE timeout**: failsafe LAND (§8.3).
- **WAIT_ARM false-abort guard**: hanya abort kalau status PX4 memang tersedia dan eksplisit non-armed (§5).

## 10. Alur Gripper

Gripper dikontrol lewat publisher `std_msgs::msg::String` ke `gripper_cmd_topic` (default `/gripper_cmd`) — package `px4` **tidak** menggerakkan servo/GPIO langsung, node/hardware gripper fisik harus subscribe topic ini sendiri. Command yang dikirim hanya `"open"` dan `"close"`.

Drop default terjadi setelah **WP1** selesai (`current_wp_==0`, `gripper_drop_after_wp_=0`, masih hard-coded — untuk drop di WP lain perlu diubah jadi parameter di kode):

```text
IDLE -> publish "open" -> tunggu gripper_open_wait_ticks (default 15 tick ≈1.5s)
     -> publish "close" -> tunggu gripper_close_wait_ticks (default 15 tick ≈1.5s)
     -> COMPLETE -> lanjut waypoint berikutnya
```

Drone tetap hold posisi+altitude waypoint terakhir selama proses ini. Trigger drop: setelah box YOLO centered (`ground_lock_enable:=true`, default) atau setelah final-position-hold stabil 10 tick (`ground_lock_enable:=false`). `gripper_drop_enable:=false` mematikan seluruh alur ini (misi tetap lanjut, tanpa publish command apa pun).

## 11. Algoritma Navigasi (Tracking Antar-Waypoint)

### 11.1 Yaw sebelum maju

- Bearing leg dihitung dari **waypoint sebelumnya → waypoint sekarang** (bukan bearing sesaat posisi aktual→target) — supaya `turnLeft(91)` selalu berarti persis 91°, tidak berubah kalau drone sudah tergeser sedikit oleh centering.
- Drone menahan posisi (N/E/altitude beku) sambil yaw berputar ke bearing leg, pakai profil accel-limited: rate maksimum **0.35 rad/s** (≈20°/s), akselerasi **0.70 rad/s²** (≈40°/s²) — `YAW_MAX_RATE_RAD_S`/`YAW_ACCEL_RAD_S2` di `mission_manager.cpp`.
- Baru boleh maju setelah yaw aktual dalam **±5°** (`isYawAligned`, `YAW_LOCK_THRESHOLD=0.09 rad`) selama 5 tick.
- Setelah yaw besar (>0.35 rad ≈20°) selesai, program melakukan **post-yaw shift 10cm**: ~7cm pertama velocity-drive 0.20–0.30 m/s, sisa 3cm position-hold untuk mengerem, selesai setelah sisa ≤2cm & speed ≤0.10 m/s selama 3 tick (atau timeout aman 5 detik). Shift ini mengoreksi offset kecil akibat mekanika turn-in-place sebelum leg baru dimulai lurus.
- Setelah shift, waypoint sebelumnya + semua target berikutnya di-rebase ke posisi aktual (`translateWaypointsFrom`) — leg setelah yaw tetap lurus, tidak mengejar koordinat nominal lama secara diagonal.

### 11.2 Tracking forward + cross-track

Setelah yaw terkunci, kecepatan dipecah jadi komponen **along-track** (searah bearing leg) dan **cross-track** (tegak lurus, koreksi drift):

- **Forward speed** — `WaypointHandler::computeApproachSpeed(along_track_remaining)`: di dalam 2m, `max(0.3, dist*0.35)` m/s (zona rem, tidak berubah dari versi lama — mencegah overshoot). Di luar 2m, naik dengan gain **0.50** m/s per meter menuju cap **2.5 m/s**, bersambung mulus di batas 2m.
- **Cross-track correction** — sengaja **kecil dan terbatas** (bukan menghapus total error lateral langsung) supaya arah velocity tidak pernah terlihat "terbang miring": deadband 0.02m (leg pertama) / 0.03m (leg pasca-turn), gain proporsional 0.55, plus damping terhadap kecepatan lateral terukur (`measured_lateral_speed`, KD 0.70 leg-pertama / 0.80 pasca-turn) untuk meredam drift yang baru mulai tumbuh. Kecepatan lateral dibatasi maksimum **10–15% dari forward speed** (dan cap absolut 0.18–0.20 m/s) — arah velocity tidak bisa menyimpang lebih dari ~6° dari bearing leg murni.
- Seluruh vx/vy dikalikan `yawAlignmentFactor = max(0, cos(yaw_error))` — fine-tune kecil untuk redam noise yaw selama maju (bukan gate on/off, karena gate sudah ditangani terpisah di §11.1).
- Sisa along-track ≤0.45m → pindah ke **final position hold** langsung ke koordinat waypoint (posisi murni, bukan velocity).
- Setelah centering ArUco/YOLO atau final-hold tanpa vision selesai, posisi aktual dikomit sebagai anchor dan **seluruh sisa RelativePath digeser** dengan delta yang sama — bearing/panjang leg berikutnya tidak berubah walau posisi aktual meleset dari nominal.

### 11.3 Altitude & "reached"

- Toleransi altitude **0.15m** (`ALT_THRESHOLD`), vertical velocity maksimum **0.4 m/s** (`vz = clamp(alt_error*0.4, -0.4, 0.4)`).
- Waypoint dengan jarak horizontal <0.3m dianggap **pure-altitude** (`isPureAltitude`) — langsung climb/descend di tempat, skip seluruh fase yaw.
- Radius waypoint "reached" (memicu transisi ke `SEARCH_MARKER`/final-hold): **0.30m** (`WAYPOINT_RADIUS`) + altitude tercapai. Dengan vision aktif, "reached" **hanya** memicu pencarian marker — waypoint baru benar-benar selesai setelah marker centered (§6).

## 12. Altitude Lidar & Gazebo

```text
altitude_error   = target_altitude_agl - lidar_altitude_m
px4_down_target  = origin_down + current_px4_relative_down - altitude_error - altitude_command_bias_m
```

Kalau `/range` belum valid sebelum ARM/TAKEOFF, program menahan misi dan log `Menunggu data altitude TFmini /range yang valid sebelum ARM/TAKEOFF...`. Untuk Gazebo tanpa `/range`, pakai `use_lidar_altitude:=false start_tfmini_lidar:=false` — altitude jatuh ke `vehicle_local_position.z` (EKF PX4 murni).

---

## 13. Parameter Launch Lengkap

### Umum / Start Mode

| Parameter | Default | Fungsi |
| --- | --- | --- |
| `altitude_command_bias_m` | `0.0` | Bias tambahan target down PX4. |
| `use_lidar_altitude` | `true` | `true` = pakai `/range`, `false` = pakai z lokal PX4. |
| `start_tfmini_lidar` | `true` | Nyalakan node TF Mini dari launch PX4. |
| `start_mode` | `takeoff` | `takeoff` / `airborne_handoff` / `gate_pass`. |
| `start_mission_after_hover` | `false` | `false` = tes hover saja, `true` = lanjut misi. |
| `override_mission_heading` | `false` | `true` = waypoint diputar pakai `mission_heading_deg` absolut. |
| `mission_heading_deg` | `0.0` | Heading absolut misi saat override aktif. |
| `mission_heading_correction_deg` | `0.0` | Koreksi heading kecil dari yaw origin (kalau override mati). |

### Koreksi Heading Dua-Marker (§7)

| Parameter | Default | Fungsi |
| --- | --- | --- |
| `marker_heading_align_enable` | `true` | Aktifkan koreksi heading khusus `airborne_handoff`. |
| `marker_heading_back_id` | `0` | ID marker belakang/besar (posisi + referensi). |
| `marker_heading_front_id` | `1` | ID marker depan/kecil (referensi arah). |
| `marker_heading_tolerance_deg` | `5.0` | Error maksimum agar heading dianggap sudah pas (tidak dikoreksi). |
| `marker_heading_timeout_s` | `8.0` | Timeout warning fase align (posisi tetap ditahan, bukan abort). |
| `marker_heading_yaw_sign` | `1.0` | Balik ke `-1.0` kalau arah koreksi terbalik di lapangan. |

### Vision Lock (ArUco)

| Parameter | Default | Fungsi |
| --- | --- | --- |
| `vision_lock_enable` | `true` | Aktifkan pencarian + centering ArUco. |
| `camera_mount_yaw_deg` | `0.0` | Rotasi mounting kamera relatif hidung drone — **wajib kalibrasi lapangan**. |
| `vision_max_correction_m` | `0.4` | Batas magnitude koreksi vision lock. |
| `marker_center_tolerance_m` | `0.10` | Toleransi centering marker (ArUco). |
| `marker_search_timeout_s` | `20.0` | Waktu sebelum warning "marker belum ditemukan". |

### Ground Lock (YOLO, WP1)

| Parameter | Default | Fungsi |
| --- | --- | --- |
| `ground_lock_enable` | `true` | Kill-switch centering YOLO khusus WP1. |
| `yolo_camera_fx_px` | `640.0` | Fokal panjang kamera (piksel), sumbu X, proyeksi ground-plane. |
| `yolo_camera_fy_px` | `640.0` | Sama, sumbu Y. |

### Gripper

| Parameter | Default | Fungsi |
| --- | --- | --- |
| `gripper_drop_enable` | `true` | Aktifkan drop barang setelah WP1 centered. |
| `gripper_cmd_topic` | `/gripper_cmd` | Topic command gripper. |
| `gripper_open_wait_ticks` | `15` | Durasi tunggu setelah `open` (tick @10Hz). |
| `gripper_close_wait_ticks` | `15` | Durasi tunggu setelah `close` (tick @10Hz). |

### Gate Centering (Livox)

| Parameter | Default | Fungsi |
| --- | --- | --- |
| `gate_centering_enable` | `false` | Kill-switch keras subscriber + gate assist Livox. |
| `start_livox_lidar` | `false` | Jalankan driver Livox dari launch. |

### TF Mini I2C

| Parameter | Default | Fungsi |
| --- | --- | --- |
| `tfmini_bus_device` | `/dev/i2c-1` | Bus I2C. |
| `tfmini_i2c_address` | `16` | Address TF Mini (desimal, = 0x10). |
| `tfmini_i2c_read_mode` | `command` | Mode baca SMBus. |
| `tfmini_frame_id` | `tfmini_link` | Frame ID message Range. |
| `tfmini_publish_rate` | `20.0` | Frekuensi publish `/range`. |
| `tfmini_min_range` / `tfmini_max_range` | `0.03` / `12.0` | Rentang valid (meter). |
| `tfmini_field_of_view` | `0.04` | FoV message Range. |
| `tfmini_range_offset` | `0.0` | Offset kalibrasi jarak (meter). |
| `tfmini_log_rate` | `1.0` | Frekuensi log jarak. |
| `tfmini_mavlink_enabled` | `false` | Kirim DISTANCE_SENSOR MAVLink ke Pixhawk. |
| `tfmini_mavlink_device` | `/dev/ttyAMA0` | Serial MAVLink. |
| `tfmini_mavlink_system_id` / `_component_id` / `_sensor_id` | `1` / `191` / `0` | ID MAVLink. |
| `tfmini_mavlink_orientation` | `25` | Orientasi downward. |
| `tfmini_mavlink_covariance` | `0` | Covariance DISTANCE_SENSOR. |

### `config/gate_centering.yaml` (bukan launch arg — edit file langsung)

| Parameter | Default | Fungsi |
| --- | --- | --- |
| `gate_centering.detection_range_min` | `1.0` m | ROI forward minimum. |
| `gate_centering.detection_range_max` | `10.0` m | ROI forward maksimum. |
| `gate_centering.target_gate_distance` | `1.75` m | Jarak forward ideal saat centered. |
| `gate_centering.roi_lateral` | `3.0` m | ROI lateral (±). |
| `gate_centering.gate_width` | `1.5` m | Lebar gerbang yang dicari. |
| `gate_centering.centering_tolerance` | `0.2` m | Toleransi "centered". |
| `gate_centering.min_cluster_points` | `5` | Titik minimum per tiang. |
| `gate_pass.forward_velocity_m_s` | `1.0` m/s | Khusus `start_mode:=gate_pass`. |
| `gate_pass.proportional_gain` | `0.5` | Khusus `start_mode:=gate_pass`. |
| `gate_pass.max_lateral_velocity_m_s` | `0.3` m/s | Khusus `start_mode:=gate_pass`. |
| `gate_pass.pass_distance_m` | `3.5` m | Khusus `start_mode:=gate_pass` (misi normal pakai sisa jarak leg, §8.3). |
| `gate_pass.required_centered_ticks` | `10` | Khusus `start_mode:=gate_pass`. |
| `gate_pass.center_timeout_s` | `15.0` s | Khusus `start_mode:=gate_pass`. |
| `gate_pass.advance_timeout_s` | `12.0` s | Khusus `start_mode:=gate_pass` (misi normal skala otomatis, §8.3). |

Untuk contoh command launch lengkap (3 mode kanonik + mode uji), lihat `LAUNCH_TUTORIAL.md`.
