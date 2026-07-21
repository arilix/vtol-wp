# Vision Lock: Koreksi posisi via marker ArUco (kamera nadir)

Fitur ini memakai package `fiducial_detector` (kamera menghadap bawah/nadir)
untuk mengoreksi target posisi N/E saat drone sedang HOVER atau menahan
posisi di sekitar waypoint, supaya tidak bergantung 100% pada estimasi NED
yang bisa drift/jitter (lihat `BUGFIX_ALTITUDE_STUCK.md` untuk riwayat
lengkap masalah NED). Marker ArUco diletakkan di tanah pada setiap titik
yang mau presisi — kamera mendeteksi marker itu, dan posisi drone dikunci
relatif ke marker fisik, bukan ke koordinat NED mentah.

## Pemetaan istilah operator vs kode

Operator biasa menyebut "WP1", "WP2", "WP3" untuk titik pendaratan pertama,
kedua, ketiga di lapangan. Di kode, penomoran itu **tidak sama persis**:

| Istilah operator | Di kode |
|---|---|
| WP1 (titik hover setelah takeoff) | `vehicle_.hover_position` di `runHover()` — posisi tepat di atas titik lepas landas, karena takeoff murni naik vertikal |
| WP2 (tujuan horizontal pertama) | `waypoints_->at(0)` — dwell/REACHED branch di `runMission()` |
| WP3 | `waypoints_->at(1)` |
| WP4, dst | `waypoints_->at(2)`, dst |

Vision lock aktif di **dua jenis fase "tahan posisi tetap"**: `runHover()`
(WP1 operator) dan dwell setelah waypoint REACHED di `runMission()` (WP2,
WP3, dst). Vision lock **tidak** menyentuh fase mendekati waypoint (yaw-align
+ approach velocity) — itu logic yang sudah stabil dan di luar cakupan fitur
ini.

## Cara kerja singkat

1. `aruco_node` (package `fiducial_detector`) mempublish `/fiducial/pose`
   (posisi marker relatif kamera, meter) setiap frame marker terdeteksi.
2. `mission_manager` (package `px4`) subscribe ke topic itu. Setiap sample
   diubah jadi offset NED (lewat `camera_mount_yaw_deg` + yaw drone
   sekarang), lalu dijumlahkan ke posisi drone saat itu -> target terkunci.
   Ini dihitung ulang tiap sample segar datang (bukan sekali lalu diam),
   jadi drift NED yang terjadi setelah lock pertama kali tetap terkoreksi.
3. Kalau marker hilang dari kamera, target terakhir **dibekukan** (tidak
   kembali ke NED mentah, tidak terus bergerak) sampai marker terlihat lagi.
4. Koreksi **hanya horizontal** (N/E). Altitude tetap sepenuhnya dikendalikan
   jalur lidar yang sudah ada — `tvec.z` dari kamera sengaja tidak dipakai.

## WAJIB: kalibrasi `camera_mount_yaw_deg`

Kamera nadir bisa dipasang dengan rotasi berapa pun relatif hidung drone.
Parameter `camera_mount_yaw_deg` (default `0.0`, berarti asumsi "atas
gambar kamera = arah hidung drone") **harus dikalibrasi di lapangan**
sebelum dipercaya untuk terbang, karena kalau arahnya salah, drone bisa
dikoreksi MENJAUHI marker alih-alih mendekat.

Prosedur kalibrasi:

1. Hover manual (RC) tepat di atas satu marker, cukup tinggi supaya marker
   penuh terlihat kamera. Jalankan `aruco_node` dan `ros2 topic echo
   /fiducial/pose` — catat tanda `x`/`y` saat drone digeser sedikit ke arah
   yang diketahui (misal digeser ke depan drone, lihat `y` naik/turun).
2. Jalankan `mission_manager` dengan `vision_lock_enable:=true` dan
   `camera_mount_yaw_deg:=0.0` dulu. Amati log `VISION LOCK ENGAGED` dan
   arah koreksi yang dikirim — apakah menuju marker atau menjauh.
3. Sesuaikan `camera_mount_yaw_deg` (kelipatan 90° dulu kalau mounting
   berbentuk kotak/lurus, lalu fine-tune) sampai koreksi konsisten menuju
   marker.
4. **Ulangi test di heading drone yang berbeda (minimal 2 heading, ~90°
   berbeda).** Kalibrasi di satu heading saja bisa menutupi bug
   mirror/tertukar-sumbu yang tidak bisa diperbaiki oleh satu konstanta
   rotasi — bug seperti itu baru kelihatan kalau diuji di heading lain.

Jangan terbang dengan `vision_lock_enable:=true` sebelum kalibrasi ini
selesai dan konsisten di ≥2 heading.

## Menjalankan bersama

Jalankan salah satu launch file kamera dari `fiducial_detector` (pilih
sesuai hardware — lihat `launch.md` di package tersebut untuk detail
argumen), lalu `mission_manager` seperti biasa:

```bash
# Terminal 1 — kamera + deteksi ArUco (contoh webcam)
ros2 launch fiducial_detector webcam.launch.xml device_id:=0

# Terminal 2 — cek marker terdeteksi sebelum arm
ros2 topic echo /fiducial/pose

# Terminal 3 — mission manager
ros2 launch px4 px4.launch.xml \
  vision_lock_enable:=true \
  camera_mount_yaw_deg:=<hasil kalibrasi> \
  vision_max_correction_m:=0.4
```

## Parameter

| Parameter | Default | Keterangan |
|---|---|---|
| `vision_lock_enable` | `true` | Kill-switch lapangan — `false` = perilaku identik sebelum fitur ini ada (murni NED). |
| `camera_mount_yaw_deg` | `0.0` | Rotasi mounting kamera relatif hidung drone. **Wajib kalibrasi**, lihat di atas. |
| `vision_max_correction_m` | `0.4` | Batas magnitude koreksi (meter). Sengaja ketat — masalah yang dikoreksi berskala cm; koreksi yang mendekati batas ini adalah tanda ada yang salah (kalibrasi/deteksi), bukan koreksi besar yang valid. |

## Kalau ada yang aneh di lapangan

- Drone bergerak menjauhi marker saat lock engage -> `camera_mount_yaw_deg`
  salah (kemungkinan perlu +90/180/270°) atau ada bug mirror — jangan
  terbang, ulangi kalibrasi.
- Log `Vision lock: koreksi terpotong ke batas ...` sering muncul -> marker
  terlalu jauh dari toleransi normal, cek kalibrasi kamera
  (`fiducial_detector/config/detector.yaml`, `marker_size`) atau
  `camera_mount_yaw_deg`.
- Ingin nonaktifkan cepat tanpa rebuild -> `vision_lock_enable:=false` di
  launch, atau matikan saja `aruco_node` (tanpa data `/fiducial/pose`,
  vision lock otomatis tidak pernah engaged, perilaku sama seperti sebelum
  fitur ini).
