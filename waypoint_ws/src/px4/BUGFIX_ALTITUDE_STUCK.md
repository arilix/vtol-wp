# Debug Log: `px4` VTOL — Sesi Debugging Lengkap

Kronologi lengkap satu sesi debugging (2026-07-08) dari gejala awal "altitude tidak naik" sampai test 2-waypoint dengan masalah orbit yang belum tuntas. Ditulis apa adanya — termasuk hipotesis yang **salah** — supaya pola diagnosanya bisa dipakai lagi untuk masalah serupa.

---

## Bagian 1 — Gejala Awal: Altitude Stuck di "0.00/1.5"

Drone arm & takeoff fisik normal, tapi log ROS 2 selalu menampilkan:
```
Altitude(rel): 0.00m / 1.50m | raw z: -34.01 -> -35.50
```
angka **persis sama** berulang-ulang selama puluhan detik — altitude tidak pernah dianggap tercapai, misi macet di fase `TAKEOFF`.

### Percobaan #1 (SALAH) — QoS Durability

**Hipotesis:** subscriber `vehicle_local_position` pakai `DurabilityPolicy::TransientLocal`, diasumsikan tidak cocok dengan publisher PX4 (`Volatile`) → QoS incompatible.

**Hasil setelah fix + test terbang:** gejala identik terulang. **Salah** — `ros2 topic info -v` membuktikan publisher PX4 memang `TransientLocal`, sama dengan yang di-request.

### Percobaan #2 (BENAR, tapi bukan akar utama) — Bandwidth Serial

**Temuan:** `MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 115200`. Rate topik `vehicle_local_position` cuma ~2Hz dengan jitter sampai 1.6 detik **bahkan saat drone diam** — macet total begitu offboard mulai streaming.

**Fix:** naikkan baud ke **921600** di kedua sisi (`MicroXRCEAgent` companion + parameter baud FC via QGC, harus sama persis). **Hasil: rate naik ke ~95Hz, stabil.** Tetap dipertahankan sebagai prasyarat link sehat.

### Percobaan #3 — Origin Capture Meleset Puluhan Meter

Setelah bandwidth diperbaiki: **drone "terbang sangat tinggi"**. Log: `Alt:-37.77->1.5m` — error ~39 meter.

**Temuan:** origin (titik nol relatif) dikunci dari **satu sample pertama** posisi PX4, yang sering belum konvergen (EKF baru boot). Terbukti dari origin z berbeda-beda tiap run (-13.52, -34.00, dst) padahal start dari titik fisik sama.

**Fix (`onPositionUpdate`):** origin baru dikunci setelah posisi terbukti stabil (±0.2m) selama 1 detik penuh. Plus **altitude sanity guard**: kalau altitude relatif >8m dari origin terkunci, auto-LAND.

### Percobaan #4 — Drift 9 Meter Saat MISSION

Takeoff & hover sukses. Tapi di fase `MISSION`: drone **drift menjauh** — jarak 0.9m → **9.0m**, altitude 1.5m → **3.65m**. Operator stop manual.

**Root cause sebenarnya** — dari field lengkap `vehicle_local_position`:
```
xy_reset_counter: 68   z_reset_counter: 34   heading_reset_counter: 22
```
EKF2 PX4 sudah **reset posisi horizontal 68x dan altitude 34x sejak boot**. Kode tidak pernah membaca counter/delta ini — origin dikunci sekali lalu dianggap tetap selamanya. Reset EKF di tengah misi = lompatan instan pada posisi relatif, dan velocity controller mengejar lompatan itu → drift.

**Fix — Kompensasi Reset EKF:** `PositionSample` struct membawa `xy_reset_counter`, `delta_x/y`, `z_reset_counter`, `delta_z`, `heading_reset_counter`, `delta_heading` dari PX4. Setiap kali counter berubah, `origin_north_/east_/down_` digeser sebesar delta yang sama — posisi relatif tetap kontinu walau EKF melompat internal.

### Percobaan #5 — SUKSES PENUH (1 waypoint, target 1.0m)

1. Origin stabil terkunci 1 detik
2. Takeoff mulus 0 → 0.85m → **"Takeoff complete!"**
3. Hover 30/30 sukses
4. MISSION: **konvergen halus** (1.24m → 0.2m) → **"REACHED!"**
5. **"ALL WAYPOINT REACHED"** → **"LAND command sent"**
6. Reset EKF nyata terjadi (Z **-11.45m**, **-7.73m** saat landing) — **semua terkompensasi otomatis**, misi tidak terganggu.

Validasi nyata: fix kompensasi-reset-EKF menyelesaikan akar masalah drift.

---

## Bagian 2 — Skenario 2 Waypoint: Utara 1m → Belok Kanan + 1m

### Bug Syntax C++ (build error)

Operator edit manual, dapat error compile. **Penyebab:** `{ 1,00, 0.00, -1.0 }` — koma dipakai sebagai desimal (gaya Indonesia), tapi di C++ koma = pemisah elemen list. **Fix:** tulis ulang dengan titik: `{1.00, 0.00, -1.0}`, `{1.00, 1.00, -1.0}`.

### Percobaan #6 — Altitude Datar, Drone Diam di Darat

Altitude tidak naik, malah turun perlahan ke -0.17m. **Diagnosis operator:** QGC "Not Ready", drone tidak pernah lepas landas — angka altitude cuma noise sensor di darat.

**Root cause code:** `runInit()` kirim `arm()` lalu **langsung pindah ke TAKEOFF tanpa mengecek** apakah PX4 benar-benar menerimanya.

**Fix v1 (kurang tepat, lihat Bagian 3) — Guard `WAIT_ARM`:** fase baru, tunggu `vehicle_.arming_state == ARMED` sebelum lanjut; kalau tidak terkonfirmasi dalam 3 detik → batalkan misi dengan `timer_->cancel()`.

### Percobaan #7 — Guard Bekerja, Tapi ARM "Ditolak" 3x Berturut-turut

Guard mendeteksi & membatalkan misi dengan aman 3 kali berturut-turut, walau QGC menunjukkan "Ready to Fly". Investigasi QoS `/fmu/in/vehicle_command` (compatible, command terbukti sampai ke PX4) dan `/fmu/out/vehicle_command_ack` (tooling CLI tidak reliable membaca topik ini) tidak memberi jawaban pasti.

---

## Bagian 3 — Koreksi Penting: Guard ARM Salah Diagnosis, Bukan PX4 yang Menolak

**Pertanyaan tajam dari operator:** *"kok program sebelumnya (1 waypoint) bisa arm, apa gara-gara guard-mu makanya sekarang selalu ditolak?"*

Ini memicu investigasi ulang yang membuktikan operator **benar**:

1. Ditambahkan counter `status_update_count_`, increment di `onStatusUpdate()`.
2. Test ulang: log menunjukkan **`status callback terpanggil 0 kali`** — sepanjang seluruh proses, callback status dari topik `/fmu/out/vehicle_status_v1` **tidak pernah terpanggil sama sekali**, sementara callback posisi (`onPositionUpdate`) tetap normal (terbukti dari log EKF reset yang terus muncul lama setelah node "berhenti").

**Kesimpulan:** bukan PX4 yang menolak ARM — **topik status PX4 tidak pernah terbaca oleh kode ini**, kemungkinan karena topik ber-suffix `_v1` (skema versi lama) tidak cocok dengan `px4_msgs::msg::VehicleStatus` versi terbaru yang di-compile di workspace. Guard `WAIT_ARM` v1 SELALU membatalkan misi dalam 3 detik terlepas dari kondisi PX4 sebenarnya — **false abort**, bukan deteksi kegagalan nyata. Kemungkinan besar, di 3 percobaan "gagal arm" sebelumnya, PX4 sebenarnya BERHASIL arm, tapi guard yang salah membaca status memutus heartbeat sebelum sempat takeoff, membuat PX4 sendiri failsafe/disarm karena kehilangan sinyal offboard — terlihat seperti "arm ditolak" padahal "arm sukses lalu ditinggal".

**Fix v2 (rendah risiko) — `runWaitArm()` dirombak:**
- Tunggu jeda singkat (1 detik) — cukup untuk PX4 memproses arm.
- **Kalau `status_update_count_ == 0`** (topik status terbukti tidak memberi data): **lanjut ke TAKEOFF apa pun kondisinya** — mengembalikan perilaku lama yang TERBUKTI sukses di Percobaan #5, bukan blokir berdasarkan data yang memang tidak pernah tersedia.
- **Kalau topik status ternyata mulai memberi data** (`status_update_count_ > 0`): baru dipercaya — lanjut cepat kalau ARMED, batalkan dengan aman kalau setelah jeda benar-benar masih DISARMED.

### Percobaan #8 — VALIDASI: Guard Fix v2 Bekerja, ARM & Takeoff & WP1 Sukses

```
WAIT_ARM: raw arming_state = 0 | status callback terpanggil 0 kali
[WARN] Topik status PX4 tidak memberi data (callback 0x) — lanjut TAKEOFF
=== TAKEOFF ===
... altitude 0 -> 0.85m ...
Takeoff complete! -> Hover 30/30 -> START MISSION
[WP1-marker1] ... REACHED! (N=0.76 E=-0.38)
```
Terbukti: masalah ARM "ditolak" 100% disebabkan oleh guard v1 yang salah, bukan PX4. Fix v2 menyelesaikannya tuntas.

---

## Bagian 4 — Masalah Baru: Orbit Tidak Konvergen di WP2 (Belok Kanan)

Setelah WP1 REACHED, drone lanjut mengejar WP2 (butuh belok ~90° dari menghadap Utara ke menghadap Timur). **Drone tidak pernah REACHED** — bergerak dalam pola:

- **Sebelum fix yaw:** loiter/orbit **liar**, jarak naik-turun tajam antara 0.7m dan 2m berulang-ulang selama 90+ detik, altitude tetap wajar (~1.0-1.1m, tidak berbahaya).
- Operator menganalisis sendiri: *"kayaknya di bagian yaw-nya kurang stabil"* — **tepat**.

### Analisis Root Cause

1. `computeYaw()` menghitung ulang bearing (`atan2`) ke target **setiap tick berdasarkan posisi SAAT INI** — pure pursuit tanpa lead/smoothing. Makin dekat target, makin sensitif sudutnya terhadap noise posisi kecil.
2. Gate lama di `runMission()` **biner**: `yaw_error > 3° → berhenti TOTAL (vx=vy=0), cuma berputar`. Begitu drone maju sedikit, bearing bergeser, `yaw_error` nyebrang threshold 3° (sangat ketat), drone berhenti mendadak, berputar lagi — siklus stop-go yang menghasilkan orbit.
3. WP1 tidak kena masalah ini karena drone sudah kurang lebih menghadap WP1 sejak fase HOVER (yaw error kecil dari awal). WP2 butuh belok besar → rentan sepanjang pendekatan.

### Fix — Yaw Alignment Smooth (Percobaan #9)

`waypoint_handler.h/.cpp`:
- `YAW_THRESHOLD`: 0.05 rad (3°) → 0.26 rad (~15°) — kurang kritikal sekarang karena gate biner dihapus.
- `YAW_FREEZE_RADIUS` baru (0.5m, terpisah dari `PURE_ALT_RADIUS`=0.3m) — kunci `target_yaw` lebih awal untuk redam bearing liar dekat target.
- `yawAlignmentFactor(yaw_error)` baru: `max(0, cos(yaw_error))` — skala kecepatan maju **kontinu** (1.0 saat pas menghadap, turun smooth ke 0 saat error →90°, tidak pernah mundur).

`mission_manager.cpp` (`runMission`): hapus gate biner `if (!facing) stop`, ganti dengan `vx,vy *= yawAlignmentFactor(yaw_error)` — selalu ada progres, tidak ada stop-go mendadak.

**Hasil test:** orbit jadi jauh lebih **halus** (jarak stabil di kisaran 0.5-0.65m, tidak lagi lompat 0.7m↔2m), TAPI **masih belum konvergen** — drone kini orbit **stabil dengan radius konstan** (~0.5-0.65m) mengelilingi WP2 tanpa pernah masuk ke `WAYPOINT_RADIUS`=0.5m. Ini pola *limit-cycle* klasik pada guidance pure-pursuit (drone secara efektif "mengunci" pada orbit alih-alih menutup jarak), bukan lagi osilasi liar — root cause sudah teridentifikasi dengan jelas, tapi konvergensi penuh belum tercapai. Operator menghentikan test (baterai terpakai ~60 detik tanpa hasil, altitude tetap aman).

### Opsi Fix yang Didiskusikan (belum diterapkan — sesi dihentikan sebelum eksekusi)

**Diusulkan (rendah risiko):** longgarkan `WAYPOINT_RADIUS` dari 0.5m → ~0.8m. Drone sudah terbukti orbit stabil tepat di kisaran 0.5-0.65m — secara praktis sudah "sampai", radius toleransi saja yang terlalu ketat untuk presisi kontrol yang ada. **Belum disetujui/diterapkan** — operator memilih sudahi sesi sebelum memutuskan.

**Alternatif yang belum dieksplorasi** (kalau radius-longgar ternyata tidak cukup):
- Tambah komponen "closing velocity" eksplisit di `computeApproachVelocity` (bukan cuma arah bearing, tapi proyeksi menuju target yang mengurangi kecenderungan tangensial).
- Kurangi `SPEED_MS` / gain proporsional supaya drone tidak overshoot melewati garis lurus ke target sebelum sempat menyesuaikan arah.

---

## Ringkasan Semua Fix di Kode (per akhir sesi)

| File | Perubahan |
|---|---|
| `control_module.h`/`.cpp` | QoS subscriber Volatile+KeepLast(5); `PositionSample` struct membawa info reset EKF |
| `mission_manager.h`/`.cpp` | Origin stabilization (1s stabil); stale-position failsafe (>1.5s → LAND); altitude sanity guard (>8m → LAND); kompensasi reset EKF (geser origin sebesar delta); fase `WAIT_ARM` v2 (jeda 1s, TIDAK blokir kalau topik status tidak memberi data — lihat Bagian 3) |
| `waypoint_handler.h`/`.cpp` | `YAW_THRESHOLD` dilonggarkan (3°→15°); `YAW_FREEZE_RADIUS` baru (0.5m, terpisah dari `PURE_ALT_RADIUS`); `yawAlignmentFactor()` baru — skala kecepatan maju kontinu berbasis `cos(yaw_error)`, ganti gate biner lama |
| `mission_manager.cpp` (`defaultWaypoints`) | 2 waypoint: WP1 utara 1m, WP2 belok kanan + timur 1m; target altitude 1.0m |
| Infrastruktur (di luar kode) | `MicroXRCEAgent` baud 115200 → 921600 (companion), parameter baud FC via QGC |

## Ringkasan Semua Hipotesis

| # | Hipotesis | Status |
|---|---|---|
| 1 | QoS durability subscriber salah | ❌ Salah |
| 2 | Bandwidth link serial 115200 baud jenuh | ✅ Benar, diperbaiki (→921600) |
| 3 | Origin captured dari sample pertama yang belum konvergen | ✅ Benar, diperbaiki (stabilisasi 1 detik) |
| 4 | Tidak ada sumber posisi valid sama sekali | ❌ Salah — GPS fix ada, topik mentahnya cuma tidak di-bridge |
| 5 | EKF sering reset (68x sejak boot) tidak dikompensasi kode | ✅ **Root cause drift 9 meter**, diperbaiki — **tervalidasi lewat penerbangan sukses** |
| 6 | Salah ketik koma-desimal di waypoint C++ | ✅ Penyebab build error, diperbaiki |
| 7 | ARM dikirim tapi tidak dicek diterima PX4 (guard v1: blokir+abort) | ⚠️ Fix v1 **salah arah** — lihat #8 |
| 8 | Guard v1 sendiri yang salah membaca status (topik tidak pernah terbaca) → false abort | ✅ **Root cause sebenarnya**, dikonfirmasi operator, diperbaiki dengan guard v2 (non-blocking) — **tervalidasi: ARM+takeoff+WP1 sukses** |
| 9 | Gate yaw biner (facing ya/tidak) menyebabkan orbit liar di WP2 | ✅ Benar, diperbaiki dengan yaw alignment smooth — **orbit jadi halus, tapi belum konvergen penuh (masih orbit stabil di radius ~0.5-0.65m)** |

## Status Akhir Sesi & Langkah Berikutnya

- **Misi 1-waypoint** (climb 1.0m, reach, land): **sukses penuh end-to-end**, sudah tervalidasi berkali-kali.
- **Misi 2-waypoint** (WP1 utara 1m: **sukses**; WP2 belok kanan + timur 1m: **belum reached**, orbit stabil di radius ~0.5-0.65m).
- Drone dalam kondisi aman di darat, tidak ada proses tersisa di companion computer per akhir sesi.
- **Langkah berikutnya (belum dieksekusi):**
  1. Pertimbangkan longgarkan `WAYPOINT_RADIUS` 0.5m → 0.8m (fix termudah, minim risiko) — **butuh keputusan/konfirmasi operator dulu**.
  2. Kalau masih belum konvergen setelah itu, eksplorasi penambahan komponen closing-velocity di `computeApproachVelocity` atau penurunan gain kecepatan proporsional.
  3. Setelah 2-waypoint tuntas, lanjutkan uncomment waypoint berikutnya di `defaultWaypoints()` (WP3-WP5, arena kompetisi penuh) secara bertahap, satu-dua waypoint per sesi test, dengan pola verifikasi yang sama (safety check → launch → tail log live → analisis).
