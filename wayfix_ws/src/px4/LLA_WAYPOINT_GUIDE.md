# NED Waypoint Guide

Ada dua cara mengisi waypoint di `defaultMissionWaypoints()` (`src/core/mission_manager.cpp`).

## Cara 1 (disarankan): RelativePath — tidak perlu tahu kompas

Cukup deskripsikan gerakan relatif terhadap arah drone sendiri, seperti kasih instruksi ke orang jalan:

```cpp
RelativePath path(/*start_altitude_agl_m=*/1.0);
path.forward(3.0)      // maju 3m
    .turnRight(90.0)   // belok kanan 90 derajat (tidak gerak, cuma ganti arah "depan")
    .forward(5.0)       // maju 5m ke arah baru
    .climbTo(2.0);       // naik ke 2m AGL di posisi sekarang
return path.build();
```

Method yang tersedia:

- `forward(dist_m)` — maju searah "depan" saat ini (mundur: nilai negatif). Nambah 1 waypoint.
- `strafeRight(dist_m)` / `strafeLeft(dist_m)` — geser menyamping tegak lurus arah "depan" TANPA mengubah arah "depan". Nambah 1 waypoint.
- `turnRight(deg)` / `turnLeft(deg)` — ubah arah "depan" untuk gerakan berikutnya. TIDAK nambah waypoint (bukan perintah drone berputar di tempat, murni matematika arah).
- `climbTo(altitude_agl_m)` — naik/turun altitude di posisi N/E sekarang. Nambah 1 waypoint.

"Depan" (heading 0) di sini adalah arah drone menghadap saat mulai hover, BUKAN utara kompas — jadi kamu tidak perlu tahu orientasi kompas arena sama sekali, cukup tahu drone mau maju berapa meter dan belok ke mana.

Semua method bisa dirangkai (chaining) dan panggil `.build()` di akhir untuk dapat list waypoint NED.

## Cara 2: NED manual (kalau sudah tahu offset N/E)

```cpp
return {
    { 3.0, 0.0, -1.0 },
    { 3.0, 5.0, -1.0 },
};
```

Formatnya:

```text
{ north_m, east_m, down_m }
```

- `north_m` / `east_m`: offset dalam meter dari posisi hover (posisi drone saat selesai takeoff dan stabil). North+ = maju, East+ = kanan/timur.
- `down_m`: target altitude, negatif = naik. `down_m = -1.0` artinya target tinggi 1m AGL. Jika `use_lidar_altitude:=true`, nilai ini dibandingkan dengan `/range` TFmini. Jika `false`, dibandingkan dengan altitude lokal PX4.

Waypoint pertama juga dipakai sebagai target altitude TAKEOFF (drone naik vertikal di tempat sampai `down_m` waypoint pertama tercapai, baru dianggap "hover").

## Membaca Log

Saat start, node menampilkan daftar waypoint:

```text
WP1 N=...m E=...m Alt=...m
Leg WP1->WP2: dN=...m dE=...m Dist=...m Bearing=...deg
```

Maknanya:

- `N`: North relatif posisi hover.
- `E`: East relatif posisi hover.
- `Alt`: target altitude AGL.
- `dN/dE`: selisih meter dari waypoint sebelumnya ke waypoint sekarang.
- `Dist`: jarak horizontal antar waypoint.
- `Bearing`: arah dari waypoint sebelumnya ke waypoint sekarang dalam derajat, 0 derajat = utara, 90 derajat = timur.

Saat misi berjalan:

```text
Pos N=... E=... -> Target N=... E=... | Dist=...
```

`Pos N/E` adalah posisi drone sekarang (relatif hover). `Target N/E` adalah waypoint aktif. `Dist` adalah jarak drone ke waypoint aktif dalam meter.

## Tanda Offset

```text
North +  = maju ke utara
North -  = mundur ke selatan
East  +  = kanan ke timur
East  -  = kiri ke barat
```

Jika drone/arena dianggap menghadap utara:

```text
kanan 1m -> north=0, east=+1
kiri 5m  -> north=0, east=-5
maju 3m  -> north=+3, east=0
mundur 2m -> north=-2, east=0
```

Lalu hasilnya masukkan ke `defaultMissionWaypoints()`.

## Belok Kanan/Kiri Relatif Arah Terbang

Kalau "kanan/kiri" bukan relatif utara, tapi relatif arah dari waypoint sebelumnya ke waypoint berikutnya, gunakan heading/bearing.

Jika arah maju punya yaw/bearing `psi`:

```text
forward_n = cos(psi)
forward_e = sin(psi)

right_n = -sin(psi)
right_e =  cos(psi)

left_n  =  sin(psi)
left_e  = -cos(psi)
```

Contoh: dari titik sekarang ingin belok kanan 1m relatif arah terbang:

```text
north_offset = right_n * 1.0
east_offset  = right_e * 1.0
```

Tambahkan `north_offset/east_offset` ke N/E waypoint sebelumnya untuk dapat waypoint berikutnya.

## Saran Test

Untuk melihat yaw dan belokan jelas, jangan pakai waypoint terlalu dekat. Dengan `WAYPOINT_RADIUS = 0.8m`, jarak antar waypoint 1m bisa cepat dianggap sampai.

Gunakan jarak awal sekitar:

```text
3m sampai 5m
```

baru kecilkan radius atau jarak setelah kontrol sudah terlihat stabil.
