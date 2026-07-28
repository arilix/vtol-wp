# LLA Waypoint Guide

Program waypoint sekarang memakai koordinat LLA asli:

```cpp
return {
    { -7.310749, 112.728550, 1.0 },
    { -7.310751, 112.728560, 1.0 },
};
```

Formatnya:

```text
{ latitude_deg, longitude_deg, altitude_agl_m }
```

`altitude_agl_m` adalah target tinggi relatif terhadap permukaan bawah drone dalam meter. Jika `use_lidar_altitude:=true`, nilai ini dibandingkan dengan `/range` TFmini. Jika `false`, nilai ini dibandingkan dengan altitude lokal PX4.

## Membaca Log

Saat start, node menampilkan hasil konversi:

```text
WP1 LLA=(lat, lon, AGL alt) -> N=...m E=...m Alt=...m
Leg WP1->WP2: dN=...m dE=...m Dist=...m Bearing=...deg
```

Maknanya:

- `N`: North lokal PX4 dalam meter dari GPS origin.
- `E`: East lokal PX4 dalam meter dari GPS origin.
- `Alt`: target altitude AGL.
- `dN/dE`: selisih meter dari waypoint sebelumnya ke waypoint sekarang.
- `Dist`: jarak horizontal antar waypoint.
- `Bearing`: arah dari waypoint sebelumnya ke waypoint sekarang dalam derajat, 0 derajat = utara, 90 derajat = timur.

Saat misi berjalan:

```text
Pos N=... E=... -> Target N=... E=... | Dist=...
```

`Pos N/E` adalah posisi drone sekarang. `Target N/E` adalah waypoint aktif. `Dist` adalah jarak drone ke waypoint aktif dalam meter.

## Konversi Offset Meter Ke LLA

Untuk arena kecil, pakai rumus pendek ini:

```text
lat_baru = lat_awal + (north_m / 6378137.0) * 180/pi
lon_baru = lon_awal + (east_m / (6378137.0 * cos(lat_awal_rad))) * 180/pi
```

Tanda offset:

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

Contoh Python cepat:

```python
import math

def offset_to_lla(lat_deg, lon_deg, north_m, east_m):
    r = 6378137.0
    lat = lat_deg + math.degrees(north_m / r)
    lon = lon_deg + math.degrees(east_m / (r * math.cos(math.radians(lat_deg))))
    return lat, lon

lat0 = -7.310749
lon0 = 112.728550

print("kanan 1m:", offset_to_lla(lat0, lon0, 0.0, 1.0))
print("kiri 5m :", offset_to_lla(lat0, lon0, 0.0, -5.0))
print("maju 3m :", offset_to_lla(lat0, lon0, 3.0, 0.0))
```

Lalu hasilnya masukkan ke `defaultGlobalWaypoints()`.

## Belok Kanan/Kiri Relatif Arah Terbang

Kalau “kanan/kiri” bukan relatif utara, tapi relatif arah dari waypoint sebelumnya ke waypoint berikutnya, gunakan heading/bearing.

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

Setelah itu masukkan `north_offset/east_offset` ke rumus `offset_to_lla()`.

## Saran Test

Untuk melihat yaw dan belokan jelas, jangan pakai waypoint terlalu dekat. Dengan `WAYPOINT_RADIUS = 0.8m`, jarak antar waypoint 1m bisa cepat dianggap sampai.

Gunakan jarak awal sekitar:

```text
3m sampai 5m
```

baru kecilkan radius atau jarak setelah kontrol sudah terlihat stabil.
