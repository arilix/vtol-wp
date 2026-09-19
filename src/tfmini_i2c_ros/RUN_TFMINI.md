# Menjalankan TF Mini I2C ROS 2

Dokumen ini berisi langkah run TF Mini I2C di Raspberry Pi 5 dengan ROS 2 Jazzy.

## Kondisi Yang Sudah Terbukti

Sensor sudah terdeteksi di I2C bawaan Raspberry Pi:

```bash
i2cdetect -y 1
```

Target hasil:

```text
10: 10 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
```

Artinya:

- Bus: `/dev/i2c-1`
- Address: `0x10`
- Address desimal untuk launch ROS: `16`

## Protokol Yang Dipakai

Driver C++ dibuat mengikuti script Python yang sudah berhasil:

```python
bus.write_i2c_block_data(0x10, 0x5A, [0x05, 0x07, 0x01, 0x67])
bus.write_i2c_block_data(0x10, 0x5A, [0x05, 0x00, 0x01, 0x60])
data = bus.read_i2c_block_data(0x10, 0x00, 9)
```

Di launch ROS, mode ini disebut:

```text
i2c_read_mode:=command
```

Mode `command` sudah menjadi default.

## Build

```bash
cd ~/tfmini_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Tanpa offset:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml
```

Dengan offset naik `5 cm`:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml range_offset:=0.05
```

Command lengkap:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml bus_device:=/dev/i2c-1 i2c_address:=16 i2c_read_mode:=command range_offset:=0.05
```

Terminal launch akan menampilkan log jarak setiap `1 detik`.

Contoh:

```text
TF Mini range: raw=0.17 m offset=0.05 m final=0.22 m strength=5048
```

Jika ingin mengubah frekuensi log:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml range_offset:=0.05 log_rate:=2.0
```

Jika ingin mematikan log periodik:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml range_offset:=0.05 log_rate:=0.0
```

## Cek Data Jarak

Publisher `/range` memakai QoS sensor data, jadi echo pakai `best_effort`:

```bash
ros2 topic echo /range --qos-reliability best_effort
```

## Kirim Ketinggian Ke Pixhawk

Node ini bisa mengirim hasil TF Mini ke Pixhawk sebagai MAVLink `DISTANCE_SENSOR`.

Default serial:

```text
/dev/ttyAMA0
```

Baudrate MAVLink dibuat fixed:

```text
921600
```

Run:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml range_offset:=0.05 mavlink_enabled:=true
```

Jika memakai device lain:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml range_offset:=0.05 mavlink_enabled:=true mavlink_device:=/dev/ttyAMA0
```

Data yang dikirim ke Pixhawk adalah jarak final setelah offset:

```text
distance_to_pixhawk = raw_distance + range_offset
```

Message MAVLink:

```text
DISTANCE_SENSOR
```

Default orientasi sensor:

```text
mavlink_orientation:=25
```

Nilai `25` adalah orientasi downward untuk rangefinder yang mengukur ketinggian ke bawah.

Contoh hasil valid:

```yaml
header:
  frame_id: tfmini_link
radiation_type: 1
field_of_view: 0.03999999910593033
min_range: 0.029999999329447746
max_range: 12.0
range: 0.2199999988079071
```

Jika sensor membaca `17 cm` dan offset `0.05 m`, hasil ROS menjadi:

```text
0.17 m + 0.05 m = 0.22 m
```

## Cek Topic Dan Node

```bash
ros2 topic list
ros2 node list
ros2 node info /tfmini_i2c_node
```

Topic yang diharapkan:

```text
/range
/parameter_events
/rosout
```

## Parameter Launch

| Parameter | Default | Keterangan |
| --- | --- | --- |
| `bus_device` | `/dev/i2c-1` | Bus I2C Raspberry Pi pin 3/5 |
| `i2c_address` | `16` | Address `0x10` dalam desimal |
| `i2c_read_mode` | `command` | Mode SMBus yang cocok dengan script Python |
| `frame_id` | `tfmini_link` | Frame ROS untuk message Range |
| `publish_rate` | `20.0` | Frekuensi publish Hz |
| `range_offset` | `0.0` | Koreksi jarak dalam meter |
| `log_rate` | `1.0` | Frekuensi log jarak di terminal, Hz |
| `mavlink_enabled` | `false` | Aktifkan kirim data ke Pixhawk |
| `mavlink_device` | `/dev/ttyAMA0` | Port serial ke TELEM2 Pixhawk |
| `mavlink_system_id` | `1` | MAVLink system id |
| `mavlink_component_id` | `191` | MAVLink component id companion |
| `mavlink_sensor_id` | `0` | ID sensor rangefinder |
| `mavlink_orientation` | `25` | Orientasi sensor, default downward |
| `mavlink_covariance` | `0` | Covariance untuk message DISTANCE_SENSOR |
| `min_range` | `0.03` | Range minimum message |
| `max_range` | `12.0` | Range maksimum message |

## Troubleshooting

### `/range` Tidak Keluar

Cek sensor muncul di I2C:

```bash
i2cdetect -y 1
```

Harus muncul `10`. Jika kosong, cek wiring, power, dan mode sensor.

### Topic Ada Tapi `ros2 topic echo` Kosong

Gunakan QoS `best_effort`:

```bash
ros2 topic echo /range --qos-reliability best_effort
```

### Permission Denied Ke `/dev/i2c-1`

Tambahkan user ke group `i2c`, lalu reboot:

```bash
sudo usermod -aG i2c $USER
sudo reboot
```

### Kadang Sensor Hilang Dari `i2cdetect`

Cek power Raspberry Pi:

```bash
vcgencmd get_throttled
```

Target ideal:

```text
throttled=0x0
```

Jika pernah muncul undervoltage, pakai power supply Raspberry Pi 5 yang kuat dan kabel pendek/bagus.

### Jangan Pakai `/dev/i2c-13` Atau `/dev/i2c-14`

Untuk pin I2C bawaan Raspberry Pi:

```text
SDA = pin 3 / GPIO2
SCL = pin 5 / GPIO3
Bus = /dev/i2c-1
```

Jika `i2cdetect` di bus lain menampilkan hampir semua address, itu biasanya bus floating atau bukan bus sensor.

## UART AMA0 Untuk Pixhawk

UART `ttyAMA0` boleh tetap aktif untuk Pixhawk/TELEM2.

Yang harus dimatikan hanya Linux serial console/getty, supaya Raspberry Pi tidak mengirim boot log atau login prompt ke Pixhawk:

```text
console=serial0,115200  -> jangan ada di /boot/firmware/cmdline.txt
serial-getty@ttyAMA0    -> inactive
```

Cek:

```bash
systemctl is-active serial-getty@ttyAMA0.service
ls -l /dev/ttyAMA0
```

Kondisi yang benar:

```text
/dev/ttyAMA0 ada
serial-getty@ttyAMA0 inactive
```

Jika launch gagal membuka `/dev/ttyAMA0`, cek permission:

```bash
ls -l /dev/ttyAMA0
groups
```

Di Raspberry Pi ini `/dev/ttyAMA0` bisa muncul seperti:

```text
crw--w---- 1 root tty ... /dev/ttyAMA0
```

Jika begitu, user perlu masuk group `tty`, lalu reboot:

```bash
sudo usermod -aG tty $USER
sudo reboot
```

Untuk test cepat bisa run dengan `sudo`, tapi untuk operasional lebih baik atur permission serial dengan benar.
