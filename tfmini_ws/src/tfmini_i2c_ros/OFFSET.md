# Panduan Offset TF Mini I2C

Offset dipakai untuk mengoreksi hasil jarak lidar jika pembacaan sensor berbeda dari jarak asli.

Contoh:

- Jarak asli ke objek: `1.00 m`
- Data lidar terbaca: `0.95 m`
- Offset yang dibutuhkan: `+0.05 m`

Rumus:

```text
range_final = range_raw + range_offset
```

## Lokasi Kode

File utama untuk mengatur data yang dipublish:

```text
src/tfmini_i2c_ros/src/source/tfmini_i2c_node.cpp
```

Baris penting saat ini:

```cpp
msg.range = static_cast<float>(measurement.distance_cm) / 100.0F;
```

Nilai dari TF Mini masih dalam centimeter, lalu dibagi `100.0` agar menjadi meter.

## Cara Cepat: Hardcode Offset

Jika ingin menaikkan data jarak `5 cm`, ubah menjadi:

```cpp
const float range_offset = 0.05F;
msg.range = (static_cast<float>(measurement.distance_cm) / 100.0F) + range_offset;
```

Jika ingin menurunkan data jarak `5 cm`, pakai offset negatif:

```cpp
const float range_offset = -0.05F;
msg.range = (static_cast<float>(measurement.distance_cm) / 100.0F) + range_offset;
```

Satuan offset adalah meter.

## Cara Rapi: Offset Dari Launch XML

Cara ini lebih disarankan karena offset bisa diganti tanpa edit dan rebuild kode.

### 1. Tambahkan Variable Di Header

Edit file:

```text
src/tfmini_i2c_ros/utils/tfmini_i2c_node.h
```

Tambahkan member ini di bagian `private`:

```cpp
double range_offset_{0.0};
```

Contoh posisinya:

```cpp
double min_range_{0.03};
double max_range_{12.0};
double field_of_view_{0.04};
double range_offset_{0.0};
```

### 2. Tambahkan Parameter Di Node

Edit file:

```text
src/tfmini_i2c_ros/src/source/tfmini_i2c_node.cpp
```

Di constructor `TfminiI2cNode::TfminiI2cNode()`, tambahkan:

```cpp
range_offset_ = declare_parameter<double>("range_offset", 0.0);
```

Contoh posisinya:

```cpp
min_range_ = declare_parameter<double>("min_range", 0.03);
max_range_ = declare_parameter<double>("max_range", 12.0);
field_of_view_ = declare_parameter<double>("field_of_view", 0.04);
range_offset_ = declare_parameter<double>("range_offset", 0.0);
```

### 3. Pakai Offset Saat Publish

Masih di file:

```text
src/tfmini_i2c_ros/src/source/tfmini_i2c_node.cpp
```

Ubah:

```cpp
msg.range = static_cast<float>(measurement.distance_cm) / 100.0F;
```

Menjadi:

```cpp
msg.range = (static_cast<float>(measurement.distance_cm) / 100.0F) +
  static_cast<float>(range_offset_);
```

### 4. Tambahkan Parameter Di Launch XML

Edit file:

```text
src/tfmini_i2c_ros/launch/tfmini_i2c.launch.xml
```

Tambahkan arg:

```xml
<arg name="range_offset" default="0.0"/>
```

Lalu tambahkan param di dalam tag `<node>`:

```xml
<param name="range_offset" value="$(var range_offset)"/>
```

## Build Ulang

Setelah mengubah kode:

```bash
cd ~/tfmini_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run Dengan Offset

Offset naik `5 cm`:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml range_offset:=0.05
```

Offset turun `5 cm`:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml range_offset:=-0.05
```

Tanpa offset:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml range_offset:=0.0
```

## Cara Menghitung Offset

1. Letakkan lidar menghadap objek datar.
2. Ukur jarak asli pakai meteran.
3. Baca data lidar:

```bash
ros2 topic echo /range
```

4. Hitung offset:

```text
range_offset = jarak_asli - jarak_lidar
```

Contoh:

```text
jarak_asli  = 1.00 m
jarak_lidar = 0.96 m
range_offset = 1.00 - 0.96 = 0.04 m
```

Maka run:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml range_offset:=0.04
```
