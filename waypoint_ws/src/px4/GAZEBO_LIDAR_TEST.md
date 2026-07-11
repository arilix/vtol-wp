# Gazebo and TFmini Altitude Test

Panduan singkat untuk memilih sumber altitude pada program waypoint PX4.

## Mode Altitude

Program waypoint punya parameter:

```bash
use_lidar_altitude:=true
```

Jika `true`, altitude misi memakai topic TFmini:

```text
/range
sensor_msgs/msg/Range
```

Jika `false`, altitude misi memakai `vehicle_local_position.z` dari PX4. Mode ini cocok untuk uji Gazebo jika simulator belum menyediakan topic `/range`.

## Uji Di Gazebo Tanpa Lidar

Gunakan mode fallback PX4 local position:

```bash
cd ~/waypoint_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml use_lidar_altitude:=false
```

Di log node harus muncul:

```text
Use lidar altitude: false
```

## Cek Apakah Gazebo Punya Topic Range

Saat Gazebo dan bridge ROS 2 sudah berjalan:

```bash
ros2 topic list | grep range
```

Jika ada `/range`, cek tipe message:

```bash
ros2 topic info /range
```

Program waypoint butuh:

```text
Type: sensor_msgs/msg/Range
```

Cek data range:

```bash
ros2 topic echo /range --qos-reliability best_effort
```

Nilai `range` harus update dan masuk akal dalam satuan meter.

## Uji Dengan Lidar

Jika `/range` valid, jalankan:

```bash
cd ~/waypoint_ws
source install/setup.bash
ros2 launch px4 px4.launch.xml use_lidar_altitude:=true
```

Di log node harus muncul:

```text
Use lidar altitude: true
Altitude lidar: ...
```

Jika `/range` belum valid, node akan menahan misi sebelum arm/takeoff dan menampilkan:

```text
Menunggu data altitude TFmini /range yang valid sebelum ARM/TAKEOFF...
```

Jika `/range` stale saat misi berjalan, node akan mengirim LAND failsafe.

## Uji TFmini Asli

Jalankan node TFmini dari workspace lidar:

```bash
cd ~/tfmini_ws
source install/setup.bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml
```

Cek output:

```bash
ros2 topic echo /range --qos-reliability best_effort
```

Setelah data stabil, jalankan waypoint dengan `use_lidar_altitude:=true`.
