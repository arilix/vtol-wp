# tfmini_i2c_ros

ROS 2 Jazzy C++ package untuk membaca Benewake TF Mini/TF Mini Plus lewat I2C di Raspberry Pi 5.

Struktur paket:

```text
tfmini_i2c_ros/
├── launch/
│   └── tfmini_i2c.launch.xml
├── src/
│   ├── main/
│   │   └── main.cpp
│   └── source/
│       ├── tfmini_i2c.cpp
│       └── tfmini_i2c_node.cpp
└── utils/
    ├── tfmini_i2c.h
    └── tfmini_i2c_node.h
```

Default:

- I2C bus: `/dev/i2c-1`
- I2C address: `0x10` atau desimal `16`
- Topic: `/range`
- Message: `sensor_msgs/msg/Range`

## Build

```bash
cd ~/tfmini_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml
```

Dengan parameter:

```bash
ros2 launch tfmini_i2c_ros tfmini_i2c.launch.xml bus_device:=/dev/i2c-1 i2c_address:=16 frame_id:=tfmini_link publish_rate:=20.0
```

Driver default memakai mode `command`, sama seperti script Python SMBus:

```text
enable output command -> request data command -> read block dari register 0x00
```

## Cek Data

```bash
ros2 topic echo /range --qos-reliability best_effort
```

Panduan run dan troubleshooting lengkap ada di:

```text
src/tfmini_i2c_ros/RUN_TFMINI.md
```

## Offset

Panduan menaikkan atau menurunkan data jarak ada di:

```text
src/tfmini_i2c_ros/OFFSET.md
```

Jika permission `/dev/i2c-1` ditolak, tambahkan user ke group `i2c`, lalu login ulang:

```bash
sudo usermod -aG i2c $USER
```

Pastikan address lidar sesuai:

```bash
i2cdetect -y 1
```
