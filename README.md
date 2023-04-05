# Leptrino Force Torque Sensor
 
 This code is refered and used for [Leptrino 6-DoF Force Torque Sensor](https://www.leptrino.co.jp/product/6axis-force-sensor).
 
Some features are refered to [hiveground](https://github.com/hiveground-ros-package/leptrino_force_torque).

A ROS 2 repository interfaces 6-DoF Leptrino Force Sensor. The offset of force sensor has been calculated to get actual force.

### 1. Download and build package
Download the source repository
```bash
cd /path/to/ros2-workspace/src
git clone https://github.com/labiybafakh/leptrino_force_torque_sensor
```
Compile the package
```bash
cd /path/to/ros2-workspace/
colcon build
```

### 2. How to lunch
```bash
source install/local_setup.bash
ros2 launch leptrino_force_torque leptrino.launch.py 
```