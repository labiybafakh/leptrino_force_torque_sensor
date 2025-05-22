# Leptrino Force Torque Sensor

This code is referred and used for [Leptrino 6-DoF Force Torque Sensor](https://www.leptrino.co.jp/product/6axis-force-sensor).

Some features are referred to [hiveground](https://github.com/hiveground-ros-package/leptrino_force_torque).

A ROS 2 repository that interfaces with 6-DoF Leptrino Force Sensor. The offset of force sensor has been calculated to get actual force measurements. **Enhanced with automatic recalibration capabilities for improved long-term accuracy.**

## Features

- **Real-time force/torque data** - 6-DoF measurements at 1kHz
- **Automatic calibration** - Removes sensor drift and offset errors
- **Multiple calibration methods** - Service calls, topic triggers, or time-based
- **Calibration monitoring** - Real-time status feedback
- **Thread-safe operation** - Safe concurrent access to calibration functions
- **Configurable parameters** - Flexible setup for different applications

## 1. Download and Build Package

### Download the source repository
```bash
cd /path/to/ros2-workspace/src
git clone https://github.com/labiybafakh/leptrino_force_torque_sensor
```

### Compile the package
```bash
cd /path/to/ros2-workspace/
colcon build --packages-select leptrino_force_torque_sensor
```

### Source the workspace
```bash
source install/local_setup.bash
```

## 2. Basic Launch

### Set serial port permissions
```bash
sudo chmod 666 /dev/ttyACM0
```

### Launch the sensor node
```bash
ros2 launch leptrino_force_torque_sensor leptrino.launch.py 
```

### Launch with custom parameters
```bash
# Launch with different serial port
ros2 launch leptrino_force_torque_sensor leptrino.launch.py com_port:=/dev/ttyUSB0

# Launch with auto-recalibration enabled
ros2 launch leptrino_force_torque_sensor leptrino.launch.py \
  auto_recalibration_enabled:=true \
  auto_recalibration_interval_minutes:=15.0

# Launch with custom calibration sample count
ros2 launch leptrino_force_torque_sensor leptrino.launch.py \
  calibration_samples:=200
```

## 3. Sensor Calibration

### Why Calibration is Important

Force-torque sensors experience **drift over time** due to:
- **Temperature changes** - Affects strain gauge readings
- **Mechanical stress** - Mounting forces and vibrations
- **Electronic drift** - Amplifier offset changes
- **Gravitational effects** - Sensor orientation dependency
- **Component aging** - Long-term material changes

**Calibration removes these systematic errors** by measuring the sensor's zero-point when no external forces are applied.

### Calibration Methods

#### Method 1: Manual Calibration via Service (Recommended)

**When to use:** For on-demand, precise calibration.

```bash
# Trigger calibration
ros2 service call /leptrino/recalibrate std_srvs/srv/Trigger

# Expected response:
# success: True
# message: "Recalibration completed successfully"
```

#### Method 2: Topic-Based Calibration

**When to use:** For integration with external systems or automated workflows.

```bash
# Trigger calibration via topic
ros2 topic pub /leptrino/recalibrate_trigger std_msgs/msg/Empty "{}" --once
```

#### Method 3: Automatic Calibration

**When to use:** For continuous operation without manual intervention.

Configure in `config/params.yaml`:
```yaml
leptrino_sensor:
  ros__parameters:
    auto_recalibration_enabled: true
    auto_recalibration_interval_minutes: 30.0  # Every 30 minutes
    calibration_samples: 100
```

Or via launch parameters:
```bash
ros2 launch leptrino_force_torque_sensor leptrino.launch.py \
  auto_recalibration_enabled:=true \
  auto_recalibration_interval_minutes:=20.0
```

### Calibration Best Practices

#### Pre-Calibration Setup
1. **Ensure no external forces** - Remove all loads from the sensor
2. **Stable mounting** - Minimize vibrations and movement
3. **Temperature stability** - Wait for thermal equilibrium (~5 minutes)
4. **Proper orientation** - Sensor should be in operational position

#### Calibration Steps
```bash
# 1. Launch the sensor
ros2 launch leptrino_force_torque_sensor leptrino.launch.py

# 2. Wait for sensor initialization (check logs for "Data acquisition is started")

# 3. Remove all external forces from the sensor

# 4. Trigger calibration
ros2 service call /leptrino/recalibrate std_srvs/srv/Trigger

# 5. Wait for completion (typically 100ms for 100 samples)

# 6. Verify calibration success in logs
```

#### Monitoring Calibration Status
```bash
# Monitor calibration status in real-time
ros2 topic echo /leptrino/calibration_status

# While calibrating: data: true
# When complete: data: false
```

### Calibration Timing Guidelines

| Application | Frequency | Trigger |
|-------------|-----------|---------|
| **Research/Lab** | Before each experiment | Manual service call |
| **Industrial** | Every 15-30 minutes | Automatic timer |
| **Robotics** | Before precision tasks | Programmatic service call |
| **Continuous** | Temperature change >5°C | Manual or automatic |

## 4. Configuration Parameters

### Available Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `com_port` | string | `/dev/ttyACM0` | Serial port device |
| `auto_recalibration_enabled` | bool | `false` | Enable automatic recalibration |
| `auto_recalibration_interval_minutes` | double | `30.0` | Minutes between auto-calibrations |
| `calibration_samples` | int | `100` | Number of samples for calibration |

### Parameter Configuration

#### Via config file (`config/params.yaml`):
```yaml
leptrino_sensor:
  ros__parameters:
    com_port: "/dev/ttyACM0"
    auto_recalibration_enabled: true
    auto_recalibration_interval_minutes: 15.0
    calibration_samples: 200
```

#### Via launch arguments:
```bash
ros2 launch leptrino_force_torque_sensor leptrino.launch.py \
  com_port:=/dev/ttyACM0 \
  auto_recalibration_enabled:=true \
  auto_recalibration_interval_minutes:=10.0 \
  calibration_samples:=150
```

## 5. Topics and Services

### Published Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/leptrino` | `geometry_msgs/WrenchStamped` | Force/torque measurements |
| `/leptrino/calibration_status` | `std_msgs/Bool` | Calibration status (true = calibrating) |

### Subscribed Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/leptrino/recalibrate_trigger` | `std_msgs/Empty` | Trigger calibration via topic |

### Services

| Service | Service Type | Description |
|---------|--------------|-------------|
| `/leptrino/recalibrate` | `std_srvs/Trigger` | Manual calibration trigger |

### Example Usage

```bash
# Monitor force/torque data
ros2 topic echo /leptrino

# Monitor calibration status
ros2 topic echo /leptrino/calibration_status

# Trigger calibration via service
ros2 service call /leptrino/recalibrate std_srvs/srv/Trigger

# Trigger calibration via topic
ros2 topic pub /leptrino/recalibrate_trigger std_msgs/msg/Empty "{}" --once

# List all leptrino topics
ros2 topic list | grep leptrino

# List all leptrino services
ros2 service list | grep leptrino
```