# `pez_core` ROS2 Package

## 1. Package Overview

The `pez_core` package provides ROS2 nodes for teleoperating, controlling, and sensing capabilities for the **Pez robot fish** using the Bluerobotics Navigator board. It enables direct interaction with the robot's actuators, including tail, fins, camera pan control, and electromagnet, alongside integrated sensor support for real-time monitoring of environmental parameters.

### Key Functionalities

* Tail and fin gait control for maneuvering (forward, steering, dive)
* Camera pan adjustments
* Electromagnet toggle functionality
* Dynamic parameter adjustments via `rqt_reconfigure`
* Real-time sensor data integration:

  * TSYS01 Temperature Sensor
  * MS5837 Pressure and Depth Sensor

---

## 2. Dependencies

* ROS 2 Humble
* Python 3.9+ (for Bluerobotics Navigator compatibility)
* `bluerobotics_navigator` Python library
* ROS standard message packages: `geometry_msgs`, `std_msgs`, `std_srvs`
* Python Sensor Modules:

  * [`tsys01-python`](https://github.com/bluerobotics/tsys01-python)
  * [`ms5837-python`](https://github.com/bluerobotics/ms5837-python)
  * [`smbus2`](https://pypi.org/project/smbus2/)

---

## 3. Installation & Build

There are two primary ways to deploy and run `pez_core`:

### 3.1. Local ROS2 Build

Use this method if you prefer a native ROS2 workspace build and have the requisite hardware and libraries installed.

1. Clone into your ROS workspace:

   ```bash
   cd ~/pez_ws/src
   git clone <repo-url> pez_core
   ```

2. Install Python sensor dependencies:

   ```bash
   pip3 install tsys01 ms5837 smbus2
   ```

3. Build and source:

   ```bash
   cd ~/pez_ws
   colcon build --packages-select pez_core
   source install/setup.bash
   ```

### 3.2. Docker Container Deployment

We provide two Docker-based options for simplified, reproducible environments. See the corresponding container READMEs for full details:

* **Fish-side Container**:
  Use the [`pez_docker`](/pez_ros/pez_docker/pez/README.md) image on the Raspberry Pi 4 (64-bit) for onboard control of servos, camera, electromagnet, and sensor publishing.

* **Host-side Container**:
  Use the [`pez_humble`](/pez_ros/pez_humble/README.md) environment on your development machine (Jammy-compatible architecture). It provides ROS2 Humble with joystick support, RQT, PlotJuggler, and sensor monitoring.

---

## 4. Parameters

Parameters can be dynamically adjusted at runtime via `rqt_reconfigure`:

| Parameter              | Type  | Default | Range       | Description                                |
| ---------------------- | ----- | ------- | ----------- | ------------------------------------------ |
| `k_accel`              | float | 0.5     | 0.0 – 2.0   | Acceleration gain for forward speed        |
| `k_drag`               | float | 0.3     | 0.0 – 3.0   | Drag coefficient                           |
| `tail_freq_slow`       | float | 1.5     | 0.1 – 3.0   | Slow gait tail frequency (Hz)              |
| `tail_freq_fast`       | float | 5.0     | 2.0 – 10.0  | Fast gait tail frequency (Hz)              |
| `tail_deg_amp_slow`    | float | 30.0    | 0.0 – 60.0° | Tail amplitude for slow speed (degrees)    |
| `tail_deg_amp_fast`    | float | 20.0    | 0.0 – 60.0° | Tail amplitude for fast speed (degrees)    |
| `tail_max_deflect`     | int   | 75      | 0 – 150     | Max tail deflection for steering (degrees) |
| `rate_hz`              | int   | 50      | 10 – 200    | Control loop rate (Hz)                     |
| `fin_max_deflect`      | int   | 75      | 0 – 150     | Max fin deflection angle (degrees)         |
| `fin_turn_max_deflect` | int   | 20      | 0 – 50      | Max fin deflection for turning (degrees)   |
| `fin_blend`            | float | 0.5     | 0.0 – 1.0   | Blend factor for fin position smoothing    |

---

## 5. ROS Interfaces

### 5.1 Subscribed Topics

| Topic                 | Type                  | Description                                               |
| --------------------- | --------------------- | --------------------------------------------------------- |
| `/pez/cmd_vel`        | `geometry_msgs/Twist` | Controls linear x (forward), y (steering), z (dive/climb) |
| `/pez/camera_control` | `std_msgs/Float64`    | Camera pan commands (±1 steps, zero ignored)              |

### 5.2 Published Topics

| Topic                 | Type                         | Description                                               |
| --------------------- | ---------------------------- | --------------------------------------------------------- |
| `/pez/pwm`            | `std_msgs/Float32MultiArray` | PWM outputs (tail, left\_fin, right\_fin, camera, magnet) |
| `/tsys01/temperature` | `sensor_msgs/Temperature`    | TSYS01 Temperature readings                               |
| `/ms5837/temperature` | `sensor_msgs/Temperature`    | MS5837 Temperature readings                               |
| `/ms5837/pressure`    | `sensor_msgs/FluidPressure`  | MS5837 Pressure readings (converted to Pa)                |
| `/ms5837/depth`       | `std_msgs/Float32`           | MS5837 Depth readings (m)                                 |

### 5.3 Services

| Service                             | Type               | Description                           |
| ----------------------------------- | ------------------ | ------------------------------------- |
| `/pez/teleoperation/start_swim`     | `std_srvs/Trigger` | Starts swimming motion (tail & fins)  |
| `/pez/teleoperation/stop_swim`      | `std_srvs/Trigger` | Stops swimming and resets PWM signals |
| `/pez/teleoperation/toggle_magnet`  | `std_srvs/Trigger` | Toggles electromagnet state (on/off)  |
| `/pez/teleoperation/toggle_neutral` | `std_srvs/Trigger` | Toggles neutral buoyancy mode         |

---

## 6. Launch Files & Runtime Flags

### `joy_launch.py`

Joystick teleoperation, GUI visualization, and sensor node launching:

```bash
ros2 launch pez_core joy_launch.py display_flag:=true fish_robot:=true
ros2 run pez_core fish_sense
```

### `teleop_launch.py`

Main teleoperation and camera nodes, used for robot or host simulation:

```bash
ros2 launch pez_core teleop_launch.py test_flag:=false
ros2 run pez_core fish_sense
```

---

Visual demonstrations of launch behaviors, teleoperation, and sensor integration will be provided separately.
