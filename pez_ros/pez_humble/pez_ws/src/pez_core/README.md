# `pez_core` ROS2 Package

## 1. Package Overview

The `pez_core` package provides ROS2 nodes for teleoperating and controlling the **Pez robot fish** using the Bluerobotics Navigator board. It enables direct interaction with the robot's actuators, including tail, fins, camera pan control, and electromagnet, and allows real-time adjustments through dynamic parameters.

### Key Functionalities

* Tail and fin gait control for maneuvering (forward, steering, dive)
* Camera pan adjustments
* Electromagnet toggle functionality
* Dynamic parameter adjustments via `rqt_reconfigure`

---

## 2. Dependencies

* ROS 2 Humble
* Python 3.9+ (for Bluerobotics Navigator compatibility)
* `bluerobotics_navigator` Python library
* ROS standard message packages: `geometry_msgs`, `std_msgs`, `std_srvs`

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
2. Build and source:

   ```bash
   cd ~/pez_ws
   colcon build --packages-select pez_core
   source install/setup.bash
   ```

This approach is ideal for development and rapid iteration when working directly on a supported Ubuntu Jammy system.

### 3.2. Docker Container Deployment

We provide two Docker-based options for simplified, reproducible environments. See the corresponding container READMEs for full details:

* **Fish-side Container**:
  Use the [`pez_docker`](/pez_ros/pez_docker/README.md) image on the Raspberry Pi 4 (64-bit) for onboard control of servos, camera, and electromagnet. It’s published to Docker Hub as `yourorg/pez_docker:arm64-latest`.

* **Host-side Container**:
  Use the [`pez_humble`](/pez_ros/pez_humble/README.md) environment on your development machine (any Jammy-compatible architecture). It provides ROS2 Humble with joystick support, RQT, and PlotJuggler for control and visualization.

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

| Topic      | Type                         | Description                                               |
| ---------- | ---------------------------- | --------------------------------------------------------- |
| `/pez/pwm` | `std_msgs/Float32MultiArray` | PWM outputs (tail, left\_fin, right\_fin, camera, magnet) |

### 5.3 Services

| Service                             | Type               | Description                           |
| ----------------------------------- | ------------------ | ------------------------------------- |
| `/pez/teleoperation/start_swim`     | `std_srvs/Trigger` | Starts swimming motion (tail & fins)  |
| `/pez/teleoperation/stop_swim`      | `std_srvs/Trigger` | Stops swimming and resets PWM signals |
| `/pez/teleoperation/toggle_magnet`  | `std_srvs/Trigger` | Toggles electromagnet state (on/off)  |
| `/pez/teleoperation/toggle_neutral` | `std_srvs/Trigger` | Toggles neutral buoyancy mode         |

---

## 6. Launch Files & Runtime Flags

### 6.1 `joy_launch.py`

Used for joystick teleoperation, optional GUI visualization with RQT and PlotJuggler.

**Launch Arguments:**

* `display_flag` (default: `true`):

  * `true`: Launches RQT and PlotJuggler
  * `false`: Skips launching RQT and PlotJuggler

* `fish_robot` (default: `true`):

  * `true`: Uses robot-side teleoperation via `teleop_launch.py`
  * `false`: Launches host-side teleoperation (`test_flag=true`)

**Recommendation:** Wait until `/pez/pwm` is actively publishing before connecting PlotJuggler subscribers.

### 6.2 `teleop_launch.py`

Launches main teleoperation and camera nodes, used by both robot (container entrypoint) and host (simulation).

**Launch Argument:**

* `test_flag` (default: `false`):

  * `false`: Runs hardware controller (`fish_teleop_node`) and USB camera
  * `true`: Runs simulation controller (`fish_teleop_node_test`) and skips camera node

**Note:** When running on the fish-side container, default (`test_flag=false`) is set automatically. For host-side simulations, explicitly set `fish_robot=false` in `joy_launch.py`, which forwards `test_flag=true` to enable simulation mode.

---

## Visual Demonstrations

Launch file behaviors and node functionalities are illustrated through visual GIFs included separately.
