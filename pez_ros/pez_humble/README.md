# Pez Humble (Host-side)

The `pez_humble` directory provides a comprehensive ROS 2 Humble environment tailored for developing, controlling, interacting, and sensor integration for the **Pez robot fish**. It integrates ROS packages (`pez_core` and `pez_comms`) and includes Docker configurations (`Dockerfile`, `entrypoint.sh`, and `docker-compose.yml`) for seamless deployment.

---

## Overview

`pez_humble` provides a **host-side Docker container** for ROS 2 Humble, tailored to control and interact with the Pez robot fish. The ROS2 packages (`pez_core`, `pez_comms`) can be built and used in any ROS2 environment, including the fish-side container in `pez_docker` or a native installation.

Core components include:

* ROS 2 Humble environment within Docker (host-side)
* Shared ROS2 packages:

  * `pez_core`: Teleoperation, actuator control, and sensor integration (`fish_sense`).
  * `pez_comms`: Acoustic communication handling.
* Joystick integration, visualization tools (RQT and PlotJuggler), and real-time sensor monitoring.

---

## ROS Packages Included

### [`pez_core`](./pez_ws/src/pez_core/README.md)

Responsible for real-time teleoperation, control, and sensor data publishing:

* Tail and fins (PWM signals via Bluerobotics Navigator).
* Camera control (pan).
* Electromagnet toggling.
* Sensor Integration (`fish_sense` node):

  * TSYS01 temperature sensor.
  * MS5837 pressure and depth sensor.
  * Parameters dynamically reconfigurable via `rqt_reconfigure`.

### [`pez_comms`](./pez_ws/src/pez_comms/README.md)

Manages acoustic modem-based communication through a YAML-configurable node:

* Pre-defined packet formats for velocity and command data.
* Serial communication configured entirely via YAML.
* Topic and service behaviour extendable with custom plugins.

### [`pez_interfaces`](./pez_ws/src/pez_interfaces)

Defines custom service types for sensor snapshots used across the project:

* `SnapSensors.srv` for requesting immediate sensor readings.

---

## Docker Deployment

The provided Docker configuration simplifies host-side deployments, ensuring a consistent development and operational environment.

### Dockerfile

Builds a ROS 2 Humble container with necessary dependencies:

* ROS 2 Humble (Jammy-based)
* Essential development tools (`colcon`, joystick libraries, Navigator libs)
* Python sensor libraries (`tsys01-python`, `ms5837-python`, `smbus2`)
* ROS workspace setup

### Entrypoint (`entrypoint.sh`)

This script configures the runtime environment:

* Sources ROS 2 Humble environment
* Sources pre-built workspace setup (if available)
* Ensures correct permissions for newly created files
* Defaults to an interactive bash shell unless overridden by compose commands

### Docker Compose (`docker-compose.yml`)

Provides a ready-to-use Docker Compose setup:

* Mounts workspace (`pez_ws`) and device files (joystick `/dev/input/js0`)
* Configures GUI access through X11 (for visualization with RQT and PlotJuggler)
* Enables interaction with the Pez robot or simulation mode

#### Usage

```bash
cd Pez/pez_ros/pez_docker/host
docker compose up -d
```

Ensure joysticks and any serial adapters are connected before starting the container. Adjust the `DISPLAY` environment variable if needed for GUI access.

---

## Launch Configurations

Launch the Pez robot control nodes, sensor integration, and visualization tools:

* **Real Fish Control and Sensors:**

  ```bash
  ros2 launch pez_core teleop_launch.py
  ros2 run pez_core fish_sense --ros-args -p publish_tsys01_temperature:=True -p publish_ms5837_pressure:=True
  ```

* **Joystick Control, Sensors, and Visualization:**

  ```bash
  ros2 launch pez_core joy_launch.py display_flag:=true fish_robot:=false
  ros2 run pez_core fish_sense --ros-args -p publish_tsys01_temperature:=True -p publish_ms5837_pressure:=True
  ```

* **Simulation (No Robot Hardware):**

  ```bash
  ros2 launch pez_core joy_launch.py fish_robot:=false display_flag:=true
  ```

---

## Recommended Usage

* **Host-side Development:** Use Docker Compose setup for consistency across development environments.
* **Sensor Testing:** Verify sensor readings via ROS topics (`/tsys01/temperature`, `/ms5837/pressure`).
* **Visualization:** Utilize RQT and PlotJuggler for debugging, parameter tuning, and sensor data visualization.
* **Deployment:** Deploy `pez_core` directly onto Raspberry Pi using [`pez_docker`](/pez_ros/pez_docker/pez/README.md) container for onboard control and sensing.

---

## Further Information

For detailed instructions and explanations about individual components:

* [pez\_core README](./pez_ws/src/pez_core/README.md)
* [pez\_comms README](./pez_ws/src/pez_comms/README.md)
* [pez\_docker (Fish-side) README](/pez_ros/pez_docker/pez/README.md)

---

This environment is actively maintained to ensure compatibility and performance with ongoing developments of the Pez robot fish project.
