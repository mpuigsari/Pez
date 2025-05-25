# Pez Humble (Host-side)

The `pez_humble` directory provides a comprehensive ROS 2 Humble environment tailored for developing, controlling, and interacting with the **Pez robot fish**. It integrates ROS packages (`pez_core` and `pez_comms`) and includes Docker configurations (`Dockerfile`, `entrypoint.sh`, and `docker-compose.yml`) for seamless deployment.

---

## Overview

`pez_humble` provides a **host-side Docker container** for ROS 2 Humble, tailored to drive and interact with the Pez robot fish, but **the ROS2 packages themselves (`pez_core`, `pez_comms`) can be built and used in any ROS2 environment**, including the fish-side container in `pez_docker` or a native install.

Core components include:

* ROS 2 Humble environment within Docker (host-side)
* Shared ROS2 packages:

  * `pez_core`: Teleoperation and actuator control
  * `pez_comms`: Acoustic communication handling
* Joystick integration and visualization tools (RQT and PlotJuggler)

---

## ROS Packages Included

### [`pez_core`](./pez_ws/src/pez_core/README.md)

Responsible for real-time teleoperation and control of:

* Tail and fins (PWM signals via Bluerobotics Navigator).
* Camera control (pan).
* Electromagnet toggling.
* Dynamic parameter reconfiguration via `rqt_reconfigure`.

### [`pez_comms`](./pez_ws/src/pez_comms/README.md)

Manages acoustic modem-based communication:

* Packet creation (Packet A and B) for wireless commands.
* Reception and interpretation of remote commands.
* Service requests handling through acoustic packets.

> **Note:** `pez_comms` is under active development and detailed documentation will be provided upon completion.

---

## Docker Deployment

The provided Docker configuration simplifies host-side deployments, ensuring a consistent development and operational environment.

### Dockerfile

Builds a ROS 2 Humble container with necessary dependencies:

* ROS 2 Humble (Jammy-based)
* Essential development tools (`colcon`, joystick libraries, Navigator libs)
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
docker-compose up --build
```

Ensure your joystick is connected before starting the container. Adjust the `DISPLAY` environment variable if needed for GUI access.

---

## Launch Configurations

Launch the Pez robot control nodes and visualization tools:

* **Real Fish Control:**

  ```bash
  ros2 launch pez_core teleop_launch.py
  ```

* **Joystick Control and Visualization:**

  ```bash
  ros2 launch pez_core joy_launch.py display_flag:=true fish_robot:=false
  ```

* **Simulation (No Robot Hardware):**

  ```bash
  ros2 launch pez_core joy_launch.py fish_robot:=false display_flag:=true
  ```

---

## Recommended Usage

* **Host-side Development:** Use the Docker Compose setup for consistency across development environments.
* **Visualization:** Utilize RQT and PlotJuggler for debugging and parameter tuning.
* **Deployment:** Deploy the `pez_core` directly onto the Raspberry Pi using the [`pez_docker`](/pez_ros/pez_docker/README.md) container for onboard control.

---

## Further Information

For more detailed instructions and explanations about individual components:

* [pez\_core README](./pez_ws/src/pez_core/README.md)
* [pez\_comms README](./pez_ws/src/pez_comms/README.md) *(Coming Soon)*
* [pez\_docker (Fish-side) README](/pez_ros/pez_docker/README.md)

---

This environment is actively maintained to ensure compatibility and performance with ongoing developments of the Pez robot fish project.
