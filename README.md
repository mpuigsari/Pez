# Pez – ROS2 Teleoperation, Sensor Integration & Acoustic Control of a Robotic Fish

> A comprehensive solution for underwater robotic teleoperation, sensor integration, and wireless communication using ROS2, Docker, Python sensor drivers, and acoustic modems.

---

## 🌊 Project Overview

### Developed at the Research Center in Robotics and Underwater Technologies (CIRTESU), Universitat Jaume I (UJI), Castellón, Spain.

The **Pez** project enables seamless remote operation and control of an underwater robotic fish by integrating:

* **ROS2 Humble** for robust robotics control
* **Docker** for simplified deployment and reproducibility
* **Bluerobotics Navigator** for actuator control (servos, camera, electromagnet)
* Real-time visualization with **RQT** and **PlotJuggler**
* Integrated environmental sensing (temperature, pressure, depth)
* Custom acoustic communication protocols for remote command transmission

This project facilitates both simulation-based testing and real-world robotic fish teleoperation, designed for use in educational, research, or practical underwater inspection and monitoring scenarios.

---

## 🧰 Technologies & Tools

* [ROS2 Humble](https://docs.ros.org/en/humble/)
* [Docker and Docker Compose](https://docs.docker.com/compose/)
* [Bluerobotics Navigator](https://bluerobotics.com/store/comm-control-power/control/navigator/)
* [RQT](https://wiki.ros.org/rqt)
* [PlotJuggler](https://github.com/facontidavide/PlotJuggler)
* [tsys01-python](https://github.com/bluerobotics/tsys01-python)
* [ms5837-python](https://github.com/bluerobotics/ms5837-python)
* [smbus2](https://pypi.org/project/smbus2/)

---

## 🧱 System Architecture

```
+---------------------------+       +---------------------------+       +---------------------------+
|     Fish-side (Pi 4)      | <---> |     Acoustic Comms        | <---> |       Host-side           |
|---------------------------|       |---------------------------|       |---------------------------|
| - ROS2 Humble (Docker)    |       | - Custom Packet Protocol  |       | - ROS2 Humble (Docker)    |
| - Actuator Control        |       | (Packet A, B, future C)   |       | - Joystick Teleoperation  |
| - Sensor Data Publishing  |       |                           |       | - Real-time Visualization |
|   • TSYS01 Temperature    |       |                           |       | • Sensor Data Reception   |
|   • MS5837 Pressure/Depth |       |                           |       |                           |
+---------------------------+       +---------------------------+       +---------------------------+
```

---

## 🚀 Quick Start

### Fish-side (Raspberry Pi 4, 64-bit)

Copy the compose file from
[`pez_docker/pez`](pez_ros/pez_docker/pez/docker-compose.yml) to the
Raspberry Pi and start one of the **modes**. The compose file pulls the tagged
image automatically:

```bash
cp pez_ros/pez_docker/pez/docker-compose.yml .
# Interactive shell for maintenance
docker compose run --rm pez-dev
# Tele‑op over USB
docker compose up pez-cable
# Tele‑op with acoustic comms
docker compose up pez-comms
```

The same container image is reused – the first word passed to the entrypoint
(`dev`, `cable` or `comms`) selects the launch behaviour.

### Host-side (Ubuntu Jammy, 64-bit compatible)

Clone the repository, copy the compose file from
[`pez_docker/host`](pez_ros/pez_docker/host/docker-compose.yml) and start the
**host** container (`mapuigsari/pez:core-amd64`) in the desired mode:

```bash
# Clone and copy compose file
git clone https://github.com/mpuigsari/Pez
cp Pez/pez_ros/pez_docker/host/docker-compose.yml ./host-compose.yml
# Short-lived dev shell
docker compose run --rm pez-dev
# USB/serial tether
docker compose up pez-cable
# Radio/MAVLink comms
docker compose up pez-comms
```

Plug in your joystick (and any serial adapters) before starting.
See [host Docker README](pez_ros/pez_docker/host/README.md) for details.

### Native ROS2 Build (optional)

```bash
cd ~/pez_ws/src
git clone https://github.com/mpuigsari/Pez pez_core
git clone https://github.com/mpuigsari/Pez pez_comms
cd ~/pez_ws
colcon build
source install/setup.bash
```

---

## 📁 Repository Structure

```
Pez
├── pez_ros/
│   ├── pez_docker/   # Docker setups for fish, host and buoy
│   ├── pez_humble/   # ROS 2 packages shared across all platforms
│   └── pez_noetic(discarded)/
├── blueos_ros/       # ROS Noetic workspace for BlueROV2 teleop
└── README.md
```

---

## 🐟 Fish-side Container

ROS 2 Humble image for the onboard Raspberry Pi. It exposes Navigator-driven PWM control and publishes sensor data.

*See [pez_docker](pez_ros/pez_docker/README.md) for all container images.*

---

## 🖥️ Host-side Container & Workspace

Provides ROS 2 Humble with joystick support, visualization tools and optional acoustic modem bridge. The container tag is `mapuigsari/pez:core-amd64`.

* [pez\_humble](pez_ros/pez_humble/README.md)

---

## 🔧 Core ROS Packages

* **[`pez_core`](pez_ros/pez_humble/pez_ws/src/pez_core/README.md)**: Teleoperation, actuator control, and sensor nodes.
* **[`pez_comms`](pez_ros/pez_humble/pez_ws/src/pez_comms/README.md)**: YAML-driven serial communication with plugin extensibility.

---

## 📡 Acoustic Communication

Custom lightweight packets are defined in `pez_comms/core/packet_def.py`:

* **PacketA_Normal** – 8‑bit thruster commands for simple motion.
* **PacketB_Command** – 8‑bit service request/response format.
* **Packet40** – 32‑bit velocity + service field packet for ROS 2 teleop.
* **PacketB_Full** – 32‑bit service command with CRC protection.
* **PacketBlueRov** – 32‑bit twist packet for BlueROV2 control.

These allow a Noetic vehicle and a Humble host to communicate seamlessly over the acoustic modem.

---

## 🚧 Future Work & Roadmap

* **Packet C Implementation** – extended packet carrying 6‑DoF pose data
* **Forward Error Correction (FEC)** – improve reliability of acoustic transmissions
* **Boya Surface Buoy** – Wi‑Fi link to host and acoustic link to fish for increased range
* **Additional Architectures** – Docker support for multiple hardware platforms
* **Enhanced Simulation Environment** – improved simulation tools and automated testing

---

## 🤝 Contributing & License

**License**: *(confirm or specify your license preference)*

---

👤 **Author**

**Max Puig**
Bachelor in Robotics Intelligence – Universitat Jaume I (2021–2025)

> This project forms part of my academic portfolio, demonstrating integration of ROS2, Docker, sensor fusion, and real-time communication for robotic teleoperation.
