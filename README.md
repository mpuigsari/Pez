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

### Fish-side (Raspberry Pi 4, 64-bit)

Pull and run the container:

```bash
docker pull mapuigsari/pez:core-arm64v8
docker compose -f pez_ros/pez_docker/pez/docker-compose.yml up -d
```

To enable acoustic teleoperation add `comms` after the image name:
`docker run ... mapuigsari/pez:core-arm64v8 comms` or set
`command: ["comms"]` in the compose file.

### Host-side (Ubuntu Jammy, 64-bit compatible)

Clone the repository and launch:

```bash
git clone https://github.com/mpuigsari/Pez
cd Pez/pez_ros/pez_docker/host
docker compose up -d
```

Plug in your joystick (and USB‑serial adapter for acoustic comms) before
starting the container.  See [pez_core](pez_ros/pez_humble/pez_ws/src/pez_core/README.md)
for launch commands.

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
├── pez_docker/       # Fish-side Docker container (on-board Pi)
├── pez_humble/       # Host-side Docker & ROS2 workspace
└── pez_noetic/       # Deprecated ROS1/BlueOS (archived)
```

---

## 🐟 Fish-side Container

Runs ROS2 Humble, Navigator-lib onboard Raspberry Pi, providing direct control of actuators and sensors:

* [pez\_docker](pez_ros/pez_docker/README.md)

---

## 🖥️ Host-side Container & Workspace

Provides ROS2 Humble, joystick support, acoustic communication, sensor integration, and visualization tools:

* [pez\_humble](pez_ros/pez_humble/README.md)

---

## 🔧 Core ROS Packages

* **[`pez_core`](pez_ros/pez_humble/pez_ws/src/pez_core/README.md)**: Teleoperation, actuator control, and sensor nodes.
* **[`pez_comms`](pez_ros/pez_humble/pez_ws/src/pez_comms/README.md)**: YAML-driven serial communication with plugin extensibility.

---

## 📡 Acoustic Communication

Custom lightweight acoustic communication packets:

* **Packet A**: Continuous joystick and real-time command data
* **Packet B**: Service requests and responses
* *(Packet C: future implementation for 6DOF pose data)*

Detailed documentation will follow upon completion of development.

---

## 🚧 Future Work & Roadmap

* **Packet C Implementation**: Extended acoustic packets with 6DOF pose data
* **Forward Error Correction (FEC)**: Enhanced reliability of acoustic transmissions
* **Additional Architectures**: Docker support for multiple hardware platforms
* **Enhanced Simulation Environment**: Improved simulation tools and automated testing

---

## 🤝 Contributing & License

**License**: *(confirm or specify your license preference)*

---

👤 **Author**

**Max Puig**
Bachelor in Robotics Intelligence – Universitat Jaume I (2021–2025)

> This project forms part of my academic portfolio, demonstrating integration of ROS2, Docker, sensor fusion, and real-time communication for robotic teleoperation.
