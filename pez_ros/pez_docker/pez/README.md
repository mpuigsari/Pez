# Pez Docker (Raspberry Pi 4, 64-bit)

This repository provides the Docker configuration for the ROS 2 Humble–based control software for the **Pez robot fish**, optimized for the **Raspberry Pi 4 (64‑bit)**. This container offers a plug-and-play onboard environment, fully compatible with the Bluerobotics Navigator library.

---

## Overview

The `pez_docker` image encapsulates:

* **ROS 2 Humble** runtime on Ubuntu Jammy
* **Bluerobotics Navigator** Python 3.9 library
* Teleoperation nodes for tail, fins, electromagnet, and camera

It has been tested on Raspberry Pi 4 (ARM64), published to Docker Hub, and can be easily adapted to any architecture supported by the `ros:humble-ros-base-jammy` base image.

---

## 📦 Docker Hub Publication

Built and published automatically via GitHub Actions for **ARM64 (Raspberry Pi 4)**:

```bash
docker pull mapuigsari/blueos-ros2-navigator:arm64v8
```

Future tags may include additional architectures as needed.

---

## 🛠️ Usage

### 1. Example `docker-compose.yml` on Raspberry Pi OS (64‑bit)

```yaml
version: '3.8'
services:
  pez:
    image: mapuigsari/blueos-ros2-navigator:arm64v8
    restart: unless-stopped    # auto-restart on reboot or failure
    privileged: true           # allow PWM and device access
    volumes:
      - /dev:/dev              # expose hardware interfaces
    environment:
      - DISPLAY=${DISPLAY}     # for GUI tools if needed
```

Save as `docker-compose.yml` and run:

```bash
docker-compose up -d
```

### 2. Manual `docker run` with restart

```bash
docker run -d \
  --name pez \
  --privileged \
  --restart unless-stopped \
  -v /dev:/dev \
  yourorg/pez_docker:arm64-latest
```

No additional setup is required; the container initializes ROS 2 and launches teleop nodes automatically.

---

## 🏗️ Building for Other Architectures

The Dockerfile is based on `ros:humble-ros-base-jammy` and can be rebuilt for any architecture supported by that image:

```bash
docker build \
  --platform linux/amd64 \
  -t yourorg/pez_docker:amd64-latest \
  .
```

> **Note:** For **32‑bit** architectures (e.g. Raspberry Pi OS 32-bit), ROS 2 Humble does not offer official binaries. You must [build ROS 2 Humble from source](https://docs.ros.org/en/humble/Installation/Alternatives/Building-From-Source.html) before constructing this image.

---

## 📁 Repository Structure

```plaintext
pez_docker/
├── Dockerfile           # base image, dependencies, entrypoint
└── ros_entrypoint.sh    # launches ROS 2 nodes on container start
```

---

## 🚀 Recommended Host‑side Setup

To complement the onboard container, see:

* [**pez\_humble**](../pez_humble/README.md): Host‑side ROS 2 Humble environment with joystick, GUI tools, and wireless comms.

---

## 📚 Development History

1. **BlueOS + Jupyter Notebooks** (initial proof of concept)
2. **BlueOS + ROS Noetic** (discarded: 32-bit/Python mismatch)
3. **Raspbian 64‑bit + ROS 2 Humble (Docker)** (current, actively maintained)

---

> ⚠️ **Ensure you pull the matching architecture tag** for your platform to guarantee compatibility and optimal performance.
