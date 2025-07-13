# Pez Noetic (Discarded)

This repository contains the ROS Noetic-based control software for the **Pez robot fish**, initially developed to run on the **BlueOS platform** with a **32-bit architecture**. This approach has since been discontinued due to compatibility constraints and architectural limitations.

---

## Overview

The goal of this project was to enable remote control of the Pez robot fish using ROS Noetic on a Raspberry Pi running BlueOS. This directory serves as an archive for historical reference, showcasing the earlier stages of development and experimentation.

---

## 🚨 **Why Was This Approach Discarded?**

Key challenges that led to the abandonment of this method include:

* **Python Version Compatibility**: The Bluerobotics Navigator library requires Python 3.9, while ROS Noetic officially supports Python 2.7. This created major compatibility issues.
* **32-bit Architecture Limitation**: ROS2, the natural alternative to address Python compatibility, provides limited support for 32-bit systems, which BlueOS utilizes.

These constraints prompted a strategic shift to a **64-bit Raspbian-based solution with ROS2 Humble**, ensuring compatibility with the Bluerobotics Navigator library and leveraging modern ROS2 features.

---

## 📁 **Repository Structure**

The contents of this repository include:

* **`docker_pez_ros/`**: Docker configurations and scripts for ROS Noetic-based teleoperation.
* **`pez_ws_rosnoetic/`**: ROS workspace containing packages, scripts, and resources for basic robot control, joystick interfacing, and teleoperation.
* **`github/`**: GitHub workflows originally designed for continuous integration and deployment, automating Docker image builds.

```
pez_noetic
├── docker_pez_ros
│   ├── docker-compose.yml
│   ├── Dockerfile
│   └── entrypoint.sh
├── github
│   └── workflows
│       └── build-and-push.yml
└── pez_ws_rosnoetic
    ├── build
    ├── devel
    └── src
        ├── bluerobotics_navigator_stub
        ├── CMakeLists.txt
        └── pez
            ├── CMakeLists.txt
            ├── config
            ├── launch
            ├── package.xml
            └── scripts
```

---

## 📌 **Recommendations**

This directory remains solely for **historical reference and archival purposes**:

* ✅ Feel free to use this codebase as a reference for understanding legacy implementations or migrating functionalities to the current ROS2 setup.
* ❌ **Do not use** this repository as the basis for new deployments or active developments, as no further support or updates will be provided.

---

## 🚀 **Current Recommended Solution**

Please refer to the actively maintained and supported solutions:

* [**pez\_docker**](../pez_docker/pez/README.md): Raspberry Pi 64-bit ROS2 container for onboard control
* [**pez\_humble**](../pez_humble/README.md): Host-side ROS2 Humble stack for development and teleoperation

These updated repositories ensure full compatibility with the Navigator library, ROS2 Humble, and modern Python environments.

---

## 📚 **Development History**

This directory represents an intermediate step in the evolution of the Pez robot fish project:

1. **Initial implementation**: BlueOS with Jupyter notebooks and keyboard-based teleoperation
2. **Intermediate step (this repository)**: BlueOS with ROS Noetic (discarded due to aforementioned constraints)
3. **Current solution**: Raspbian 64-bit with ROS2 Humble and Docker-based deployment

---

⚠️ **Note**: This repository is archived, and no further development or support will be provided.
