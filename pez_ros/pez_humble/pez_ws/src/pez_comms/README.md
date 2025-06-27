# pez\_comms

**Version 0.1.0**

> Config-Driven Communication Engine for the Pez Robotic Fish

`pez_comms` is a YAML-driven, plugin-extensible ROS 2 package providing a single, generic node (`comms`) that can drive **both** fish-side and host-side acoustic-modem links. Define your entire comms flow—packet decode rules, timed sends, topic- and service-triggered packets and more—by editing a simple YAML file. No code changes needed to modify or extend a flow!

---

## Table of Contents

1. [Features](#features)
2. [Repository Layout](#repository-layout)
3. [Installation](#installation)
4. [Configuration](#configuration)
5. [Launch Files](#launch-files)
6. [Plugins](#plugins)
7. [Core Modules](#core-modules)
8. [Examples](#examples)
9. [Contributing](#contributing)
10. [License](#license)

---

## Features

* **Single Generic Node** (`comms`): All comms flows handled by one executable.
* **YAML-Driven**: Everything from packet IDs to ROS-topic mappings lives in `config/*.yaml`.
* **Inbound Handling**: Decode any packet into ROS topics or service calls.
* **Timed Schedules**: Fire off packets at arbitrary intervals.
* **Topic Triggers**: Publish or send packets in response to ROS topics.
* **Service Bridges**: Expose ROS services that encode packets, wait for ACK/NACK, then return or publish follow-ups.
* **Dynamic Parameters**: Tweak rates, payload fields, timeouts at runtime with `ros2 param set`.
* **Plugin Architecture**: Drop in a Python module under `plugins/` to hook any custom logic.

---

## Repository Layout

```
pez_comms/
├── config/                # YAML profiles
│   ├── fish_comms.yaml    # fish-side default flow
│   ├── host_comms.yaml    # host-side default flow
│   └── guide_config.yaml  # annotated example for all sections
├── launch/                # launch scripts
│   ├── comms_launch.py    # generic `comms` node launcher
│   ├── fish_launch.py     # wraps `comms_launch.py` under /fish
│   ├── host_launch.py     # wraps `comms_launch.py` under /host
│   └── test_launch.py     # socat + dual-namespace end-to-end test
├── nodes/                 # generic comms node
│   └── comms_node.py      # entry point for `comms` executable
├── plugins/               # optional custom logic
│   ├── fish_side.py       # example fish-side packet loop
│   └── host_side.py       # example host-side scheduler loop
├── core/                  # shared libraries
│   ├── modem_io.py        # serial I/O manager  
│   ├── packet_def.py      # registry and codecs for packets  
│   ├── scheduler.py       # TransmissionScheduler & steps  
│   └── serial_test.py     # helper CLI for debugging ports  
├── package.xml            # ROS2 package manifest
├── setup.py               # setuptools entry point
└── resource/pez_comms     # ament index entry
```

---

## Installation

```bash
# 1) Install system dependencies (Ubuntu Jammy / ROS 2 Humble)
sudo apt update && sudo apt install -y \
  python3-pip python3-colcon-common-extensions \
  build-essential cmake ros-humble-serial ros-humble-usb-cam

# 2) Install Python libraries
pip3 install --user pyserial pyyaml

# 3) Build your workspace
cd ~/pez_ws
colcon build --packages-select pez_comms
source install/setup.bash
```

---

## Configuration

All communication logic is declared in YAML under `config/*.yaml`. Key sections:

* **`modem_io`**: serial port, baud rate, timeouts
* **`parameters`**: expose ROS params for dynamic control
* **`schedules`**: timed packet sends
* **`triggers`**: ROS-topic → packet (on publish)
* **`publishers`**: ROS-topic → periodic packet sends
* **`services`**: ROS services → send packet + return/publish result
* **`serial_handlers`**: incoming packet IDs → decode & publish/service
* **`plugins`**: load extra Python hooks

See [`guide_config.yaml`](config/guide_config.yaml) for a fully annotated reference.

---

## Launch Files

All `ros2 launch` commands assume you’ve sourced your workspace:

```bash
# Generic (use any config):
ros2 launch pez_comms comms_launch.py \
  config_file:=config/fish_comms.yaml \
  port:=/dev/ttyUSB0

# Fish-side (namespaced /fish):
ros2 launch pez_comms fish_launch.py \
  config_file:=config/fish_comms.yaml

# Host-side (namespaced /host):
ros2 launch pez_comms host_launch.py \
  config_file:=config/host_comms.yaml

# End-to-end test (socat + both namespaces):
ros2 launch pez_comms test_launch.py
```

---

## Plugins

Drop your own modules under `plugins/`. Each must implement:

```python
def register(node: rclpy.node.Node, cfg: dict):
    """Called by comms_node after startup."""
    # e.g. node.create_subscription(...)
```

Built-in examples:

* `fish_side.py`: inbound decode loop for Packet A/B with ACK/NACK
* `host_side.py`: outbound scheduler state machine

---

## Core Modules

* **`modem_io.py`**: abstracts serial reads/writes into thread-safe queues
* **`packet_def.py`**: register, encode/decode packet classes by ID
* **`scheduler.py`**: define sequences of timed sends (TransmissionScheduler)
* **`serial_test.py`**: CLI for quick port sanity checks

---

## Examples

### Fish-side only

```bash
ros2 launch pez_comms fish_launch.py \
  config_file:=config/fish_comms.yaml
```

### Host-side only

```bash
ros2 launch pez_comms host_launch.py \
  config_file:=config/host_comms.yaml
```

### Loopback Test

```bash
ros2 launch pez_comms test_launch.py
```

---

## Contributing

Bug reports, feature requests, and pull requests welcome! New comms flows should go in YAML or plugin modules—core changes only when strictly necessary.

---

## License

Apache 2.0
