# pez\_comms

> **Config-Driven Communication Engine for the Pez Robotic Fish**

`pez_comms` is a unified, YAML‑driven ROS 2 package that handles both **fish‑side** and **host‑side** acoustic modem communication.  A single node (`comms`) reads declarative configuration files to:

* **Decode inbound packets** → publish ROS topics or call services
* **Schedule outbound packets** at arbitrary intervals
* **Trigger packets** from incoming ROS topics
* **Publish ROS messages** on a timer (parameter‑driven)
* **Bridge ROS services** to packet senders and optional follow‑ups
* **Load custom Python plugins** for any specialized logic

No code changes are required to add or modify communication flows—just edit the YAML and relaunch.

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

* **Single Generic Node** (`comms`): all communication flows via one executable.
* **YAML‑Driven**: define everything in `config/*.yaml`—no recompilation needed.
* **Inbound Handling**: decode arbitrary packet types to ROS topics or services.
* **Outbound Scheduling**: send any packet ID on a custom schedule.
* **Topic Triggers**: map incoming ROS topics to packet transmissions.
* **Service Bridges**: expose ROS services that encode and send packets, then optionally publish responses.
* **Parameter‑Driven**: dynamic parameters (`ros2 param set`) control packet fields, rates, etc.
* **Plugin Architecture**: drop in Python modules under `plugins/` for bespoke logic.

---

## Repository Layout

```
pez_comms/
├── config/                # YAML profiles
│   ├── fish_comms.yaml    # fish-side only
│   ├── host_comms.yaml    # host-side only
│   └── guide_config.yaml  # annotated example
├── launch/                # launch scripts
│   ├── comms_launch.py    # generic comms launcher
│   ├── fish_launch.py     # fish namespace wrapper
│   ├── host_launch.py     # host namespace wrapper
│   └── test_launch.py     # socat + dual-comms test
├── nodes/                 # generic comms node
│   └── comms_node.py
├── plugins/               # optional custom logic
│   ├── fish_side.py       # fish-side packet loop plugin
│   └── host_side.py       # host-side scheduler plugin
├── core/                  # shared libraries
│   ├── modem_io.py        # serial I/O manager
│   ├── packet_def.py      # packet encode/decode registry
│   ├── scheduler.py       # TransmissionScheduler
│   └── serial_test.py     # serial debugging tool
├── package.xml            # ROS2 package manifest
├── setup.py               # setuptools entry point for `comms`
└── resource/pez_comms     # ament index entry
```

---

## Installation

```bash
# 1. Install system dependencies on Ubuntu Jammy (ROS2 Humble)
sudo apt update && sudo apt install -y \
  python3-pip python3-colcon-common-extensions \
  python3-all-dev build-essential cmake \
  ros-humble-usb-cam i2c-tools libgpiod2 python3-rpi.gpio

# 2. Install Python libraries
pip3 install --user pyserial pyyaml bluerobotics-navigator adafruit-blinka smbus2

# 3. Build your workspace
cd ~/pez_ws
colcon build --packages-select pez_interfaces pez_comms
source install/setup.bash
```

---

## Configuration

All communication logic is declared in **YAML** under `config/`.  Key sections:

* `modem_io`: `{port, baud, timeout}`
* `parameters`: expose named ROS params for `param:...` references
* `schedules`: timed sequences of packet sends
* `triggers`: topic → packet mappings
* `publishers`: periodic ROS message publishers
* `services`: service servers that send packets and optionally publish responses
* `serial_handlers`: inbound packet types → decode + publish/service
* `plugins`: load custom Python modules

See [`guide_config.yaml`](config/guide_config.yaml) for a fully annotated example.

---

## Launch Files

* **`comms_launch.py`**: generic node launcher

  ```bash
  ros2 launch pez_comms comms_launch.py \
    config_file:=config/fish_comms.yaml port:=/dev/ttyUSB0
  ```
* **`fish_launch.py`**: wraps `comms_launch.py` under the `/fish` namespace
* **`host_launch.py`**: wraps `comms_launch.py` under the `/host` namespace
* **`test_launch.py`**: runs `socat` to link two PTYs and launches both fish+host

---

## Plugins

Drop any custom logic into `plugins/`. Each plugin must define:

```python
def register(node: rclpy.node.Node, cfg: dict):
    """
    Called by comms_node; `cfg` is the plugin-specific YAML block.
    """
    # e.g. node.create_subscription(...)
```

Example plugins:

* `fish_side.py`: inbound decode loop for Packet A/B with ACK/NACK
* `host_side.py`: outbound scheduler for Packet A and Packet B state machine

---

## Core Modules

* **`modem_io.py`**: manages a background serial thread and queue
* **`packet_def.py`**: register & lookup packet classes by ID
* **`scheduler.py`**: `TransmissionStep` + `TransmissionScheduler` for timed flows
* **`serial_test.py`**: quick serial port utility script

---

## Examples

### Fish‐side

```bash
ros2 launch pez_comms fish_launch.py \
  port:=/tmp/pez_fish
```

### Host‐side

```bash
ros2 launch pez_comms host_launch.py \
  port:=/tmp/pez_host
```

### End-to-end Test

```bash
socat -d -d pty,raw,echo=0,link=/tmp/pez_host pty,raw,echo=0,link=/tmp/pez_fish &
ros2 launch pez_comms test_launch.py
```

---

## Contributing

Please open issues or pull requests for bugs and feature requests.  New flows should be added via YAML configurations or plugins—core changes only if strictly necessary.

---

## License

Apache 2.0
