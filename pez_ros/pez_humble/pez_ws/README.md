# pez_ws – ROS 2 Humble Workspace

This workspace aggregates all ROS 2 packages used in the Pez project. It is mounted inside the Docker containers and can also be built natively.

## Packages
- **pez_core** – teleoperation nodes, sensor integration and actuator control.
- **pez_comms** – YAML driven serial communication with acoustic modems.
- **pez_interfaces** – custom service definitions.

## Build

```bash
cd pez_ros/pez_humble/pez_ws
colcon build
source install/setup.bash
```

Both the fish‑side and host‑side Docker images mount this workspace at `/pez_ws`.
