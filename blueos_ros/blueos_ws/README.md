# blueos_ws

ROS Noetic workspace used inside the BlueOS extension container
`mapuigsari/blueos-ros-server:arm32-v7-comms`.
It contains a ROS1 version of `pez_comms` and the `bluerov_server`
package to drive a BlueROV2 via MAVLink.

This setup has been tested as a bridge between a ROS Noetic vehicle
and a ROS 2 (Humble) host. Each side runs its own `pez_comms`
implementation but shares the same packet definitions, allowing
mixed‑distro communication over the acoustic modem.

Build inside the container with:
```bash
cd /blueos_ws
catkin_make
source devel/setup.bash
```
