# blueos_ros

This directory provides the ROS Noetic implementation for operating a BlueROV2 from BlueOS while communicating over Tritech MicronModem acoustic links.  It contains the `pez_comms` package for modem handling and the `bluerov_server` package that bridges received velocity commands into MAVLink RC overrides.  Docker files exist under `blueos_ros_extension`, but containerisation is still work in progress and omitted here.

---

## Directory layout

```
blueos_ros/
├── pez_comms/          # generic serial communications node
├── bluerov_server/     # MAVLink RC override server
└── blueos_ros_extension/ (WIP Docker setup)
```

---

## pez_comms
`pez_comms` exposes a single node defined in [`comms_node.py`](./pez_comms/src/pez_comms/nodes/comms_node.py).  The node is entirely driven by a YAML configuration file so that packet formats, schedules and topic mappings can be changed without code edits.  At start-up the node reads the file specified in `comms.launch.xml` and configures the modem and all handlers.

Key features of the node include:

* Serial handlers that decode packets and publish ROS messages
* Timed transmission schedules
* Topic and service triggered packets
* Optional plugin hooks for custom logic

Relevant code lines show the engine setup and serial thread start:
```
 23 from pez_comms.core.modem_io import ModemIOMgr
 24 from pez_comms.core.packet_def import get_packet
 25 from pez_comms.core.scheduler import TransmissionStep, TransmissionScheduler
 ...
 65 self._setup_schedules(schedules)
 68 self._setup_triggers(triggers)
 71 self._setup_publishers(publishers)
 73 self._setup_services(services)
 75 self._setup_serial_handlers(serial_handlers)
 ...
 85 if self._serial_handlers:
 86     self._stop_event = threading.Event()
 87     self._serial_thread = threading.Thread(
 88         target=self._serial_loop, daemon=True
 89     )
 90     self._serial_thread.start()
```
【F:blueos_ros/pez_comms/src/pez_comms/nodes/comms_node.py†L23-L90】

The default configuration [`bluerov_config.yaml`](./pez_comms/config/bluerov_config.yaml) polls `PacketBlueRov` once per second and publishes the decoded fields as a `geometry_msgs/Twist` message:
```
 19 - name: bluerov_poll
 20   loop: true
 21   steps:
 22     - name: poll
 23       duration: 1.0
 24       encode:
 25         type: PacketBlueRov
 26         seq:         0
 27         vx:          0.0
 28         vy:          0.0
 29         vz:          0.0
 30         wz:          0.0
 31         svc_pending: 0
```
【F:blueos_ros/pez_comms/config/bluerov_config.yaml†L19-L31】
and maps the fields to `/cmd_vel`:
```
 39 - packet: PacketBlueRov
 40   publish:
 41     topic: "/cmd_vel"
 42     type:  "geometry_msgs/msg/Twist"
 43     mapping:
 44       linear.x:   "vx"
 45       linear.y:   "vy"
 46       linear.z:   "vz"
 47       angular.z:  "wz"
```
【F:blueos_ros/pez_comms/config/bluerov_config.yaml†L39-L47】

Launch the node with:
```bash
roslaunch pez_comms comms.launch.xml
```
It will open the serial port described in the YAML and begin sending/receiving packets.

---

## bluerov_server
`bluerov_server` provides a simple Python script [`server.py`](./bluerov_server/scripts/server.py) that converts the `/cmd_vel` messages from `pez_comms` into RC override commands for the BlueROV2 via `pymavlink`.  The script also exposes topics for camera servo control, arm/disarm and lighting.

The helper used to send RC values can be seen around these lines:
```
 68  rc_channel_values = [65535 for _ in range(18)]
 69  rc_channel_values[channel_id - 1] = pwm
 70  master.mav.rc_channels_override_send(
 71      master.target_system,
 72      master.target_component,
 73      *rc_channel_values)
```
【F:blueos_ros/bluerov_server/scripts/server.py†L68-L75】

Subscriptions are set up in `listener()`:
```
 189  rospy.init_node('cmd_vel_subscriber', anonymous=True)
 191  rospy.Subscriber('cmd_vel', Twist, callback)
 192  rospy.Subscriber('camara_servo', Int32, callback_camara_servo)
 193  rospy.Subscriber('arm_disarm', Bool, callback_armar)
 194  rospy.Subscriber('luces_pwm', Int32, callback_luces_pwm)
```
【F:blueos_ros/bluerov_server/scripts/server.py†L189-L195】
Each callback converts the received command into the appropriate RC channel override, enabling thruster control, camera tilt and light brightness.

---

## Teleoperation pipeline
1. `comms_node.py` reads and writes acoustic packets on the serial link to the MicronModem.
2. Decoded velocity data is published on `/cmd_vel`, while start/stop services are available as standard ROS services.
3. `bluerov_server` subscribes to those topics and issues MAVLink RC overrides to the vehicle.
4. Video from the onboard camera can be viewed using any ROS image viewer (e.g. `rqt_image_view`) by subscribing to the corresponding camera topic provided by BlueOS.

---

## Usage
Ensure ROS Noetic and MAVROS are installed. Launch the communications node with your desired YAML configuration and run the RC bridge:
```bash
roslaunch pez_comms comms.launch.xml     # starts CommsFullNode
rosrun bluerov_server server.py          # converts /cmd_vel to RC overrides
```
Open an image viewer for the camera topic and teleoperate the vehicle over the acoustic channel.

---

This README summarises the current ROS Noetic code base.  Docker support under `blueos_ros_extension` is under development and therefore not covered here.
