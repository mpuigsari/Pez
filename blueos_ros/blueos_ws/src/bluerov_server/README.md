# bluerov_server

MAVLink bridge for BlueROV2 running under BlueOS.

This package was created at the **Research Center in Robotics and Underwater Technologies (CIRTESU)**, Universitat Jaume I. It contains a single Python script, `server.py`, that translates ROS Noetic topics and services into MAVLink RC overrides.

`server.py` subscribes to `/cmd_vel` for thruster commands and `/camara_servo` for the camera tilt PWM. It also exposes Trigger services for arming and disarming the vehicle as well as toggling the lights. All commands are forwarded over UDP to the BlueOS MAVLink endpoint using `pymavlink`.

Use this node together with `pez_comms` to drive the vehicle from acoustic packets.
