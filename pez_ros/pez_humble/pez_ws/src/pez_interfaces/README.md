# pez_interfaces

Small package containing custom ROS 2 interfaces used by the Pez project.
Currently it provides the `SnapSensors` service which returns an instantaneous
reading of all onboard sensors.

## Service

`SnapSensors.srv`
```
---
std_msgs/Header header
float64 tsys01_temp
float64 ms5837_temp
float64 ms5837_pressure
float64 ms5837_depth
bool    success
string  message
```

Use it with:
```bash
ros2 service call /snap_sensors pez_interfaces/srv/SnapSensors {}
```
