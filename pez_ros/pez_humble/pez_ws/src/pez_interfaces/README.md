# `pez_interfaces`

## SnapSensors Service

`pez_interfaces` defines custom ROS 2 service messages for the Pez project. The primary service is **SnapSensors**, used to fetch an instantaneous reading from all enabled sensors.

The service is created in `fish_sense.py` within `pez_core` and exposed as `get_sensor_snapshot`. When called, the node reads the active sensors and returns their latest values along with a timestamp.

### `.srv` Definition

```text
# SnapSensors.srv
---
std_msgs/Header header
float64 tsys01_temp
float64 ms5837_temp
float64 ms5837_pressure
float64 ms5837_depth
bool    success
string  message
```

* `header` – time the snapshot was taken
* `tsys01_temp` – TSYS01 temperature (°C)
* `ms5837_temp` – MS5837 temperature (°C)
* `ms5837_pressure` – pressure from MS5837 (mbar → Pa when published)
* `ms5837_depth` – depth derived from the MS5837 (m)
* `success` – indicates successful reads
* `message` – optional status text

Clients such as `fish_joy.py` call `get_sensor_snapshot` to log these readings or save them when specific events occur.

### Building in a ROS 2 Workspace

`SnapSensors` is generated alongside the other Pez packages. Clone the repository into `~/pez_ws/src` and build with `colcon`:

```bash
cd ~/pez_ws/src
git clone https://github.com/mpuigsari/Pez pez_core
git clone https://github.com/mpuigsari/Pez pez_comms
cd ~/pez_ws
colcon build
source install/setup.bash
```

After building, you can import the service via `from pez_interfaces.srv import SnapSensors` in your ROS 2 nodes.
