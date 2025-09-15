# pez_core/fish_sense.py

import math
from typing import Any, Callable, Dict

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Temperature, FluidPressure
from std_msgs.msg import Float32

# Import the updated service type (with a Header in the response)
from pez_interfaces.srv import SnapSensors

from pez_core.sensors import SensorManager


class SensorsNode(Node):
    """
    ROS2 node that publishes TSYS01 and MS5837 topics, with highly optimized
    dynamic reconfiguration via a dispatch table—and a new SnapSensors service
    that returns the current active measurements with a stamped header.
    """

    def __init__(self):
        super().__init__('sensors_node')

        # 1) Declare all parameters with defaults
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('publish_tsys01_temperature', True)
        self.declare_parameter('publish_ms5837_temperature', False)
        self.declare_parameter('publish_ms5837_pressure', True)
        self.declare_parameter('publish_ms5837_depth', False)
        self.declare_parameter('fluid_density', 997.0)

        # 2) Read initial parameter values into attributes
        self.pub_freq      = self.get_parameter('publish_frequency').value
        self.fluid_density = self.get_parameter('fluid_density').value

        # boolean flags for sensor toggles
        self.flags = {
            'tsys01_temp':     self.get_parameter('publish_tsys01_temperature').value,
            'ms5837_temp':     self.get_parameter('publish_ms5837_temperature').value,
            'ms5837_pressure': self.get_parameter('publish_ms5837_pressure').value,
            'ms5837_depth':    self.get_parameter('publish_ms5837_depth').value,
        }

        # 3) Build a dispatch table: param_name -> action function
        self._param_dispatch: Dict[str, Callable[[Any], None]] = {
            'publish_frequency':           lambda v: setattr(self, 'pub_freq', float(v)),
            'fluid_density':               lambda v: setattr(self, 'fluid_density', float(v)),
            'publish_tsys01_temperature':  lambda v: self._set_flag('tsys01_temp', bool(v)),
            'publish_ms5837_temperature':  lambda v: self._set_flag('ms5837_temp', bool(v)),
            'publish_ms5837_pressure':     lambda v: self._set_flag('ms5837_pressure', bool(v)),
            'publish_ms5837_depth':        lambda v: self._set_flag('ms5837_depth', bool(v)),
        }

        # 4) Define sensor_map exactly as before
        self.sensor_map = {
            'tsys01_temp': {
                'topic': 'tsys01/temperature',
                'msg_type': Temperature,
                'read_method': 'read_tsys01',
            },
            'ms5837_temp': {
                'topic': 'ms5837/temperature',
                'msg_type': Temperature,
                'read_method': 'read_ms5837_temp',
            },
            'ms5837_pressure': {
                'topic': 'ms5837/pressure',
                'msg_type': FluidPressure,
                'read_method': 'read_ms5837_pressure',
            },
            'ms5837_depth': {
                'topic': 'ms5837/depth',
                'msg_type': Float32,
                'read_method': 'read_ms5837_depth',
            },
        }

        # 5) Instantiate SensorManager for the first time
        self._init_sensor_manager()

        # 6) Create a dictionary for our own publishers (rename to avoid conflict)
        self.sensor_publishers: Dict[str, Any] = {}
        self._create_publishers()

        # 7) Create the timer based on publish_frequency
        self._create_timer()

        # 8) Log startup summary
        self.get_logger().info(self._summary_text())

        # 9) Register a single on-set callback
        self.add_on_set_parameters_callback(self._on_parameter_event)

        # ─── SNAPSHOT SERVICE SETUP ───
        self._snapshot_srv = self.create_service(
            SnapSensors,
            'get_sensor_snapshot',
            self._handle_snapshot
        )

    def _set_flag(self, key: str, new_val: bool):
        """
        Helper for toggling sensor flags. If the flag actually changes,
        mark that we need to rebuild the SensorManager and recreate publishers.
        """
        if new_val != self.flags[key]:
            self.flags[key] = new_val
            # We’ll handle the rebuild logic in _on_parameter_event
            self._need_rebuild_sm = True
            self._need_recreate_pubs = True

    def _init_sensor_manager(self):
        """
        Instantiate or re‐instantiate SensorManager if at least one flag is True.
        Called during init and whenever flags change.
        """
        use_tsys01 = self.flags['tsys01_temp']
        use_ms5837 = any(
            self.flags[k]
            for k in ('ms5837_temp', 'ms5837_pressure', 'ms5837_depth')
        )

        if not (use_tsys01 or use_ms5837):
            self.get_logger().error("No sensor magnitudes enabled; shutting down.")
            raise RuntimeError("At least one 'publish_*' parameter must be True.")

        # Close existing manager if it exists
        if hasattr(self, 'sensor_manager'):
            try:
                self.sensor_manager.close()
            except Exception as e:
                self.get_logger().warn(f"Error closing old SensorManager: {e}")

        # Create a new SensorManager
        try:
            self.sensor_manager = SensorManager(
                use_tsys01=use_tsys01,
                use_ms5837=use_ms5837
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize SensorManager: {e}")
            raise

    def _create_publishers(self):
        """
        (Re)create publishers for each sensor whose flag is True.
        Stores them in self.sensor_publishers[key] = Publisher or None.
        """
        ns = False #self.get_namespace().strip('/')
        prefix = f"{ns}/" if ns else ""

        # Clear old publishers
        self.sensor_publishers.clear()

        for key, conf in self.sensor_map.items():
            if self.flags[key]:
                topic = prefix + conf['topic']
                # This create_publisher will append into Node._publishers internally
                self.sensor_publishers[key] = self.create_publisher(conf['msg_type'], topic, 10)
            else:
                self.sensor_publishers[key] = None

    def _create_timer(self):
        """
        (Re)create the ROS timer based on publish_frequency.
        Always cancel any existing timer first.
        """
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()

        # Avoid division by zero; ensure period > 0
        period = 1.0 / max(float(self.pub_freq), 1e-6)
        self.timer = self.create_timer(period, self._timer_callback)

    def _summary_text(self) -> str:
        """
        Return a short summary of current configuration for logging.
        """
        lines = [f"SensorsNode starting with:\n  • publish_frequency: {self.pub_freq:.2f} Hz"]
        for key, conf in self.sensor_map.items():
            status = 'ON' if self.flags[key] else 'OFF'
            lines.append(f"  • {conf['topic']}: {status}")
        lines.append(f"  • fluid_density: {self.fluid_density} kg/m³")
        return "\n".join(lines)

    def _on_parameter_event(self, params):
        """
        Called whenever any declared parameter is set. We use the dispatch table
        self._param_dispatch to update internal fields. Then, if any sensor flags
        changed, we rebuild SensorManager and publishers. If frequency changed,
        we recreate the timer at the end.
        """
        # Reset the per-callback markers
        self._need_rebuild_sm = False
        self._need_recreate_pubs = False
        self._need_update_timer = False

        # 1) Loop over changed parameters, call the corresponding action
        for p in params:
            name, value = p.name, p.value
            if name in self._param_dispatch:
                # Before calling action, detect if this was publish_frequency
                if name == 'publish_frequency':
                    new_freq = float(value)
                    if new_freq > 0 and new_freq != self.pub_freq:
                        self._param_dispatch[name](value)
                        self._need_update_timer = True
                else:
                    self._param_dispatch[name](value)

        # 2) If any sensor flags changed, rebuild SensorManager once
        if self._need_rebuild_sm:
            try:
                self._init_sensor_manager()
            except RuntimeError as e:
                # No sensor enabled → shut down
                self.get_logger().error(f"Cannot reinitialize SensorManager: {e}")
                self.destroy_node()
                return SetParametersResult(successful=False)

        # 3) If flags changed, recreate publishers once
        if self._need_recreate_pubs:
            self._create_publishers()

        # 4) If frequency changed, recreate timer once
        if self._need_update_timer:
            self._create_timer()

        # 5) Log updated configuration
        self.get_logger().info("Updated SensorsNode configuration:\n" + self._summary_text())

        return SetParametersResult(successful=True)

    def _timer_callback(self):
        """
        On each timer tick, read and publish for every sensor key whose flag is True.
        """
        now = self.get_clock().now().to_msg()

        for key, conf in self.sensor_map.items():
            pub = self.sensor_publishers.get(key)
            if not (self.flags[key] and pub):
                continue

            # Dynamically call the read_... method on sensor_manager
            read_fn: Callable[..., Any] = getattr(self.sensor_manager, conf['read_method'])
            try:
                val = read_fn() if key != 'ms5837_depth' else read_fn(self.fluid_density)
            except Exception as e:
                self.get_logger().error(f"{conf['read_method']} error: {e}")
                continue

            # Construct and publish the ROS message
            if conf['msg_type'] is Temperature:
                msg = Temperature()
                msg.header.stamp = now
                msg.temperature = float(val)
                msg.variance = 0.0
                pub.publish(msg)

            elif conf['msg_type'] is FluidPressure:
                msg = FluidPressure()
                msg.header.stamp = now
                # read_fn returned mbar; convert to Pa
                msg.fluid_pressure = float(val) * 100.0
                msg.variance = 0.0
                pub.publish(msg)

            elif conf['msg_type'] is Float32:
                msg = Float32()
                msg.data = float(val)
                pub.publish(msg)

    def _handle_snapshot(self, request, response):
        """
        Service callback for SnapSensors. First stamps response.header,
        then loops over self.sensor_map keys:
        - If enabled, read via SensorManager (or NaN on error).
        - Assign each value to response.<key>.
        """
        # 1) Stamp the header with current ROS time
        now = self.get_clock().now().to_msg()
        response.header.stamp = now

        # 2) Initialize all response fields to NaN
        for key in self.sensor_map.keys():
            setattr(response, key, math.nan)

        # 3) Iterate through each sensor entry
        for key, conf in self.sensor_map.items():
            # If that sensor is disabled, skip it
            if not self.flags.get(key, False):
                continue

            # Otherwise, attempt to read via SensorManager
            try:
                if key == 'ms5837_depth':
                    # read_ms5837_depth needs fluid_density
                    val = getattr(self.sensor_manager, conf['read_method'])(self.fluid_density)
                else:
                    val = getattr(self.sensor_manager, conf['read_method'])()
            except Exception as e:
                self.get_logger().error(f"{conf['read_method']} failed: {e}")
                val = math.nan

            # Assign into the response field (named exactly as `key`)
            setattr(response, key, float(val))

        # 4) Indicate success
        response.success = True
        response.message = 'Snapshot OK'
        return response

    def destroy_node(self):
        """
        Clean up sensor_manager on shutdown.
        """
        try:
            self.sensor_manager.close()
        except Exception as e:
            self.get_logger().warn(f"Error closing SensorManager: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorsNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Fatal error in SensorsNode: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
