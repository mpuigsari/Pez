# pez_core/fish_sense.py

import time
from typing import Any, Callable, Dict

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Temperature, FluidPressure
from std_msgs.msg import Float32
from rclpy.parameter import Parameter

from pez_core.sensors import SensorManager


class SensorsNode(Node):
    """
    ROS 2 node that publishes sensor topics for TSYS01 and MS5837, with full parameter-based toggling and 
    dynamic reconfiguration.

    Declared parameters (defaults):
      • publish_frequency           (float, Hz; default=1.0)
      • publish_tsys01_temperature  (bool; default=True)
      • publish_ms5837_temperature  (bool; default=False)
      • publish_ms5837_pressure     (bool; default=True)
      • publish_ms5837_depth        (bool; default=False)
      • fluid_density               (float; default=997.0)
    """

    def __init__(self):
        super().__init__('sensors_node')

        # --- 1) Declare parameters ---
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('publish_tsys01_temperature', True)
        self.declare_parameter('publish_ms5837_temperature', False)
        self.declare_parameter('publish_ms5837_pressure', True)
        self.declare_parameter('publish_ms5837_depth', False)
        self.declare_parameter('fluid_density', 997.0)

        # --- 2) Read parameter values into attributes ---
        self.pub_freq = self.get_parameter('publish_frequency').value
        self.fluid_density = self.get_parameter('fluid_density').value

        # boolean flags: will update on param changes
        self.flags = {
            'tsys01_temp': self.get_parameter('publish_tsys01_temperature').value,
            'ms5837_temp': self.get_parameter('publish_ms5837_temperature').value,
            'ms5837_pressure': self.get_parameter('publish_ms5837_pressure').value,
            'ms5837_depth': self.get_parameter('publish_ms5837_depth').value,
        }

        # --- 3) Build sensor‐to‐publisher/read mapping ---
        # Each entry defines:
        #   • param key (in self.flags)
        #   • topic suffix  (appended to namespace)
        #   • ROS message class
        #   • SensorManager method name to call
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
                # note: FluidPressure.fluid_pressure expects Pascals; conversion applied below
            },
            'ms5837_depth': {
                'topic': 'ms5837/depth',
                'msg_type': Float32,
                'read_method': 'read_ms5837_depth',
            },
        }

        # --- 4) Instantiate SensorManager with initial flags ---
        self._init_sensor_manager()

        # --- 5) Create publishers based on flags ---
        self.publishers: Dict[str, Any] = {}
        self._create_publishers()

        # --- 6) Create timer using publish_frequency ---
        self._create_timer()

        # --- 7) Log startup summary ---
        self.get_logger().info(self._summary_text())

        # --- 8) Register parameter change callback ---
        self.add_on_set_parameters_callback(self._on_parameter_event)

    def _init_sensor_manager(self):
        """Instantiate (or re-instantiate) SensorManager if at least one flag is True."""
        use_tsys01 = self.flags['tsys01_temp']
        use_ms5837 = any(self.flags[k] for k in ('ms5837_temp', 'ms5837_pressure', 'ms5837_depth'))

        if not (use_tsys01 or use_ms5837):
            self.get_logger().error("No sensor magnitudes enabled; shutting down.")
            raise RuntimeError("At least one 'publish_*' parameter must be True.")

        # If a previous sensor_manager exists, close it first
        if hasattr(self, 'sensor_manager'):
            try:
                self.sensor_manager.close()
            except Exception as e:
                self.get_logger().warn(f"Error closing existing SensorManager: {e}")

        # Instantiate new manager
        try:
            self.sensor_manager = SensorManager(use_tsys01=use_tsys01, use_ms5837=use_ms5837)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize SensorManager: {e}")
            raise

    def _create_publishers(self):
        """(Re)create ROS publishers based on current flags."""
        ns = self.get_namespace().strip('/')  # e.g. 'pez' or '' if root
        base = f"{ns}/" if ns else ""

        # Cancel any existing publishers
        self.publishers.clear()

        # Create a publisher for each enabled sensor in sensor_map
        for key, conf in self.sensor_map.items():
            if self.flags[key]:
                topic = base + conf['topic']
                self.publishers[key] = self.create_publisher(conf['msg_type'], topic, 10)
            else:
                self.publishers[key] = None

    def _create_timer(self):
        """(Re)create the ROS timer based on current publish_frequency."""
        # If there is a timer, cancel it
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()

        # Compute period (avoid division by zero)
        period = 1.0 / max(float(self.pub_freq), 1e-6)
        self.timer = self.create_timer(period, self._timer_callback)

    def _summary_text(self) -> str:
        """Return a multiline string summarizing current configuration."""
        lines = [f"SensorsNode starting with:\n  • publish_frequency: {self.pub_freq:.2f} Hz"]
        for key, conf in self.sensor_map.items():
            status = 'ON' if self.flags[key] else 'OFF'
            lines.append(f"  • {conf['topic']}: {status}")
        lines.append(f"  • fluid_density: {self.fluid_density} kg/m³")
        return "\n".join(lines)

    def _on_parameter_event(self, params):
        """
        Dynamic reconfiguration callback: reacts to any declared parameter change.
        Returns SetParametersResult(successful=True) if all OK, otherwise False.
        """
        # Track which high-level flags changed
        rebuild_sm = False
        recreate_pubs = False
        update_timer = False

        for p in params:
            name = p.name
            # Update publish_frequency
            if name == 'publish_frequency':
                new_val = float(p.value)
                if new_val > 0 and new_val != self.pub_freq:
                    self.pub_freq = new_val
                    update_timer = True

            # Update fluid_density
            elif name == 'fluid_density':
                self.fluid_density = float(p.value)

            # Update any of the sensor toggles
            elif name in (
                'publish_tsys01_temperature',
                'publish_ms5837_temperature',
                'publish_ms5837_pressure',
                'publish_ms5837_depth'
            ):
                # Map param name → internal key
                key_map = {
                    'publish_tsys01_temperature': 'tsys01_temp',
                    'publish_ms5837_temperature': 'ms5837_temp',
                    'publish_ms5837_pressure': 'ms5837_pressure',
                    'publish_ms5837_depth': 'ms5837_depth',
                }
                key = key_map[name]
                new_flag = bool(p.value)
                if new_flag != self.flags[key]:
                    self.flags[key] = new_flag
                    rebuild_sm = True
                    recreate_pubs = True

        # Rebuild SensorManager if needed
        if rebuild_sm:
            try:
                self._init_sensor_manager()
            except RuntimeError as e:
                self.get_logger().error(f"Cannot reinitialize SensorManager: {e}")
                # If no sensors enabled, shut down
                self.destroy_node()
                return SetParametersResult(successful=False)

        # Recreate publishers if toggles changed
        if recreate_pubs:
            self._create_publishers()

        # Update timer if frequency changed
        if update_timer:
            self._create_timer()

        # Log new config
        self.get_logger().info("Updated SensorsNode configuration:\n" + self._summary_text())

        return SetParametersResult(successful=True)

    def _timer_callback(self):
        """Called each tick: read each enabled sensor and publish its message."""
        now = self.get_clock().now().to_msg()

        # Iterate over each sensor key in sensor_map
        for key, conf in self.sensor_map.items():
            pub = self.publishers.get(key)
            if not (self.flags[key] and pub):
                continue

            read_fn: Callable[..., Any] = getattr(self.sensor_manager, conf['read_method'])
            try:
                val = read_fn() if key != 'ms5837_depth' else read_fn(self.fluid_density)
            except Exception as e:
                self.get_logger().error(f"{conf['read_method']} error: {e}")
                continue

            # Construct message
            if conf['msg_type'] is Temperature:
                msg = Temperature()
                msg.header.stamp = now
                msg.temperature = float(val)
                msg.variance = 0.0
                pub.publish(msg)

            elif conf['msg_type'] is FluidPressure:
                # read_fn returned mbar; convert to Pa
                msg = FluidPressure()
                msg.header.stamp = now
                msg.fluid_pressure = float(val) * 100.0
                msg.variance = 0.0
                pub.publish(msg)

            elif conf['msg_type'] is Float32:
                msg = Float32()
                msg.data = float(val)
                pub.publish(msg)

    def destroy_node(self):
        """Clean up sensor_manager on shutdown."""
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
