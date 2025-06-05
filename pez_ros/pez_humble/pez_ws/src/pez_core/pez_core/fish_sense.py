# pez_core/nodes/sensors_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, FluidPressure
from std_msgs.msg import Float32

from pez_core.sensors import SensorManager

class SensorsNode(Node):
    """
    ROS 2 node that publishes the following topics (each controlled by its own parameter):
      • tsys01/temperature        (sensor_msgs/Temperature)
      • ms5837/temperature        (sensor_msgs/Temperature)
      • ms5837/pressure           (sensor_msgs/FluidPressure)
      • ms5837/depth              (std_msgs/Float32)

    Parameters (all declared in __init__, override via CLI or YAML):
      • publish_frequency           (float, Hz; default=1.0)
      • publish_tsys01_temperature  (bool; default=True)
      • publish_ms5837_temperature  (bool; default=True)
      • publish_ms5837_pressure     (bool; default=True)
      • publish_ms5837_depth        (bool; default=True)
      • fluid_density               (float; default=997.0)
    """
    def __init__(self):
        super().__init__('sensors_node')

        # 1) Declare all parameters with defaults
        self.declare_parameter('publish_frequency', 1.0)

        # TSYS01 toggle:
        self.declare_parameter('publish_tsys01_temperature', True)

        # MS5837 toggles (each magnitude separately):
        self.declare_parameter('publish_ms5837_temperature', False)
        self.declare_parameter('publish_ms5837_pressure', True)
        self.declare_parameter('publish_ms5837_depth', False)

        # Fluid density (for depth calculation), only needed if depth is True
        self.declare_parameter('fluid_density', 997.0)

        # 2) Read parameter values into member variables
        self.pub_freq                   = self.get_parameter('publish_frequency').value
        self.pub_tsys01_temp            = self.get_parameter('publish_tsys01_temperature').value
        self.pub_ms5837_temp            = self.get_parameter('publish_ms5837_temperature').value
        self.pub_ms5837_pressure        = self.get_parameter('publish_ms5837_pressure').value
        self.pub_ms5837_depth           = self.get_parameter('publish_ms5837_depth').value
        self.fluid_density              = self.get_parameter('fluid_density').value

        # 3) Instantiate SensorManager only if at least one magnitude per sensor is requested
        use_tsys01 = bool(self.pub_tsys01_temp)
        use_ms5837 = bool(self.pub_ms5837_temp or
                          self.pub_ms5837_pressure or
                          self.pub_ms5837_depth)

        if not (use_tsys01 or use_ms5837):
            self.get_logger().error("No sensor magnitudes enabled; node will shut down.")
            raise RuntimeError("At least one 'publish_*' parameter must be True.")

        try:
            self.sensor_manager = SensorManager(
                use_tsys01=use_tsys01,
                use_ms5837=use_ms5837
            )
        except Exception as e:
            self.get_logger().error(f"SensorManager init failed: {e}")
            raise

        # 4) Create one publisher per enabled magnitude
        if self.pub_tsys01_temp:
            self.tsys_pub = self.create_publisher(
                Temperature,
                'tsys01/temperature',
                10
            )
        else:
            self.tsys_pub = None

        if self.pub_ms5837_temp:
            self.ms_temp_pub = self.create_publisher(
                Temperature,
                'ms5837/temperature',
                10
            )
        else:
            self.ms_temp_pub = None

        if self.pub_ms5837_pressure:
            self.ms_pres_pub = self.create_publisher(
                FluidPressure,
                'ms5837/pressure',
                10
            )
        else:
            self.ms_pres_pub = None

        if self.pub_ms5837_depth:
            self.ms_depth_pub = self.create_publisher(
                Float32,
                'ms5837/depth',
                10
            )
        else:
            self.ms_depth_pub = None

        # 5) Create the timer based on publish_frequency
        timer_period = 1.0 / float(self.pub_freq)
        self.timer = self.create_timer(timer_period, self._timer_callback)

        # 6) Log startup summary
        self.get_logger().info(
            "SensorsNode starting with:\n"
            f"  • publish_frequency: {self.pub_freq} Hz\n"
            f"  • TSYS01 temperature: {'ON' if self.pub_tsys01_temp else 'OFF'}\n"
            f"  • MS5837 temperature: {'ON' if self.pub_ms5837_temp else 'OFF'}\n"
            f"  • MS5837 pressure:    {'ON' if self.pub_ms5837_pressure else 'OFF'}\n"
            f"  • MS5837 depth:       {'ON' if self.pub_ms5837_depth else 'OFF'}\n"
            f"  • fluid_density:      {self.fluid_density} kg/m³"
        )

    def _timer_callback(self):
        now = self.get_clock().now().to_msg()

        # 1) TSYS01 temperature
        if self.pub_tsys01_temp and self.tsys_pub:
            try:
                temp_c = self.sensor_manager.read_tsys01()
                msg = Temperature()
                msg.header.stamp = now
                msg.temperature = temp_c
                msg.variance = 0.0
                self.tsys_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"TSYS01 read error: {e}")

        # 2) MS5837 temperature
        if self.pub_ms5837_temp and self.ms_temp_pub:
            try:
                temp_c = self.sensor_manager.read_ms5837_temp()
                msg_t = Temperature()
                msg_t.header.stamp = now
                msg_t.temperature = temp_c
                msg_t.variance = 0.0
                self.ms_temp_pub.publish(msg_t)
            except Exception as e:
                self.get_logger().error(f"MS5837 temperature error: {e}")

        # 3) MS5837 pressure
        if self.pub_ms5837_pressure and self.ms_pres_pub:
            try:
                pressure_mbar = self.sensor_manager.read_ms5837_pressure()
                msg_p = FluidPressure()
                msg_p.header.stamp = now
                # Convert mbar → Pa
                msg_p.fluid_pressure = pressure_mbar * 100.0
                msg_p.variance = 0.0
                self.ms_pres_pub.publish(msg_p)
            except Exception as e:
                self.get_logger().error(f"MS5837 pressure error: {e}")

        # 4) MS5837 depth
        if self.pub_ms5837_depth and self.ms_depth_pub:
            try:
                depth_m = self.sensor_manager.read_ms5837_depth(self.fluid_density)
                msg_d = Float32()
                msg_d.data = float(depth_m)
                self.ms_depth_pub.publish(msg_d)
            except Exception as e:
                self.get_logger().error(f"MS5837 depth error: {e}")

    def destroy_node(self):
        # Cleanly close sensors before shutting down
        try:
            self.sensor_manager.close()
        except Exception as e:
            self.get_logger().warn(f"Error closing sensors: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SensorsNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Fatal error in SensorsNode: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
