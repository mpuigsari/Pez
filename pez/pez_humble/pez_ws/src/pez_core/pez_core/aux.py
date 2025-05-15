# aux.py (modified)

import time
import queue

from std_msgs.msg import Float64, Float64MultiArray

class PwmChannel:
    Ch9  = 9-1    # magnet
    Ch11 = 11-1   # camera
    Ch14 = 14-1   # left fin
    Ch15 = 15-1   # right fin
    Ch16 = 16-1   # tail

class FakeNav:
    def __init__(self, node):
        """
        node: rclpy.node.Node in which to create publishers
        """
        self.node = node
        self.start = time.time()
        self._q = queue.Queue()

        # Publishers for PlotJuggler
        self.tail_pub   = node.create_publisher(Float64,           'pwm/tail',   10)
        self.fins_pub   = node.create_publisher(Float64MultiArray, 'pwm/fins',   10)
        self.camera_pub = node.create_publisher(Float64,           'pwm/camera', 10)
        self.magnet_pub = node.create_publisher(Float64,           'pwm/magnet', 10)

        msg, msg_a = Float64(), Float64MultiArray()
        self.tail_pub.publish(msg)
        self.fins_pub.publish(msg_a)
        self.camera_pub.publish(msg)
        self.magnet_pub.publish(msg)

        node.get_logger().info('[FakeNav] Ready to publish PWM topics')

    def init(self):
        """Previously printed, now a no-op or you can log."""
        self.node.get_logger().info('[FakeNav] init() called')

    def set_pwm_enable(self, e):
        """Stub if you need it."""
        self.node.get_logger().info(f'[FakeNav] PWM enable set to {e}')

    def set_pwm_freq_hz(self, f):
        """Stub if you need it."""
        self.node.get_logger().info(f'[FakeNav] PWM frequency set to {f} Hz')

    def set_pwm_channels_values(self, channels, values):
        """
        Called from your controller threads.
        Publishes each group of values to the right topic.
        """
        # Enqueue if you still need the raw data
        t = time.time() - self.start
        self._q.put((t, list(zip(channels, values))))

        # Handle tail
        if len(channels) == 1 and channels[0] == PwmChannel.Ch16:
            msg = Float64()
            msg.data = float(values[0])
            self.tail_pub.publish(msg)

        # Handle camera
        elif len(channels) == 1 and channels[0] == PwmChannel.Ch11:
            msg = Float64()
            msg.data = float(values[0])
            self.camera_pub.publish(msg)

        # Handle magnet
        elif len(channels) == 1 and channels[0] == PwmChannel.Ch9:
            msg = Float64()
            msg.data = float(values[0])
            self.magnet_pub.publish(msg)

        # Handle fins (must come as a pair [Ch14, Ch15])
        elif set(channels) == {PwmChannel.Ch14, PwmChannel.Ch15}:
            # preserve order [left, right]
            ordered = [values[channels.index(PwmChannel.Ch14)],
                       values[channels.index(PwmChannel.Ch15)]]
            msg = Float64MultiArray()
            msg.data = [float(v) for v in ordered]
            self.fins_pub.publish(msg)

    def set_pwm_channel_duty_cycle(self, channel, value):
        """
        If elsewhere you call this (e.g. for your magnet toggle),
        publish it here too.
        """
        if channel == PwmChannel.Ch9:
            msg = Float64()
            msg.data = float(value)
            self.magnet_pub.publish(msg)
        else:
            # fallback: route single-channel calls
            self.set_pwm_channels_values([channel], [value])
