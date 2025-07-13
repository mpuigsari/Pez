#!/usr/bin/env python3
"""
ros2_bag_to_mp4.py  
Subscribe to a sensor_msgs/msg/CompressedImage topic (e.g. from a rosbag2
player) and save the stream into an MP4 file.

USAGE EXAMPLES
--------------
1. Play a bag in another terminal, then run the recorder:
     ros2 bag play my_bag
     python3 ros2_bag_to_mp4.py --topic /camera/image/compressed --output camera.mp4 --fps 30

2. Let the script launch the bag for you:
     python3 ros2_bag_to_mp4.py --bag my_bag --topic /camera/image/compressed --output camera.mp4 --fps 30

The script uses OpenCV for decoding/encoding frames and rclpy for ROS 2.
Install dependencies on Ubuntu/Debian:
     sudo apt install python3-opencv
     pip install --user rclpy

Tested with ROS 2 Humble / Iron but should work with any recent distro.
"""

import argparse
import os
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class VideoRecorder(Node):
    """Subscribe to a compressed‑image topic and write frames to an MP4 file."""

    def __init__(self, topic: str, output: str, fps: float):
        super().__init__('bag_to_mp4_recorder')
        self._output_path = output
        self._fps = fps
        self._writer = None

        self.create_subscription(
            CompressedImage, topic, self._callback, 10)
        self.get_logger().info(
            f'Waiting for first frame on "{topic}" to create video writer…')

    # ---------------------------------------------------------------------
    # ROS subscription callback
    # ---------------------------------------------------------------------
    def _callback(self, msg: CompressedImage) -> None:
        # Decode JPEG/PNG stream in memory to a BGR image
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed to decode a frame — skipping')
            return

        # Lazy‑initialise VideoWriter when the first frame arrives
        if self._writer is None:
            h, w, _ = frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # H.264/MPEG‑4 AVC
            self._writer = cv2.VideoWriter(self._output_path, fourcc,
                                           self._fps, (w, h))
            if not self._writer.isOpened():
                self.get_logger().error(
                    f'Could not open VideoWriter for {self._output_path}')
                rclpy.shutdown()
                return
            self.get_logger().info(
                f'Started recording {w}×{h}@{self._fps} FPS → {self._output_path}')

        self._writer.write(frame)

    # ------------------------------------------------------------------
    # Clean‑up
    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self.get_logger().info(f'Saved file {self._output_path}')
        super().destroy_node()


# ============================================================================
# main()
# ============================================================================

def main(argv=None):
    parser = argparse.ArgumentParser(description='Record a compressed‑image '
                                                 'topic to MP4.')
    parser.add_argument('--topic', default='/camera/image/compressed',
                        help='ROS 2 topic of type sensor_msgs/msg/CompressedImage')
    parser.add_argument('--output', default='output.mp4',
                        help='Output MP4 filename')
    parser.add_argument('--fps', type=float, default=30.0,
                        help='Frame rate for the saved video')
    parser.add_argument('--bag', help='(Optional) Path to rosbag2 directory to play')
    parser.add_argument('--bag-play-args', default='',
                        help='Extra args forwarded to "ros2 bag play"')

    args = parser.parse_args(argv)

    # Launch rosbag2 player in a child process if requested ----------------
    bag_proc = None
    if args.bag:
        play_cmd = ['ros2', 'bag', 'play', args.bag]
        if args.bag_play_args:
            play_cmd.extend(args.bag_play_args.split())
        bag_proc = subprocess.Popen(play_cmd, preexec_fn=os.setsid)
        # Give the player a moment to start publishing before we subscribe
        time.sleep(1.0)

    # ------------------------------------------------------------------
    rclpy.init(args=None)
    recorder = VideoRecorder(args.topic, args.output, args.fps)

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

        # Terminate rosbag player gracefully
        if bag_proc is not None:
            os.killpg(os.getpgid(bag_proc.pid), signal.SIGINT)
            bag_proc.wait()


if __name__ == '__main__':
    main()
