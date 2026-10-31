#!/usr/bin/env python3
"""Record the simulated Site Robotics RGB camera to a video file and sample JPEGs.

Run with ROS 2 Jazzy sourced and ROS_DOMAIN_ID set to the robot's domain.
No bridge code or Gazebo files are changed.
"""
import argparse
from pathlib import Path
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, args):
        super().__init__('site_robotics_camera_recorder')
        self.args = args
        self.bridge = CvBridge()
        self.writer = None
        self.frames = 0
        self.first_frame_time = None
        self.last_frame_time = None
        self.next_capture = 0.0
        self.snapshots = Path(args.output).parent / 'frames'
        self.snapshots.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Image, args.topic, self.receive, qos_profile_sensor_data)
        self.get_logger().info(f'Listening to {args.topic}; output: {args.output}')

    def receive(self, msg):
        now = time.monotonic()
        if now < self.next_capture:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Dashboard applies the same 180-degree rotation for its feed.
            if not self.args.raw_orientation:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            if self.writer is None:
                height, width = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*('MJPG' if self.args.output.lower().endswith('.avi') else 'mp4v'))
                self.writer = cv2.VideoWriter(self.args.output, fourcc, self.args.fps, (width, height))
                if not self.writer.isOpened():
                    raise RuntimeError('OpenCV video writer could not open output; use a .avi output file')
                self.first_frame_time = now
                self.get_logger().info(f'Recording {width}x{height} @ {self.args.fps} fps')
            self.writer.write(frame)
            self.frames += 1
            self.last_frame_time = now
            self.next_capture = now + 1.0 / self.args.fps
            if self.frames == 1 or self.frames % (self.args.fps * self.args.snapshot_every) == 0:
                target = self.snapshots / f'camera_{self.frames:06d}.jpg'
                cv2.imwrite(str(target), frame)
            if self.frames % max(1, self.args.fps * 5) == 0:
                self.get_logger().info(f'Saved {self.frames} video frames')
        except Exception as exc:
            self.get_logger().error(f'Camera frame rejected: {exc}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/niihan/sensors/panoramic/front/image_raw')
    parser.add_argument('--output', default=str(Path.home() / 'surveillance_project' / 'recordings' / 'site_camera.mp4'))
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--duration', type=float, default=120, help='Wall-clock capture duration; Ctrl+C to finish early')
    parser.add_argument('--snapshot-every', type=int, default=5, help='Save one JPEG approximately every N video seconds')
    parser.add_argument('--raw-orientation', action='store_true', help='Do not rotate camera image as the dashboard does')
    args = parser.parse_args()
    if args.fps < 1 or args.fps > 60 or args.duration <= 0 or args.snapshot_every < 1:
        parser.error('fps 1-60, duration > 0 and snapshot-every >= 1 required')
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    args.output = str(output)
    rclpy.init()
    node = Recorder(args)
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        if node.writer:
            node.writer.release()
        node.get_logger().info(f'Completed: {node.frames} frames saved to {output}' if node.frames else
                               'NO FRAMES RECEIVED: verify camera topic publishers and ROS_DOMAIN_ID')
        node.destroy_node()
        rclpy.shutdown()
    if not node.frames:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
