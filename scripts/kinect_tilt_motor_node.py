#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


VID = 0x045e
PID = 0x02b0


class KinectTiltMotorNode(Node):
    """Mirror the ROS head tilt into the physical Kinect v1 tilt motor."""

    def __init__(self):
        super().__init__('kinect_tilt_motor_node')

        self.declare_parameter('head_pose_topic', '/hardware/head/current_pose')
        self.declare_parameter('tilt_index', 1)
        self.declare_parameter('invert_tilt', False)
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('angle_scale_deg_per_rad', 180.0 / math.pi)
        self.declare_parameter('min_angle_deg', -31.0)
        self.declare_parameter('max_angle_deg', 31.0)
        self.declare_parameter('min_delta_deg', 1.0)
        self.declare_parameter('command_rate_hz', 10.0)
        self.declare_parameter('usb_retry_period_sec', 2.0)

        self.usb = None
        self.dev = None
        self.last_command_deg = None
        self.target_angle_deg = None
        self.last_connect_attempt = self.get_clock().now()

        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter('head_pose_topic').value),
            self.head_pose_callback,
            10,
        )

        rate_hz = max(1.0, float(self.get_parameter('command_rate_hz').value))
        self.create_timer(1.0 / rate_hz, self.command_timer_callback)

        self.get_logger().info(
            f"Kinect tilt motor node listening to "
            f"{self.get_parameter('head_pose_topic').value}"
        )

    def head_pose_callback(self, msg):
        tilt_index = int(self.get_parameter('tilt_index').value)
        if len(msg.data) <= tilt_index:
            self.get_logger().warn(
                f'Head pose message has {len(msg.data)} elements, '
                f'cannot read tilt index {tilt_index}'
            )
            return

        tilt_rad = float(msg.data[tilt_index])
        if bool(self.get_parameter('invert_tilt').value):
            tilt_rad = -tilt_rad

        angle_deg = (
            tilt_rad * float(self.get_parameter('angle_scale_deg_per_rad').value)
            + float(self.get_parameter('angle_offset_deg').value)
        )
        self.target_angle_deg = self.clamp(
            angle_deg,
            float(self.get_parameter('min_angle_deg').value),
            float(self.get_parameter('max_angle_deg').value),
        )

    def command_timer_callback(self):
        if self.target_angle_deg is None:
            return

        if self.last_command_deg is not None:
            min_delta = float(self.get_parameter('min_delta_deg').value)
            if abs(self.target_angle_deg - self.last_command_deg) < min_delta:
                return

        if not self.ensure_connected():
            return

        try:
            self.set_motor_angle(self.target_angle_deg)
            self.last_command_deg = self.target_angle_deg
            self.get_logger().debug(
                f'Kinect tilt target {self.target_angle_deg:.2f} deg')
        except Exception as exc:
            self.get_logger().warn(f'Kinect motor command failed: {exc}')
            self.cleanup_device()

    def ensure_connected(self):
        if self.dev is not None:
            return True

        now = self.get_clock().now()
        retry_period = float(self.get_parameter('usb_retry_period_sec').value)
        if (now - self.last_connect_attempt).nanoseconds < retry_period * 1e9:
            return False
        self.last_connect_attempt = now

        try:
            import usb.core
            import usb.util
        except ImportError:
            self.get_logger().warn(
                'pyusb is not installed; install python3-usb to move the Kinect motor')
            return False

        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            self.get_logger().warn('Kinect motor USB device not found')
            return False

        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
            usb.util.claim_interface(dev, 0)
        except Exception as exc:
            self.get_logger().warn(f'Could not claim Kinect motor interface: {exc}')
            return False

        self.usb = usb
        self.dev = dev
        self.last_command_deg = None
        self.get_logger().info('Kinect motor USB device connected')
        return True

    def set_motor_angle(self, angle_deg):
        motor_value = int(round(angle_deg * 2.0)) & 0xffff
        self.dev.ctrl_transfer(0x40, 0x31, motor_value, 0, None, 1000)

    def cleanup_device(self):
        if self.dev is None or self.usb is None:
            self.dev = None
            self.usb = None
            return

        try:
            self.usb.util.release_interface(self.dev, 0)
            self.usb.util.dispose_resources(self.dev)
        except Exception:
            pass
        self.dev = None
        self.usb = None

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def destroy_node(self):
        self.cleanup_device()
        super().destroy_node()


def main():
    rclpy.init()
    node = KinectTiltMotorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
