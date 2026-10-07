#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from arms_interfaces.srv import MoveArms
from head_interfaces.srv import MoveHead
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
from torso_interfaces.srv import MoveTorso


class SimpleJustinaSim(Node):
    """Minimal base simulator for navigation tests in RViz.

    It integrates /hardware/base/cmd_vel, publishes odom -> base_link,
    publishes fake joint states so robot_state_publisher can build the URDF
    tree, and provides a simulated laser scan. When /navigation/map is
    available, the laser is raycast against occupied map cells; otherwise it
    publishes a clear scan.
    """

    def __init__(self):
        super().__init__('simple_justina_sim')

        self.declare_parameter('rate', 30.0)
        self.declare_parameter('initial_x', 6.5)
        self.declare_parameter('initial_y', 3.2)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('map_topic', '/navigation/map')
        self.declare_parameter('laser_beams', 361)
        self.declare_parameter('laser_fov_deg', 180.0)
        self.declare_parameter('laser_range_min', 0.06)
        self.declare_parameter('laser_range_max', 6.0)
        self.declare_parameter('laser_obstacle_threshold', 50)
        self.declare_parameter('laser_unknown_is_obstacle', False)
        self.declare_parameter('laser_publish_max_range', False)
        self.declare_parameter('laser_x_offset', 0.06)
        self.declare_parameter('laser_y_offset', 0.0)
        self.declare_parameter('laser_yaw_offset', 0.0)
        self.declare_parameter('republish_initialpose_to_navigation', True)
        self.declare_parameter('arm_motion_min_duration_sec', 0.08)
        self.declare_parameter('arm_motion_max_duration_sec', 1.5)
        self.declare_parameter('arm_motion_reference_speed', 50.0)
        self.declare_parameter('arm_motion_reference_joint_velocity', 0.8)
        self.declare_parameter('arm_motion_steps', 16)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.cmd = Twist()
        self.wheel_positions = [0.0, 0.0, 0.0]
        self.torso_position = 0.301
        self.head_pan = 0.0
        self.head_tilt = 0.0
        self.left_arm_positions = [0.0] * 6
        self.right_arm_positions = [0.0] * 6
        self.map_msg = None
        self.map_origin_yaw = 0.0
        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.map_to_odom_yaw = 0.0
        self.pending_navigation_initial_pose = None
        self.initial_pose_republish_timer = None

        self.create_subscription(Twist, '/hardware/base/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float64MultiArray, '/hardware/head/goal_pose',
                                 self.head_goal_callback, 10)
        self.create_subscription(Float64MultiArray, '/hardware/torso/goal_pose',
                                 self.torso_goal_callback, 10)
        self.create_subscription(Float64MultiArray, '/hardware/arms/left_arm/goal_pose',
                                 self.left_arm_goal_callback, 10)
        self.create_subscription(Float64MultiArray, '/hardware/arms/right_arm/goal_pose',
                                 self.right_arm_goal_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.initial_pose_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/navigation/initialpose',
                                 self.navigation_initial_pose_callback, 10)
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self.map_callback,
            map_qos,
        )

        self.navigation_initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/navigation/initialpose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/hardware/base/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/hardware/joint_states', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/hardware/base/scan', 10)
        self.head_pose_pub = self.create_publisher(
            Float64MultiArray, '/hardware/head/current_pose', 10)
        self.torso_pose_pub = self.create_publisher(
            Float64MultiArray, '/hardware/torso/current_pose', 10)
        self.left_arm_pose_pub = self.create_publisher(
            Float64MultiArray, '/hardware/arms/left_arm/current_pose', 10)
        self.right_arm_pose_pub = self.create_publisher(
            Float64MultiArray, '/hardware/arms/right_arm/current_pose', 10)
        self.create_service(MoveHead, '/hardware/head/move_head', self.move_head_callback)
        self.create_service(MoveTorso, '/hardware/torso/move_torso',
                            self.move_torso_callback)
        self.create_service(MoveArms, '/hardware/arms/move_left_arm',
                            self.move_left_arm_callback)
        self.create_service(MoveArms, '/hardware/arms/move_right_arm',
                            self.move_right_arm_callback)
        self.tf_broadcaster = TransformBroadcaster(self)

        rate = float(self.get_parameter('rate').value)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Simple Justina simulator ready on /hardware/base/cmd_vel')

    def cmd_vel_callback(self, msg):
        self.cmd = msg

    def head_goal_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn('Received head goal pose with less than 2 elements')
            return
        self.set_head_pose(msg.data[0], msg.data[1])

    def torso_goal_callback(self, msg):
        if len(msg.data) < 1:
            self.get_logger().warn('Received torso goal pose with no position')
            return
        self.set_torso_position(msg.data[0])

    def left_arm_goal_callback(self, msg):
        self.set_arm_pose('left', msg.data)

    def right_arm_goal_callback(self, msg):
        self.set_arm_pose('right', msg.data)

    def move_head_callback(self, request, response):
        if len(request.positions) < 2:
            response.success = False
            response.message = 'MoveHead requires [pan, tilt]'
            return response

        self.set_head_pose(request.positions[0], request.positions[1])
        response.success = True
        response.message = (
            f'Head target set to pan={self.head_pan:.3f}, tilt={self.head_tilt:.3f}')
        return response

    def move_torso_callback(self, request, response):
        self.set_torso_position(request.position)
        response.success = True
        response.message = f'Torso target set to {self.torso_position:.3f}'
        return response

    def move_left_arm_callback(self, request, response):
        response.success, response.message = self.move_arm_pose(
            'left', request.positions, request.speed)
        return response

    def move_right_arm_callback(self, request, response):
        response.success, response.message = self.move_arm_pose(
            'right', request.positions, request.speed)
        return response

    def set_head_pose(self, pan, tilt):
        self.head_pan = self.clamp(float(pan), -1.57, 1.57)
        self.head_tilt = self.clamp(float(tilt), -0.9, 0.47)

    def set_torso_position(self, position):
        self.torso_position = self.clamp(float(position), 0.0, 0.3014)

    def set_arm_pose(self, arm, positions):
        if len(positions) != 6:
            return False, f'{arm} arm requires 6 joint positions'

        clamped = [self.clamp(float(position), -1.57, 1.57)
                   for position in positions]
        if arm == 'left':
            self.left_arm_positions = clamped
        else:
            self.right_arm_positions = clamped
        return True, f'{arm} arm target accepted'

    def move_arm_pose(self, arm, positions, speed):
        if len(positions) != 6:
            return False, f'{arm} arm requires 6 joint positions'

        goal = [self.clamp(float(position), -1.57, 1.57)
                for position in positions]
        start = list(self.left_arm_positions if arm == 'left'
                     else self.right_arm_positions)

        max_delta = max(abs(target - current)
                        for target, current in zip(goal, start))
        duration = self.arm_motion_duration(max_delta, speed)
        steps = max(1, int(self.get_parameter('arm_motion_steps').value))
        sleep_sec = duration / float(steps)

        for step in range(1, steps + 1):
            ratio = step / float(steps)
            interpolated = [
                current + (target - current) * ratio
                for target, current in zip(goal, start)
            ]
            if arm == 'left':
                self.left_arm_positions = interpolated
            else:
                self.right_arm_positions = interpolated

            self.publish_motion_state()
            if step < steps and sleep_sec > 0.0:
                time.sleep(sleep_sec)

        return True, f'{arm} arm target reached in {duration:.2f}s'

    def arm_motion_duration(self, max_delta, speed):
        min_duration = max(
            0.0, float(self.get_parameter('arm_motion_min_duration_sec').value))
        max_duration = max(
            min_duration,
            float(self.get_parameter('arm_motion_max_duration_sec').value))
        reference_speed = max(
            1.0, float(self.get_parameter('arm_motion_reference_speed').value))
        reference_velocity = max(
            0.01,
            float(self.get_parameter('arm_motion_reference_joint_velocity').value))
        requested_speed = max(1.0, float(speed))

        velocity = reference_velocity * (requested_speed / reference_speed)
        duration = max_delta / velocity if velocity > 0.0 else max_duration
        return self.clamp(duration, min_duration, max_duration)

    def publish_motion_state(self):
        stamp = self.get_clock().now().to_msg()
        self.publish_joint_states(stamp, 0.0, 0.0, 0.0)
        self.publish_head_pose()
        self.publish_torso_pose()
        self.publish_arm_poses()

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def map_callback(self, msg):
        self.map_msg = msg
        orientation = msg.info.origin.orientation
        self.map_origin_yaw = self.quaternion_to_yaw(orientation.z, orientation.w)

    def initial_pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(orientation.z, orientation.w)
        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.map_to_odom_yaw = 0.0
        self.cmd = Twist()
        self.get_logger().info(
            f'Reset pose to x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}')
        if bool(self.get_parameter('republish_initialpose_to_navigation').value):
            self.pending_navigation_initial_pose = msg
            if self.initial_pose_republish_timer is None:
                self.initial_pose_republish_timer = self.create_timer(
                    0.15, self.republish_initial_pose_to_navigation)

    def navigation_initial_pose_callback(self, msg):
        desired_x = msg.pose.pose.position.x
        desired_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        desired_yaw = self.quaternion_to_yaw(orientation.z, orientation.w)

        self.map_to_odom_yaw = self.normalize_angle(desired_yaw - self.yaw)
        cos_yaw = math.cos(self.map_to_odom_yaw)
        sin_yaw = math.sin(self.map_to_odom_yaw)
        self.map_to_odom_x = desired_x - (self.x * cos_yaw - self.y * sin_yaw)
        self.map_to_odom_y = desired_y - (self.x * sin_yaw + self.y * cos_yaw)
        self.get_logger().info(
            'Updated simulated laser map pose reference '
            f'to x={desired_x:.2f}, y={desired_y:.2f}, yaw={desired_yaw:.2f}')

    def republish_initial_pose_to_navigation(self):
        if self.pending_navigation_initial_pose is None:
            if self.initial_pose_republish_timer is not None:
                self.initial_pose_republish_timer.cancel()
                self.initial_pose_republish_timer = None
            return

        msg = PoseWithCovarianceStamped()
        msg.header = self.pending_navigation_initial_pose.header
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.pose = self.pending_navigation_initial_pose.pose
        self.navigation_initial_pose_pub.publish(msg)
        self.pending_navigation_initial_pose = None
        if self.initial_pose_republish_timer is not None:
            self.initial_pose_republish_timer.cancel()
            self.initial_pose_republish_timer = None

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        vx = self.cmd.linear.x
        vy = self.cmd.linear.y
        wz = self.cmd.angular.z

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * self.dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * self.dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * self.dt),
                              math.cos(self.yaw + wz * self.dt))

        self.publish_odometry(stamp, vx, vy, wz)
        self.publish_joint_states(stamp, vx, vy, wz)
        self.publish_head_pose()
        self.publish_torso_pose()
        self.publish_arm_poses()
        self.publish_laser(stamp)

    @staticmethod
    def yaw_to_quaternion(yaw):
        return math.sin(yaw * 0.5), math.cos(yaw * 0.5)

    @staticmethod
    def quaternion_to_yaw(qz, qw):
        return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_odometry(self, stamp, vx, vy, wz):
        qz, qw = self.yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def publish_joint_states(self, stamp, vx, vy, wz):
        js = JointState()
        js.header.stamp = stamp
        js.name = [
            'wheel0_joint', 'wheel1_joint', 'wheel2_joint',
            'torso_joint',
            'head_pan_joint', 'head_tilt_joint',
            'lf_shoulder_joint', 'ls_shoulder_joint',
            'l_arm_twist_joint', 'l_elbow_joint',
            'l_wrist_joint', 'l_hand_joint',
            'left_gripper_R_joint', 'left_gripper_L_joint',
            'rf_shoulder_joint', 'rs_shoulder_joint',
            'r_arm_twist_joint', 'r_elbow_joint',
            'r_wrist_joint', 'r_hand_joint',
            'right_gripper_R_joint', 'right_gripper_L_joint',
        ]

        wheel_speed = math.hypot(vx, vy) + abs(wz) * 0.15
        for i in range(3):
            self.wheel_positions[i] += wheel_speed * self.dt

        js.position = self.wheel_positions + [
            self.torso_position, self.head_pan, self.head_tilt]
        js.position.extend(self.left_arm_positions)
        js.position.extend([0.0, 0.0])
        js.position.extend(self.right_arm_positions)
        js.position.extend([0.0, 0.0])
        self.joint_pub.publish(js)

    def publish_head_pose(self):
        msg = Float64MultiArray()
        msg.data = [self.head_pan, self.head_tilt]
        self.head_pose_pub.publish(msg)

    def publish_torso_pose(self):
        msg = Float64MultiArray()
        msg.data = [self.torso_position]
        self.torso_pose_pub.publish(msg)

    def publish_arm_poses(self):
        left = Float64MultiArray()
        left.data = list(self.left_arm_positions)
        self.left_arm_pose_pub.publish(left)

        right = Float64MultiArray()
        right.data = list(self.right_arm_positions)
        self.right_arm_pose_pub.publish(right)

    def publish_laser(self, stamp):
        range_min = float(self.get_parameter('laser_range_min').value)
        range_max = float(self.get_parameter('laser_range_max').value)
        beams = max(2, int(self.get_parameter('laser_beams').value))
        fov = math.radians(float(self.get_parameter('laser_fov_deg').value))

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -fov / 2.0
        scan.angle_max = fov / 2.0
        scan.angle_increment = fov / float(beams - 1)
        scan.scan_time = self.dt
        scan.range_min = range_min
        scan.range_max = range_max
        no_hit = range_max if self.bool_parameter('laser_publish_max_range') else math.inf
        scan.ranges = [
            self.cast_laser_ray(scan.angle_min + i * scan.angle_increment,
                                range_min,
                                range_max,
                                no_hit)
            for i in range(beams)
        ]
        self.scan_pub.publish(scan)

    def cast_laser_ray(self, relative_angle, range_min, range_max, no_hit):
        if self.map_msg is None:
            return no_hit

        resolution = self.map_msg.info.resolution
        step = max(resolution * 0.5, 0.02)
        laser_x, laser_y, laser_yaw = self.laser_pose()
        angle = laser_yaw + relative_angle
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        obstacle_threshold = int(self.get_parameter('laser_obstacle_threshold').value)
        unknown_is_obstacle = self.bool_parameter('laser_unknown_is_obstacle')

        distance = range_min
        while distance <= range_max:
            wx = laser_x + distance * cos_angle
            wy = laser_y + distance * sin_angle
            cell = self.world_to_map(wx, wy)
            if cell is None:
                return no_hit

            mx, my = cell
            value = self.map_msg.data[my * self.map_msg.info.width + mx]
            if value < 0 and unknown_is_obstacle:
                return distance
            if value >= obstacle_threshold:
                return distance
            distance += step

        return no_hit

    def laser_pose(self):
        x_offset = float(self.get_parameter('laser_x_offset').value)
        y_offset = float(self.get_parameter('laser_y_offset').value)
        yaw_offset = float(self.get_parameter('laser_yaw_offset').value)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        odom_laser_x = self.x + x_offset * cos_yaw - y_offset * sin_yaw
        odom_laser_y = self.y + x_offset * sin_yaw + y_offset * cos_yaw
        odom_laser_yaw = self.yaw + yaw_offset

        cos_map = math.cos(self.map_to_odom_yaw)
        sin_map = math.sin(self.map_to_odom_yaw)
        laser_x = self.map_to_odom_x + odom_laser_x * cos_map - odom_laser_y * sin_map
        laser_y = self.map_to_odom_y + odom_laser_x * sin_map + odom_laser_y * cos_map
        laser_yaw = self.normalize_angle(self.map_to_odom_yaw + odom_laser_yaw)
        return laser_x, laser_y, laser_yaw

    def bool_parameter(self, name):
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        return str(value).lower() in ('1', 'true', 'yes', 'on')

    def world_to_map(self, wx, wy):
        origin = self.map_msg.info.origin.position
        resolution = self.map_msg.info.resolution
        dx = wx - origin.x
        dy = wy - origin.y

        cos_origin = math.cos(self.map_origin_yaw)
        sin_origin = math.sin(self.map_origin_yaw)
        local_x = cos_origin * dx + sin_origin * dy
        local_y = -sin_origin * dx + cos_origin * dy

        mx = int(local_x / resolution)
        my = int(local_y / resolution)
        if mx < 0 or my < 0:
            return None
        if mx >= self.map_msg.info.width or my >= self.map_msg.info.height:
            return None
        return mx, my


def main():
    rclpy.init()
    node = SimpleJustinaSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
