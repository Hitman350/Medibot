#!/usr/bin/env python3
"""
test_mover.py — Phase 1 verification node.

Publishes velocity commands to make the TurtleBot3 drive in a square
pattern, proving that the Gazebo simulation responds to ROS 2 commands.

IMPORTANT: The TurtleBot3 Gazebo bridge on ROS 2 Lyrical maps /cmd_vel
as geometry_msgs/msg/TwistStamped (NOT plain Twist). This node uses
TwistStamped accordingly.

Topics:
  - Publishes to:  /cmd_vel (geometry_msgs/TwistStamped)
  - Subscribes to: /odom   (nav_msgs/Odometry) — position feedback
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
import math


class TestMover(Node):
    """Simple node to verify robot motion in Gazebo."""

    def __init__(self):
        super().__init__('test_mover')

        # --- Publishers & Subscribers ---
        # /cmd_vel expects TwistStamped on the TurtleBot3 Lyrical bridge
        self.vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # /odom gives us the robot's current position
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # --- State machine ---
        # States: 'forward' → 'turn' → 'forward' → ... → 'done'
        self.state = 'forward'
        self.state_start_time = self.get_clock().now()
        self.current_x = 0.0
        self.current_y = 0.0
        self.odom_received = False
        self.segment_count = 0   # Completed forward+turn pairs
        self.max_segments = 4    # 4 sides = square

        # --- Motion parameters (safe hospital speeds) ---
        self.linear_speed = 0.15    # m/s — slow and safe for indoors
        self.angular_speed = 0.5    # rad/s — gentle turns
        self.forward_duration = 3.0  # seconds driving forward per side
        # Time for a 90° turn at angular_speed:
        self.turn_duration = (math.pi / 2.0) / self.angular_speed

        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║     Test Mover Node — Phase 1       ║')
        self.get_logger().info('╚══════════════════════════════════════╝')
        self.get_logger().info(f'Plan: Drive a square ({self.max_segments} segments)')
        self.get_logger().info(f'Speed: {self.linear_speed} m/s forward, '
                               f'{self.angular_speed} rad/s turning')
        self.get_logger().info(f'Forward: {self.forward_duration}s | '
                               f'Turn: {self.turn_duration:.1f}s per segment')
        self.get_logger().info('Publishing TwistStamped to /cmd_vel')
        self.get_logger().info('Waiting for /odom data...')

    def odom_callback(self, msg: Odometry):
        """Store current position from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f'Odometry received! Start pos: '
                f'({self.current_x:.2f}, {self.current_y:.2f})')

    def elapsed_seconds(self) -> float:
        """Seconds elapsed since the current state began."""
        now = self.get_clock().now()
        return (now - self.state_start_time).nanoseconds / 1e9

    def make_cmd(self, linear_x: float = 0.0,
                 angular_z: float = 0.0) -> TwistStamped:
        """Create a TwistStamped message with the current timestamp."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        return msg

    def timer_callback(self):
        """Main control loop — runs at 10 Hz."""

        if self.state == 'forward':
            if self.elapsed_seconds() < self.forward_duration:
                cmd = self.make_cmd(linear_x=self.linear_speed)
                self.vel_pub.publish(cmd)
                self.get_logger().info(
                    f'[{self.segment_count + 1}/{self.max_segments}] '
                    f'FORWARD → pos=({self.current_x:.2f}, {self.current_y:.2f})',
                    throttle_duration_sec=1.0)
            else:
                # Switch to turning
                self.state = 'turn'
                self.state_start_time = self.get_clock().now()
                self.get_logger().info('→ Switching to TURN (90°)')

        elif self.state == 'turn':
            if self.elapsed_seconds() < self.turn_duration:
                cmd = self.make_cmd(angular_z=self.angular_speed)
                self.vel_pub.publish(cmd)
                self.get_logger().info(
                    f'[{self.segment_count + 1}/{self.max_segments}] TURNING',
                    throttle_duration_sec=1.0)
            else:
                # Segment complete
                self.segment_count += 1
                if self.segment_count >= self.max_segments:
                    self.state = 'done'
                    self.get_logger().info('══════════════════════════════════')
                    self.get_logger().info('  ✅ SQUARE COMPLETE!')
                    self.get_logger().info(
                        f'  Final pos: ({self.current_x:.2f}, '
                        f'{self.current_y:.2f})')
                    self.get_logger().info('══════════════════════════════════')
                else:
                    self.state = 'forward'
                    self.state_start_time = self.get_clock().now()
                    self.get_logger().info(
                        f'→ Starting segment {self.segment_count + 1}')

        elif self.state == 'done':
            # Stop the robot
            cmd = self.make_cmd()
            self.vel_pub.publish(cmd)
            self.get_logger().info(
                'Robot stopped. Press Ctrl+C to exit.',
                throttle_duration_sec=5.0)

    def stop_robot(self):
        """Send zero velocity to ensure the robot stops."""
        cmd = self.make_cmd()
        self.vel_pub.publish(cmd)
        self.get_logger().info('Stop command sent.')


def main(args=None):
    rclpy.init(args=args)
    node = TestMover()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
        node.get_logger().info('Interrupted — robot stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()