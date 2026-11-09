#!/usr/bin/env python3
"""
navigation_controller.py — Phase 2 Navigation Node.

Uses Nav2's BasicNavigator (nav2_simple_commander) to send navigation
goals programmatically. This is the correct way to control Nav2 from
Python — no RViz button pressing needed.

This node:
  1. Waits for Nav2 to become active
  2. Sets the robot's initial pose (for AMCL localization)
  3. Sends a navigation goal to a target position
  4. Monitors progress and reports success/failure

Usage:
  ros2 run medibot_sim navigation_controller
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
import time


class NavigationController(Node):
    """Sends Nav2 goals and monitors navigation results."""

    def __init__(self):
        super().__init__('navigation_controller')

        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║  Navigation Controller — Phase 2    ║')
        self.get_logger().info('╚══════════════════════════════════════╝')

    def create_pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        """
        Create a PoseStamped message for Nav2.

        Args:
            x: Target X position in map frame (meters)
            y: Target Y position in map frame (meters)
            yaw: Target heading angle (radians). 0 = facing +X axis.

        Returns:
            PoseStamped message ready for Nav2
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert yaw angle to quaternion (rotation around Z axis)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose


def main(args=None):
    rclpy.init(args=args)

    # Create our node (for logging)
    node = NavigationController()
    logger = node.get_logger()

    # Create the Nav2 navigator
    navigator = BasicNavigator()

    # --- Step 1: Set the initial pose for AMCL ---
    # This tells AMCL "the robot is here" so localization can start.
    # Must match where the robot was spawned in Gazebo.
    logger.info('Setting initial pose...')
    initial_pose = node.create_pose(x=-2.0, y=-0.5, yaw=0.0)
    navigator.setInitialPose(initial_pose)

    # --- Step 2: Wait for Nav2 to fully activate ---
    logger.info('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active()
    logger.info('✅ Nav2 is active!')

    # --- Step 3: Send a navigation goal ---
    # Navigate to position (2.0, 1.0) — a point in the TB3 sandbox world
    goal_x = 2.0
    goal_y = 1.0
    goal_yaw = 0.0

    logger.info(f'Sending navigation goal: ({goal_x}, {goal_y})')
    goal_pose = node.create_pose(x=goal_x, y=goal_y, yaw=goal_yaw)
    navigator.goToPose(goal_pose)

    # --- Step 4: Monitor navigation progress ---
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            # Log progress every ~5 seconds (at 2Hz feedback rate)
            eta = feedback.estimated_time_remaining
            logger.info(
                f'Navigating... ETA: {eta.sec}s | '
                f'Distance remaining: '
                f'{feedback.distance_remaining:.2f}m')

            # Safety timeout: abort if taking too long
            if eta.sec > 120:
                logger.warn('Navigation taking too long — cancelling!')
                navigator.cancelTask()
                break
        time.sleep(0.5)

    # --- Step 5: Check the result ---
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        logger.info('══════════════════════════════════')
        logger.info('  ✅ NAVIGATION SUCCEEDED!')
        logger.info(f'  Reached goal: ({goal_x}, {goal_y})')
        logger.info('══════════════════════════════════')
    elif result == TaskResult.CANCELED:
        logger.warn('Navigation was cancelled.')
    elif result == TaskResult.FAILED:
        logger.error('Navigation FAILED!')
    else:
        logger.error(f'Navigation returned unknown result: {result}')

    # Cleanup
    navigator.lifecycleShutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

