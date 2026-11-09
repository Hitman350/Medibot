#!/usr/bin/env python3
"""
delivery_manager.py -- Hospital Medicine Delivery Manager.

Visits ALL beds sequentially: Pharmacy -> Bed 1 -> Bed 2 -> Bed 3 -> Bed 4 -> Pharmacy.

Usage:
  ros2 run medibot_sim delivery_manager
  ros2 run medibot_sim delivery_manager --ros-args -p bed_id:=bed_02  (single bed)
  ros2 run medibot_sim delivery_manager --ros-args -p mode:=all       (all beds)
"""

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import time
from enum import Enum
from datetime import datetime

from medibot_sim.bed_registry import BedRegistry


class MissionState(Enum):
    IDLE = "IDLE"
    REQUEST_RECEIVED = "REQUEST_RECEIVED"
    NAVIGATING_TO_BED = "NAVIGATING_TO_BED"
    ARRIVED_AT_BED = "ARRIVED_AT_BED"
    DELIVERY_CONFIRMED = "DELIVERY_CONFIRMED"
    RETURNING_TO_PHARMACY = "RETURNING_TO_PHARMACY"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class DeliveryManager(Node):
    """Manages medicine delivery missions."""

    def __init__(self):
        super().__init__('delivery_manager')

        self.declare_parameter('bed_id', 'all')
        self.declare_parameter('medicine_id', 'MED-DEFAULT')
        self.declare_parameter('mode', 'all')

        self.bed_id = self.get_parameter('bed_id').get_parameter_value().string_value
        self.medicine_id = self.get_parameter('medicine_id').get_parameter_value().string_value
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.registry = BedRegistry()
        self.navigator = BasicNavigator()
        self.state = MissionState.IDLE
        self.mission_log = []

        self.status_pub = self.create_publisher(String, '/delivery_status', 10)

        self.get_logger().info('+======================================+')
        self.get_logger().info('|   Hospital Delivery Manager          |')
        self.get_logger().info('+======================================+')

    def set_state(self, new_state: MissionState):
        old = self.state
        self.state = new_state
        ts = datetime.now().strftime('%H:%M:%S')
        self.mission_log.append({'time': ts, 'from': old.value, 'to': new_state.value})
        self.get_logger().info(f'State: {old.value} -> {new_state.value}')
        msg = String()
        msg.data = f'{new_state.value}'
        self.status_pub.publish(msg)

    def create_pose(self, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def navigate_to(self, x: float, y: float, yaw: float, label: str) -> bool:
        self.get_logger().info(f'Navigating to {label}: ({x:.1f}, {y:.1f})')
        goal = self.create_pose(x, y, yaw)
        self.navigator.goToPose(goal)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                dist = feedback.distance_remaining
                self.get_logger().info(
                    f'  -> {label}: {dist:.2f}m remaining',
                    throttle_duration_sec=3.0)
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'  Arrived at {label}!')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warning(f'  Navigation to {label} cancelled')
            return False
        else:
            self.get_logger().error(f'  Navigation to {label} FAILED')
            return False

    def deliver_to_bed(self, bed_id: str) -> bool:
        """Navigate to a single bed, deliver, return True on success."""
        bed = self.registry.get_bed(bed_id)
        if bed is None:
            self.get_logger().error(f'Unknown bed: {bed_id}')
            return False

        self.get_logger().info(f'')
        self.get_logger().info(f'Delivery order: {bed["name"]}')
        self.get_logger().info(f'   Medicine: {self.medicine_id}')

        self.set_state(MissionState.NAVIGATING_TO_BED)
        success = self.navigate_to(bed['x'], bed['y'], bed['yaw'], bed['name'])

        if not success:
            return False

        self.set_state(MissionState.ARRIVED_AT_BED)
        self.get_logger().info(f'Delivering {self.medicine_id} to {bed["name"]}...')
        time.sleep(2.0)

        self.set_state(MissionState.DELIVERY_CONFIRMED)
        self.get_logger().info(f'Delivery confirmed at {bed["name"]}!')
        return True

    def run_delivery(self):
        """Execute delivery mission -- single bed or all beds."""
        start_time = datetime.now()
        self.set_state(MissionState.REQUEST_RECEIVED)

        # Wait for Nav2
        self.get_logger().info('Waiting for Nav2 to be fully ready...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active!')

        # Set initial pose at pharmacy
        pharmacy = self.registry.get_pharmacy()
        initial = self.create_pose(pharmacy['x'], pharmacy['y'],
                                    pharmacy.get('yaw', 0.0))
        self.navigator.setInitialPose(initial)
        self.get_logger().info('Initial pose set at Pharmacy. Waiting for AMCL...')
        time.sleep(3.0)

        # Determine which beds to visit
        if self.bed_id == 'all' or self.mode == 'all':
            bed_ids = self.registry.list_beds()
            self.get_logger().info(f'FULL WARD ROUND: visiting {len(bed_ids)} beds')
        else:
            bed_ids = [self.bed_id]
            self.get_logger().info(f'Single delivery to: {self.bed_id}')

        # Visit each bed
        delivered = []
        failed = []
        for bed_id in bed_ids:
            success = self.deliver_to_bed(bed_id)
            if success:
                delivered.append(bed_id)
            else:
                failed.append(bed_id)
                self.get_logger().warning(f'Skipping {bed_id}, continuing tour...')

        # Return to pharmacy
        self.get_logger().info('')
        self.get_logger().info('Returning to Pharmacy...')
        self.set_state(MissionState.RETURNING_TO_PHARMACY)
        self.navigate_to(pharmacy['x'], pharmacy['y'],
                         pharmacy.get('yaw', 0.0), 'Pharmacy')

        self.set_state(MissionState.COMPLETED)

        # Mission summary
        elapsed = (datetime.now() - start_time).total_seconds()
        self.get_logger().info('')
        self.get_logger().info('+======================================+')
        self.get_logger().info('|        MISSION SUMMARY               |')
        self.get_logger().info('+======================================+')
        self.get_logger().info(f'| Delivered: {len(delivered)}/{len(bed_ids)} beds')
        for b in delivered:
            bed = self.registry.get_bed(b)
            self.get_logger().info(f'|   [OK] {bed["name"]}')
        for b in failed:
            bed = self.registry.get_bed(b)
            self.get_logger().info(f'|   [FAIL] {bed["name"]}')
        self.get_logger().info(f'| Medicine: {self.medicine_id}')
        self.get_logger().info(f'| Time:     {elapsed:.0f}s')
        self.get_logger().info('+======================================+')


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryManager()

    try:
        node.run_delivery()
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted!')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
