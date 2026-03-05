#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import math

SCAN_TOPIC = '/scan'
CMD_VEL_TOPIC = '/diff_drive_controller/cmd_vel'

class ObstacleChecker(Node):
    """
    Node that monitors laser scans for obstacles and publishes stop commands.
    """

    def __init__(self, stop_distance=0.5, stop_degree=45, service_queue_size=10):
        """
        Initialize the ObstacleChecker node.

        Args:
            stop_distance (float): Minimum distance to obstacle in meters. Default is 0.5m.
            stop_degree (float): Angular range in degrees to check for obstacles. Default is 45 degrees.
            service_queue_size (int): Queue size for ROS2 subscriptions and publishers. Default is 10.
        """
        super().__init__('obstacle_checker')

        # Store the last received velocity commands to preserve angular velocity when stopping
        self.last_twist_angular_z = 0
        self.last_twist_linear_x = 0

        self.stop_distance = stop_distance  
        self.stop_degree = stop_degree

        # Subscribe to laser scan topic to receive sensor data
        self.subscription = self.create_subscription(
            LaserScan,
            SCAN_TOPIC,
            self.scan_callback,
            service_queue_size
        )

        # Subscribe to cmd_vel topic to intercept incoming velocity commands
        self.cmd_vel_subscription = self.create_subscription(
            TwistStamped,
            CMD_VEL_TOPIC,
            self.cmd_vel_callback,
            service_queue_size
        )

        # Publisher to send stop commands when obstacle is detected
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            CMD_VEL_TOPIC,
            service_queue_size
        )

        # Flag to track if an obstacle is currently detected in the path
        self.obstacle_detected = False

        self.get_logger().info(f'Stop distance threshold: {self.stop_distance} m')

    def publish_stop_message(self):
        """
        Publish a stop command to override velocity commands when obstacle detected.

        """
        # Don't publish stop if robot is already stationary
        if self.last_twist_linear_x == 0.0:
            return

        # Only publish stop command if an obstacle is detected in the path
        if self.obstacle_detected:
            # Create a TwistStamped message for velocity command
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.header.frame_id = "base_link"

            # Set linear velocity to 0 (stop forward motion)
            stop_msg.twist.linear.x = 0.0
            # Preserve angular velocity to allow rotation in place or continued turning
            stop_msg.twist.angular.z = self.last_twist_angular_z

            # Publish the stop command
            self.cmd_vel_publisher.publish(stop_msg)
            self.get_logger().warn('OBSTACLE! Incoming cmd_vel overridden with STOP command!')

    def scan_callback(self, scan: LaserScan):
        """
        Callback to process incoming laser scan data to detect obstacles.

        Args:
            scan (sensor_msgs.msg.LaserScan): The laser scan message containing range measurements.
        """
        # Reset obstacle detection flag for each new scan
        self.obstacle_detected = False

        # Iterate through all laser scan range measurements
        for i, distance in enumerate(scan.ranges):
            # Skip invalid measurements: infinity, below min range, or above max range
            if distance == float('inf') or distance < scan.range_min or distance > scan.range_max:
                continue

            # Check if the measured distance is within the stop distance threshold
            is_close_to_obstacle = distance <= self.stop_distance

            # Calculate the angle of this specific scan ray
            angle = scan.angle_min + (i * scan.angle_increment)
            angle_degree = math.degrees(angle)
            stop_degree = self.stop_degree

            # Check if the obstacle is within the angular range (in front of the robot)
            # This checks both positive and negative angles within stop_degree
            is_rotated_to_obstacle = angle_degree < stop_degree and angle_degree > -stop_degree

            # If obstacle is close enough AND within the front angular range, trigger stop
            if is_close_to_obstacle and is_rotated_to_obstacle:
                self.obstacle_detected = True
                # Stop checking further - we already found an obstacle
                break

        # After processing scan, check if we need to publish a stop command
        self.publish_stop_message()

    def cmd_vel_callback(self, twist_stamped: TwistStamped):
        """
        Callback to intercept incoming velocity commands and apply obstacle avoidance if needed.

        Args:
            twist_stamped (geometry_msgs.msg.TwistStamped): The velocity command message containing
                                                          linear and angular velocities.
        """
        # Store the latest velocity commands for later use when publishing stop commands
        self.last_twist_angular_z = twist_stamped.twist.angular.z
        self.last_twist_linear_x = twist_stamped.twist.linear.x

        # Check if we need to override this command with a stop (if obstacle detected)
        self.publish_stop_message()


def main():
    rclpy.init()
    node = ObstacleChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
