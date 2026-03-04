#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import math

class ObstacleChecker(Node):
    def __init__(self):
        super().__init__('obstacle_checker')
        
        scan_topic = '/scan'
        cmd_vel_topic = '/diff_drive_controller/cmd_vel'
        
        self.last_twist_angular_z = 0
        self.last_twist_linear_x = 0

        self.stop_distance = 0.5
        
        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        
        self.cmd_vel_subscription = self.create_subscription(
            TwistStamped,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            cmd_vel_topic,
            10
        )
        
        self.obstacle_detected = False
        
        self.get_logger().info(f'Stop distance threshold: {self.stop_distance} m')

    def scan_callback(self, scan: LaserScan):
        """Callback to process laser scan data and detect obstacles."""
        self.obstacle_detected = False
        min_distance = float('inf')
        close_range_indexes = []
        for i, distance in enumerate(scan.ranges):
            if distance == float('inf') or distance < scan.range_min or distance > scan.range_max:
                continue
            
            angle = scan.angle_min + (i * scan.angle_increment)
            
            is_close_to_obstacle = distance <= self.stop_distance
            angle_degree = math.degrees(angle)
            is_rotated_to_obstacle = angle_degree < 90 and angle_degree > -90
            if is_close_to_obstacle and is_rotated_to_obstacle:
                self.obstacle_detected = True
                break


        self.get_logger().info(f"{self.last_twist_linear_x}")
        if self.last_twist_linear_x == 0.0:
            return
        
        if self.obstacle_detected:
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.header.frame_id = scan.header.frame_id
            
            stop_msg.twist.linear.x = 0.0
            stop_msg.twist.angular.z = self.last_twist_angular_z 
            self.cmd_vel_publisher.publish(stop_msg)
            self.get_logger().warn('OBSTACLE! Incoming cmd_vel overridden with STOP command!')


    def cmd_vel_callback(self, twist_stamped: TwistStamped):
        """Callback to intercept cmd_vel commands and stop if obstacle detected."""
        self.last_twist_angular_z = twist_stamped.twist.angular.z
        self.last_twist_linear_x = twist_stamped.twist.linear.x


def main(args=None):
    rclpy.init(args=args)
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
