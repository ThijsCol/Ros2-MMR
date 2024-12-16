#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from smartcar_msgs.msg import Status
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        
        # Parameters
        self.wheel_diameter = 0.064  # meters 
        self.wheelbase = 0.257  # meters 
        
        # Initialize position
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
        
        self.prev_time = self.get_clock().now()
        
        self.odom_pub = self.create_publisher(
            Odometry, 
            '/smartcar/wheel/odom', 
            10)
        
        self.status_sub = self.create_subscription(
            Status,
            '/smartcar/vehicle_status',
            self.vehicle_status_callback,
            10)
            
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer (50Hz)
        self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info('Wheel odometry node initialized')
        
        # velocity and angles
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.steering_angle = 0.0
        
    def vehicle_status_callback(self, msg):
        # Calculatevelocity from RPM
        rpm = float(msg.engine_speed_rpm)
        self.linear_velocity = (rpm * math.pi * self.wheel_diameter) / 60.0
        
        self.steering_angle = msg.steering_angle_rad
        
        # Calculate angular velocity 
        self.angular_velocity = (self.linear_velocity / self.wheelbase) * math.tan(self.steering_angle)
        
    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  
        
        # Update position and orientation
        self.phi = self.phi + self.angular_velocity * dt
        self.x = self.x + self.linear_velocity * math.cos(self.phi) * dt
        self.y = self.y + self.linear_velocity * math.sin(self.phi) * dt
        
        #  odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.phi / 2.0)
        q.w = math.cos(self.phi / 2.0)
        odom_msg.pose.pose.orientation = q
        
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity
        
        # covariance matrices
        pose_covariance = np.zeros(36)
        twist_covariance = np.zeros(36)
        
        # Position variance
        pose_covariance[0] = 1.0  # x
        pose_covariance[7] = 1.0  # y
        pose_covariance[14] = 100000.0  # z
        
        # Orientation variance
        pose_covariance[21] = 100000.0  # roll
        pose_covariance[28] = 100000.0  # pitch
        pose_covariance[35] = 0.5  # yaw
        
        # Linear velocity variance
        twist_covariance[0] = 0.1  # x
        twist_covariance[7] = 0.1  # y
        twist_covariance[14] = 100000.0  # z
        
        # Angular velocity variance
        twist_covariance[21] = 100000.0  # roll
        twist_covariance[28] = 100000.0  # pitch
        twist_covariance[35] = 0.1  # yaw
        
        odom_msg.pose.covariance = pose_covariance.tolist()
        odom_msg.twist.covariance = twist_covariance.tolist()
        
        # Publish odometry 
        self.odom_pub.publish(odom_msg)
        
        # Broadcast transform
        transform = TransformStamped()
        transform.header = odom_msg.header
        transform.child_frame_id = odom_msg.child_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(transform)
        
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
