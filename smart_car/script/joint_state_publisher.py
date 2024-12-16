#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from smartcar_msgs.msg import Status

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Subscribe to vehicle status
        self.subscription = self.create_subscription(
            Status,
            '/smartcar/vehicle_status',
            self.vehicle_status_callback,
            10
        )
        
        # joint state message initialize
        self.msg = JointState()
        self.msg.name = [
            'back_left_wheel_joint',
            'back_right_wheel_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'front_left_wheel_steer_joint',
            'front_right_wheel_steer_joint'
        ]
        
        self.wheel_position = 0.0
        
    def vehicle_status_callback(self, msg):
        self.update_wheel_position(msg.engine_speed_rpm, msg.steering_angle_rad)
        
    def update_wheel_position(self, engine_speed, steering_angle):
        wheel_rotation_rate = (engine_speed * 2.0 * 3.14159) / 60.0  # Convert RPM to rad/s
        current_time = self.get_clock().now()
        dt = 0.02  # 50Hz update
        
        self.wheel_position += wheel_rotation_rate * dt
        
        # Create joint state message
        self.msg.header.stamp = current_time.to_msg()
        self.msg.position = [
            self.wheel_position,  # back left wheel
            self.wheel_position,  # back right wheel
            self.wheel_position,  # front left wheel
            self.wheel_position,  # front right wheel
            steering_angle,       # front left steering
            steering_angle        # front right steering
        ]
        
        # Publish 
        self.publisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
