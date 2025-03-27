#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Pose, Twist
import time

class TestPublishers(Node):
    def __init__(self):
        super().__init__('test_publishers')
        
        # Creating publishers for different message types
        self.string_publisher = self.create_publisher(
            String,
            '/string_topic',
            10
        )
        
        self.int_publisher = self.create_publisher(
            Int32,
            '/int32_topic',
            10
        )
        
        self.pose_publisher = self.create_publisher(
            Pose,
            '/pose_topic',
            10
        )
        
        self.twist_publisher = self.create_publisher(
            Twist,
            '/twist_topic',
            10
        )
        
        # timers with different frequencies
        self.create_timer(1.0, self.publish_string) 
        self.create_timer(2.0, self.publish_int)     
        self.create_timer(3.0, self.publish_pose)   
        self.create_timer(4.0, self.publish_twist) 
        
        self.counter = 0
        self.get_logger().info("Test publishers started")
        
    def publish_string(self):
        msg = String()
        msg.data = f"Hello world {self.counter}"
        self.string_publisher.publish(msg)
        self.get_logger().info(f"Published string: {msg.data}")
        self.counter += 1
        
    def publish_int(self):
        msg = Int32()
        msg.data = self.counter
        self.int_publisher.publish(msg)
        self.get_logger().info(f"Published int: {msg.data}")
        
    def publish_pose(self):
        msg = Pose()
        msg.position.x = 1.0
        msg.position.y = 2.0
        msg.position.z = 3.0
        msg.orientation.w = 1.0
        self.pose_publisher.publish(msg)
        self.get_logger().info("Published pose message")
        
    def publish_twist(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.2
        self.twist_publisher.publish(msg)
        self.get_logger().info("Published twist message")


def main(args=None):
    rclpy.init(args=args)
    node = TestPublishers()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()