#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import importlib
import traceback

class UniversalSubscriber(Node):
    def __init__(self):
        super().__init__('universal_subscriber')
        
        # Format: {topic_name: 'package_name/MessageType'}
        self.topic_msg_dict = {
            # Standard messages
            '/string_topic': 'std_msgs/String',
            '/int32_topic': 'std_msgs/Int32',
            '/float_topic': 'std_msgs/Float32',
            '/bool_topic': 'std_msgs/Bool',
            
            # Geometry messages
            '/pose_topic': 'geometry_msgs/Pose',
            '/pose_array_topic': 'geometry_msgs/PoseArray',
            '/twist_topic': 'geometry_msgs/Twist',
            '/point_topic': 'geometry_msgs/Point',
            '/vector3_topic': 'geometry_msgs/Vector3',
            '/quaternion_topic': 'geometry_msgs/Quaternion',
            '/transform_topic': 'geometry_msgs/Transform',
            
            # Sensor messages
            '/image_topic': 'sensor_msgs/Image',
            '/camera_info_topic': 'sensor_msgs/CameraInfo',
            '/laser_scan_topic': 'sensor_msgs/LaserScan',
            '/imu_topic': 'sensor_msgs/Imu',
            '/point_cloud_topic': 'sensor_msgs/PointCloud2',
            
            # Navigation messages
            '/odom_topic': 'nav_msgs/Odometry',
            '/path_topic': 'nav_msgs/Path',
            '/grid_map_topic': 'nav_msgs/OccupancyGrid',
            
            # Diagnostic messages
            '/diagnostic_topic': 'diagnostic_msgs/DiagnosticArray',
            
            # Actionlib messages
            '/goal_topic': 'actionlib_msgs/GoalID',
            
            # TF messages
            '/tf_topic': 'tf2_msgs/TFMessage',
            
            # '/custom_topic': 'my_package/CustomMessage'
        }
        
        self.subscribers = []
        self.setup_subscribers()
        
    def setup_subscribers(self):
        # Create a subscriber for each topic in the dictionary
        for topic, msg_type in self.topic_msg_dict.items():
            try:
                # Parse the message type
                package_name, message_type = msg_type.split('/')
                module_name = f"{package_name}.msg"
                
                # Dynamically import the message module and class
                module = importlib.import_module(module_name)
                msg_class = getattr(module, message_type)
                
                # Create a subscriber for this topic and message type
                self.get_logger().info(f"Creating subscriber for topic: {topic}, type: {msg_type}")
                sub = self.create_subscription(
                    msg_class,
                    topic,
                    lambda msg, topic=topic, msg_type=msg_type: self.generic_callback(msg, topic, msg_type),
                    10
                )
                self.subscribers.append(sub)
                
            except Exception as e:
                self.get_logger().error(f"Failed to create subscriber for {topic} with type {msg_type}")
                self.get_logger().error(f"Error: {str(e)}")
                self.get_logger().error(traceback.format_exc())
    
    def generic_callback(self, msg, topic, msg_type):
        self.get_logger().info(f"Received on topic: {topic}")
        self.get_logger().info(f"Message type: {msg_type}")
        self.get_logger().info(f"Message content: {msg}")
        self.get_logger().info("-----------------------------")


def main(args=None):
    rclpy.init(args=args)
    node = UniversalSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()