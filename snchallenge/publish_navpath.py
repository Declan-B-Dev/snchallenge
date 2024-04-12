#!/usr/bin/env python  

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math

class NavigationPath(Node):
    def __init__(self):
        super().__init__('aiil_navigationpath')
        
        # Define Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', 1.0)
            ]
        )
        
        # Look-up parameters values
        self.frequency = self.get_parameter('frequency').value
        
        # Publisher
        self.topic = "/path"
        self.pub = self.create_publisher(Path, self.topic, 10)
        
        # Iteration
        self.timer = self.create_timer(self.frequency, self.publish_path)

    def publish_path(self):
        # Current time 
        time = self.get_clock().now()
        self.get_logger().info(str(time))
        
        # Create a new Path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = time.to_msg()

        # Add some example poses to the path
        for i in range(10):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = time.to_msg()
            angle = i * 0.1
            radius = angle
            pose.pose.position.x = radius * math.cos(angle)
            pose.pose.position.y = radius * math.sin(angle)
            
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        # Publish the path
        self.pub.publish(path_msg)
        self.get_logger().info('Navigation path published.')

def main():
    rclpy.init()
    node = NavigationPath()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    exit(0)
