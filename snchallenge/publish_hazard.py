#!/usr/bin/env python  

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker

import math

class HazardPublisher(Node):
    def __init__(self):
        super().__init__('aiil_hazardpublisher')
        
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
        self.topic = "/hazards"
        self.pub = self.create_publisher(Marker, self.topic, 10)
        
        # Iteration
        self.timer = self.create_timer(self.frequency, self.publish_path)
        self.xmul = 1.0

    def publish_path(self):
        # Current time 
        time = self.get_clock().now()
        self.get_logger().info(str(time))
        
        # Create the marker
        marker_msg = Marker()
        marker_msg.header.frame_id = 'map'
        marker_msg.header.stamp = time.to_msg()
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        
        marker_msg.pose.position.x = 1.0 * self.xmul
        marker_msg.pose.position.y = 2.0
        marker_msg.pose.position.z = 0.0
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0
        
        marker_msg.scale.x = 1.0 
        marker_msg.scale.y = 1.0
        marker_msg.scale.z = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0
        
        # Infinite lifetime
        marker_msg.lifetime.sec = 0

        # Publish
        self.pub.publish(marker_msg)
        self.get_logger().info('Visualization marker published.')
        
        self.xmul = -self.xmul

def main():
    rclpy.init()
    node = HazardPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    exit(0)
