#!/usr/bin/env python  


import rclpy
from rclpy.node import Node

import geometry_msgs.msg
import tf2_geometry_msgs
import tf2_ros


class Transform(Node):
    def __init__(self):
        super().__init__('aiil_transform')
        
        # Transform listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        
        # Setup timer callback
        self.timer = self.create_timer(1.0, self.transform)
    
    def transform(self):
        time = self.get_clock().now() - rclpy.duration.Duration(seconds=0.1)
        
        self.get_logger().info("--------------------------------")
        self.get_logger().info("Time:" + str(time))

        try:
            # Configure frames
            dest = 'map'
            src = 'base_link'
            self.get_logger().info(f"From frame: {src}")
            self.get_logger().info(f"To frame: {dest}")
            
            
            # Timeout for transform data
            timeout = rclpy.duration.Duration(seconds=0.05)
            
            # Lookup a transform
            transform = self.tf_buffer.lookup_transform(dest, src, time, timeout=timeout)
            self.get_logger().info(f"Transform: {transform.transform}")
            
            # Apply a transform that has been retrieved from the buffer
            pose = geometry_msgs.msg.Pose()
            pose.position.x = 1.0
            pose.position.y = 0.0
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            poseT = tf2_geometry_msgs.do_transform_pose(pose, transform)
            self.get_logger().info(f" - Pose: {pose} transformed is {poseT}")
            
            # Apply a transform to a stamped pose
            poseS = geometry_msgs.msg.PoseStamped()
            poseS.header.frame_id = src
            poseS.pose.position.x = 1.0
            poseS.pose.position.y = 0.0
            poseS.pose.position.z = 0.0
            poseS.pose.orientation.w = 1.0
            poseT = self.tf_buffer.transform(poseS, dest)
            self.get_logger().info(f" - Pose: {poseS} transformed is {poseT}")

        except tf2_ros.TransformException as ex:
            self.get_logger().info(f'Could not transform {src} to {dest}: {ex}')


def main():
    rclpy.init()
    node = Transform()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    exit(0)


