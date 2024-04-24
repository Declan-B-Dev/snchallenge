#!/usr/bin/env python  

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from visualization_msgs.msg import Marker

import math
import numpy as np
import cv2 as cv

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
        #self.timer = self.create_timer(self.frequency, self.publish_path)
        self.xmul = 1.0

        # Subscriber for /Objects
        self.sub_be = self.create_subscription(
                            Float32MultiArray,
                            '/objects',
                            self.object_listener,
                            10
                    )

    def publish_hazard(self, details):
        # Current time 
        time = self.get_clock().now()
        #self.get_logger().info(str(time))
        
        # Create the marker
        marker_msg = Marker()
        marker_msg.header.frame_id = 'map'
        marker_msg.header.stamp = time.to_msg()
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        
        # Marker ID
        marker_msg.id = details[0]

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
        #self.get_logger().info('Visualization marker published.')
        
        self.xmul = -self.xmul
    
    def object_listener(self, msg):
        #MultiArrayLayout  layout        # specification of data layout
            #MultiArrayDimension[] dim #
                #string label   #
                #uint32 size    #
                #uint32 stride  #
            #uint32 data_offset        #
        #float32[]         data          # array of data
        if(msg.data):
            self.get_logger().info(f'Object Data: {msg.data[0]}')
            self.get_object_position(msg.data)
    
    # Code based on https://husarion.com/tutorials/ros-tutorials/5-visual-object-recognition/#recognizing-objects
    def get_object_position(self, data):
        CAMERA_WIDTH = 640 # left 0, right 640

        MIN_ANG_VEL = 0.15
        MAX_ANG_VEL = 0.5
        ANGULAR_GAIN = 2e-3 
        #int obj_x_pos;
        #float ang_vel;

        # Reset linear and angular speed value
        #vel_msg.linear.x = 0;
        #vel_msg.angular.z = 0;

        if data:
            id = data[0]
            objectWidth = data[1]
            objectHeight = data[2]

            inPts = np.array([[0, 0], [objectWidth, 0], [0, objectHeight], [objectWidth, objectHeight]], dtype=np.float32)
            hMatrix = np.array([[data[3],data[4],data[5]],[data[6],data[7],data[8]],[data[9],data[10],data[11]]], dtype=np.float32).reshape((3, 3))
            inPts = np.array([inPts])

            dest = cv.perspectiveTransform(inPts, hMatrix)
            self.get_logger().info(f'Dest: {dest}')
            
            obj_x_pos = (dest[0][0][0] + dest[0][1][0] + dest[0][2][0] + dest[0][3][0]) / 4
            ang_vel = ANGULAR_GAIN*(CAMERA_WIDTH/2 - obj_x_pos)

            self.get_logger().info(f'XPos: {obj_x_pos}')
            self.get_logger().info(f'Angle: {ang_vel}')            

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
