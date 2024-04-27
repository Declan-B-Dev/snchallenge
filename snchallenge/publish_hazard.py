#!/usr/bin/env python  

import rclpy
from rclpy.node import Node

from rclpy.qos import ReliabilityPolicy, QoSProfile

from std_msgs.msg import Float32MultiArray

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import time

from collections import OrderedDict

import math
import numpy as np
import cv2 as cv
import tf2_ros

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
        self.pub = self.create_publisher(Marker, '/hazards', 10)
        
        self.start_pub = self.create_publisher(Bool, '/start', 10)

        self.hazards_found = self.create_publisher(Bool, '/go_home', 10)

        # Subscriber for /Objects
        self.sub_be = self.create_subscription(
                            Float32MultiArray,
                            '/objects',
                            self.object_listener,
                            10
                    )

        # Subscriber for /Objects
        self.sub_be = self.create_subscription(
                            LaserScan,
                            '/scan',
                            self.scan_listener,
                            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                    )
        
        self.scan_dict = OrderedDict()

        self.hazards = []

        self.started = False
        
        # Transform listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

    def scan_listener(self, msg):
        timeStamp = msg.header.stamp
        keyTime = round(timeStamp.sec + timeStamp.nanosec/1000000000, 1) # convert nano to seconds, 
        # publisher already rounds to single decimal place here we explicitely enforce it

        self.scan_dict[keyTime] = msg.ranges
        #self.get_logger().info(f"Laser time: {timeStamp}")
        
        #self.get_logger().info(f"Key time: {keyTime}")
        

        if len(self.scan_dict) > 30:
            item = self.scan_dict.popitem(last=False) # pop oldest entry
            #self.get_logger().info(f"Range: {self.scan_dict[keyTime][0]}")

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
        marker_msg.id = int(details[0])

        #todo CAMERA HEIGHT

        poseT = self.transform([details[1], details[2], 0.0], 'camera_color_frame', 'map')
        
        if(poseT):
            #self.get_logger().info(f"PoseT: {poseT.pose}")
            
            marker_msg.pose.position.x = poseT.pose.position.x
            marker_msg.pose.position.y = poseT.pose.position.y
            marker_msg.pose.position.z = poseT.pose.position.z

            marker_msg.pose.orientation.x = 1.0
            marker_msg.pose.orientation.y = 0.0
            marker_msg.pose.orientation.z = 0.0
            marker_msg.pose.orientation.w = 0.0
            
            marker_msg.scale.x = 1.0 
            marker_msg.scale.y = 1.0
            marker_msg.scale.z = 1.0
            marker_msg.color.r = 1.0
            marker_msg.color.g = 0.0
            marker_msg.color.b = 0.0
            marker_msg.color.a = 1.0
            
            # Infinite lifetime
            marker_msg.lifetime.sec = 0
                 
            self.get_logger().info(f"HAZARD PUBLISHED")
            self.pub.publish(marker_msg) 
                        
            
            # self.get_logger().info('Visualization marker published.')

    def object_listener(self, msg):

        if msg.data and msg.data[0] == 13 and (not self.started):
            start_msg = Bool()
            start_msg.data = True
            self.start_pub.publish(start_msg)
            self.get_logger().info(f'Sent start msg')
            self.started = True
        elif msg.data and (msg.data[0] not in self.hazards) and msg.data[0] != 13:
            #self.get_logger().info(f'Object Data: {msg.data[0]}')
            time = self.get_clock().now() # CONSTRUCT time = rclpy.time.Time()
            
            self.get_object_position(msg.data, time)
            self.hazards.append(msg.data[0])
            self.get_logger().info(f"Hazards: {self.hazards}")
            
            if len(self.hazards) >= 5:
                self.get_logger().info(f"Go to goal pose")   
                
                start_msg = Bool()
                start_msg.data = False
                self.start_pub.publish(start_msg)
                end_msg = Bool()
                end_msg.data = True
                self.hazards_found.publish(end_msg)

                
    
    # Code based on https://husarion.com/tutorials/ros-tutorials/5-visual-object-recognition/#recognizing-objects
    def get_object_position(self, data, time):
        
        id = data[0]
        object_width = data[1]
        object_height = data[2]

        cv_homography = np.zeros((3, 3), dtype=np.float32)

        for i in range(9):
            cv_homography[i % 3, i // 3] = data[i + 3]

        in_pts = np.array([[0, 0], [object_width, 0], [0, object_height], [object_width, object_height]], dtype=np.float32)
        in_pts = np.expand_dims(in_pts, axis=1)
        
        out_pts = cv.perspectiveTransform(in_pts, cv_homography)

        obj_x_pos = np.mean(out_pts[:, :, 0])
        
        self.get_logger().info(f"X Pos: {obj_x_pos}")

        #minX = 0
        #maxX = 640

        factor = 320 - obj_x_pos

        #minAng = 315
        #maxAng = 45

        laser = 315 + round(obj_x_pos/640*90)

        if(laser > 359):
            laser = laser - 360

        depth = min(self.get_depth(time, laser), 3) 

        xValue = depth - abs(factor/640)
        yValue = depth*(factor/640)

        self.get_logger().info(f"laser: {laser}")
        self.get_logger().info(f"xValue: {xValue}")
        self.get_logger().info(f"yValue: {yValue}")
        
        details = [id, xValue, yValue]
        
        self.publish_hazard(details)

    def get_depth(self, timestamp, range):
      
        #1714130290.672538618

        timestamp = round(timestamp.nanoseconds/1000000000,1)
        self.get_logger().info(f"Time2: {timestamp}")
    

        depth = 0.5
        range = range*2 # rosbots have 720 readings not 360

        if timestamp in self.scan_dict:
            depth = self.scan_dict[timestamp][range]
            self.get_logger().info(f"Using deange1: {len(self.scan_dict[timestamp])}")
            self.get_logger().info(f"Found depth: {depth}")

        elif timestamp - 0.1 in self.scan_dict:
            depth = self.scan_dict[timestamp - 0.1][range]
            self.get_logger().info(f"Using deange2: {len(self.scan_dict[timestamp - 0.1])}")
            self.get_logger().info(f"Found depth2: {depth}")

        elif self.scan_dict:
            depth = self.scan_dict.popitem()[1] # most recent entry
            self.get_logger().info(f"Using deange3: {len(depth)}")
            depth = depth[range]
            self.get_logger().info(f"Using depth3: {depth}")

        return depth
        

    def transform(self, srcPose, src, dest):
        
        poseT = None
        
        try:
            # Current time CONFIGURABLE 
            #time = self.get_clock().now() - rclpy.duration.Duration(seconds=0.4)
            time = rclpy.time.Time()
            #self.get_logger().info(str(time))
            self.get_logger().info(f"Transform2") 

            pose = PoseStamped()
            pose.header.frame_id = src
            pose.header.stamp = time.to_msg()
            
            pose.pose.position.x = srcPose[0]
            pose.pose.position.y = srcPose[1]
            pose.pose.position.z = srcPose[2]
            
            # Equivalent to 0,0,0 roll, pitch, yaw
            pose.pose.orientation.x = 1.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0

            # Configure frames
            
            #self.get_logger().info(f"From frame: {src}")
            #self.get_logger().info(f"To frame: {dest}")

            # Timeout for transform data
            timeout = rclpy.duration.Duration(seconds=0.8)

            # Lookup a transform
            #transform = self.tf_buffer.lookup_transform(dest, src, time, timeout=timeout)
            #self.get_logger().info(f"Transform: {transform.transform}")
            self.get_logger().info(f"Tranform3")

            poseT = self.tf_buffer.transform(pose, dest)
            
            self.get_logger().info(f"Transform4")

        except tf2_ros.ExtrapolationException as ex:
            #pass
            self.get_logger().info(f'Could not gain current data for {src} to {dest}: {ex}')
        except tf2_ros.TransformException as ex2:
            #pass
            self.get_logger().info(f'Could not transform {src} to {dest}: {ex2}')
        
        return poseT
    
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
