#!/usr/bin/env python  

import rclpy
from rclpy.node import Node

from rclpy.qos import ReliabilityPolicy, QoSProfile

from std_msgs.msg import Float32MultiArray

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from collections import OrderedDict

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
        
        # Publishers
        self.hazard_pub = self.create_publisher(Marker, '/hazards', 10)
        
        self.start_pub = self.create_publisher(Bool, '/start', 10)

        self.hazards_found = self.create_publisher(Bool, '/go_home', 10)

        # The /objects topic reports the id and position of an object relative to the camera image
        self.sub_be = self.create_subscription(
                            Float32MultiArray,
                            '/objects',
                            self.object_listener,
                            10
                    )

        # The /scan topic gives us LIDAR measurements
        self.sub_be = self.create_subscription(
                            LaserScan,
                            '/scan',
                            self.scan_listener,
                            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
                    )
        
        # Ordered dictionary allows us to access the earliest and latest items in dict
        self.scan_dict = OrderedDict()

        self.hazards = []

        self.started = False
        
        # Transform listener
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

    def scan_listener(self, msg):
        DICT_LIMIT = 30
        timeStamp = msg.header.stamp
        
        # Round to single decimal place for uniformity
        keyTime = round(timeStamp.sec + timeStamp.nanosec/1000000000, 1) # convert nano to seconds, 
        
        self.scan_dict[keyTime] = msg.ranges
    
        # pop oldest entry if dict gets too large
        if len(self.scan_dict) > DICT_LIMIT:
            item = self.scan_dict.popitem(last=False) 

    def publish_hazard(self, details):
        time = self.get_clock().now()
        
        # Create the marker
        marker_msg = Marker()
        marker_msg.header.frame_id = 'map'
        marker_msg.header.stamp = time.to_msg()
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        
        # Marker ID
        marker_msg.id = int(details[0])

        src = 'camera_color_frame'
        dest = 'map'

        poseT = self.transform([details[1], details[2], 0.0], src, dest)
        
        if(poseT):
            marker_msg.pose.position.x = poseT.pose.position.x
            marker_msg.pose.position.y = poseT.pose.position.y
            marker_msg.pose.position.z = poseT.pose.position.z

            # Equivalent to 0,0,0 (roll, pitch, yaw)
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
                 
            self.hazard_pub.publish(marker_msg) 

    def object_listener(self, msg):
        
        START_ID = 13
        GOAL = 5

        if msg.data and msg.data[0] == START_ID and (not self.started):
            start_msg = Bool()
            start_msg.data = True
            self.start_pub.publish(start_msg)
            self.get_logger().info(f'Sent start msg')
            self.started = True

        elif msg.data and (msg.data[0] not in self.hazards) and msg.data[0] != START_ID:
            time = self.get_clock().now()
            self.get_object_position(msg.data, time)

            # Keep track of which hazards have been published
            self.hazards.append(msg.data[0])
            self.get_logger().info(f"Hazards: {self.hazards}")
            
            if len(self.hazards) >= GOAL:
                # Stop navigation
                start_msg = Bool()
                start_msg.data = False
                self.start_pub.publish(start_msg)

                self.get_logger().info(f"Go to goal pose")
                # Initial waypoint navigation/return to start
                end_msg = Bool()
                end_msg.data = True
                self.hazards_found.publish(end_msg)

    # Code based on https://husarion.com/tutorials/ros-tutorials/5-visual-object-recognition/#recognizing-objects
    def get_object_position(self, data, time):
        
        IMAGE_WIDTH = 640
        MAX_MEASUREMENT = 3
        MIN_RANGE = 315
        MAX_VARIATION = 90

        id = data[0]
        object_width = data[1]
        object_height = data[2]

        cv_homography = np.zeros(shape=(3, 3), dtype=np.float32)

        for i in range(9):
            cv_homography[i % 3, i // 3] = data[i + 3]

        in_pts = np.array([[[0, 0], [object_width, 0], [0, object_height], [object_width, object_height]]], dtype=np.float32)
        
        out_pts = cv.perspectiveTransform(in_pts, cv_homography)

        obj_x_pos = (out_pts[0][0][0] + out_pts[0][1][0] + out_pts[0][2][0] + out_pts[0][3][0])/4
        
        factor = IMAGE_WIDTH/2 - obj_x_pos

        # Robot can only see as far as -45 degrees which is equivalent to 315 degrees
        # Positive bound is +45 degress which gives us a variation of 90
        laser = MIN_RANGE + round(obj_x_pos/IMAGE_WIDTH*MAX_VARIATION)
        
        # LIDAR range values are from 0 - 359
        laser = laser%360
        # In construct robot gives 360 measurements where as rosbot gives 720
        laser = laser*2
        
        depth = min(self.get_depth(time, laser), MAX_MEASUREMENT) 

        # At either extreme of +-45degrees x == y
        # Where an object is straight ahead x == depth, y = 0
        xValue = depth - abs(factor/IMAGE_WIDTH)
        yValue = depth*(factor/IMAGE_WIDTH)

        details = [id, xValue, yValue]
        
        self.publish_hazard(details)

    def get_depth(self, timestamp, range):
      
        timestamp = round(timestamp.nanoseconds/1000000000,1)
        depth = 0.5

        if timestamp in self.scan_dict:
            depth = self.scan_dict[timestamp][range]

        elif timestamp - 0.1 in self.scan_dict:
            depth = self.scan_dict[timestamp - 0.1][range]

        elif self.scan_dict:
            # Access most recent entry
            depth = self.scan_dict.popitem()[1] 
            depth = depth[range]

        return depth
        
    def transform(self, srcPose, src, dest):
        
        poseT = None
        
        try:
            time = rclpy.time.Time()

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
            
            # Timeout for transform data
            timeout = rclpy.duration.Duration(seconds=0.8)

            poseT = self.tf_buffer.transform(pose, dest)
            
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().info(f'Could not gain current data for {src} to {dest}: {ex}')
        
        except tf2_ros.TransformException as ex2:
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
