#!/usr/bin/env python  

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from visualization_msgs.msg import Marker

import math
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
            details = [int(msg.data[0])]
            self.publish_hazard(details)
    
    # Code based on https://husarion.com/tutorials/ros-tutorials/5-visual-object-recognition/#recognizing-objects
    def get_object_position(self, data):
        OBJECT_TO_FOLLOW = 3
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
            
            #cv::Mat cvHomography(3, 3, CV_32F) (instead just use numpy array)
            
            cv
            std::vector<cv::Point2f> inPts, outPts;
            switch (id)
            {
            case OBJECT_TO_FOLLOW:

                // Matrix completion
                for(int i=0; i<9; i++){
                    cvHomography.at<float>(i%3, i/3) = object->data[i+3];
                }

                // Save corners to vector
                inPts.push_back(cv::Point2f(0, 0));
                inPts.push_back(cv::Point2f(objectWidth, 0));
                inPts.push_back(cv::Point2f(0, objectHeight));
                inPts.push_back(cv::Point2f(objectWidth, objectHeight));
                cv::perspectiveTransform(inPts, outPts, cvHomography);

                obj_x_pos = (outPts.at(0).x + outPts.at(1).x + outPts.at(2).x + outPts.at(3).x) / 4;
                ang_vel = ANGULAR_GAIN*(CAMERA_WIDTH/2 - obj_x_pos);

                // Set angular speed
                if(ang_vel <= -MIN_ANG_VEL || ang_vel >= MIN_ANG_VEL){
                    vel_msg.angular.z = std::max(-MAX_ANG_VEL, std::min(ang_vel, MAX_ANG_VEL));
                }
                ROS_INFO("id: %d\t ang_vel: %f", id, vel_msg.angular.z);
                break;
            }
        }

        vel_pub.publish(vel_msg);

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
