#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from image_geometry import PinholeCameraModel
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class ImageRectifierNode(Node):
    def __init__(self):
        super().__init__('image_rectifier_node')
        
        # Declare parameters
        self.declare_parameter('reliable_qos', False)
        self.declare_parameter('input_topic', 'input/image_raw')
        self.declare_parameter('output_topic', 'output/image_rect')
        
        # Get parameters
        reliable_qos = self.get_parameter('reliable_qos').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize camera model
        self.camera_model = PinholeCameraModel()
        
        # Set up QoS profile
        if reliable_qos:
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10
            )
        else:
            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10
            )
        
        # Check if input topic ends with /compressed
        self.use_compressed = input_topic.endswith('/compressed')
        
        # Create subscribers and publishers based on topic type
        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage,
                input_topic,
                self.image_callback,
                qos_profile)
                
            self.rectified_pub = self.create_publisher(
                CompressedImage,
                output_topic + '/compressed',
                qos_profile)
        else:
            self.image_sub = self.create_subscription(
                Image,
                input_topic,
                self.image_callback,
                qos_profile)
                
            self.rectified_pub = self.create_publisher(
                Image,
                output_topic,
                qos_profile)
            
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/side_right/camera_info',
            self.camera_info_callback,
            qos_profile)
            
        self.get_logger().info(f'Image rectifier node initialized with {"compressed" if self.use_compressed else "uncompressed"} images and {"reliable" if reliable_qos else "best effort"} QoS')
        
        # Flag to check if camera info is received
        self.camera_info_received = False
        
    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_model.from_camera_info(msg)
            self.camera_info_received = True
            self.get_logger().info('Camera info received and processed')
            
    def image_callback(self, msg):
        if not self.camera_info_received:
            self.get_logger().warn('Camera info not received yet, skipping frame')
            return
            
        try:
            # Convert image to OpenCV format
            if self.use_compressed:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg)
            
            # Rectify the image
            rectified_image = np.zeros_like(cv_image)
            self.camera_model.rectify_image(cv_image, rectified_image)
            
            # Convert back to ROS message
            if self.use_compressed:
                rectified_msg = self.bridge.cv2_to_compressed_imgmsg(rectified_image)
            else:
                rectified_msg = self.bridge.cv2_to_imgmsg(rectified_image)
            rectified_msg.header = msg.header
            
            # Publish the rectified image
            self.rectified_pub.publish(rectified_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageRectifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 