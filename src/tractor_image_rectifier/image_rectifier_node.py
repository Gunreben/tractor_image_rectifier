#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class ImageRectifierNode(Node):
    def __init__(self):
        super().__init__('image_rectifier_node')
        
        # Declare parameters
        self.declare_parameter('reliable_qos', False)
        self.declare_parameter('input_topic', '/camera/rear_right/image_raw')
        self.declare_parameter('output_topic', '/camera/rear_right/image_rect')
        self.declare_parameter('camera_info_topic', '/camera/rear_right/camera_info')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('balance', 0.0)  # For fisheye undistortion
        
        # Get parameters
        reliable_qos = self.get_parameter('reliable_qos').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.balance = self.get_parameter('balance').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Storage for camera parameters
        self.K = None  # Camera matrix
        self.D = None  # Distortion coefficients
        self.map1 = None  # Undistortion map x
        self.map2 = None  # Undistortion map y
        self.img_size = None  # Image size
        
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
        
        # Create subscribers and publishers based on topic type
        if self.use_compressed:
            # Ensure input topic doesn't already end with /compressed
            compressed_input = input_topic
            if not compressed_input.endswith('/compressed'):
                compressed_input = f"{input_topic}/compressed"
                
            self.image_sub = self.create_subscription(
                CompressedImage,
                compressed_input,
                self.image_callback,
                qos_profile)
                
            # Ensure output topic doesn't already end with /compressed
            compressed_output = output_topic
            if not compressed_output.endswith('/compressed'):
                compressed_output = f"{output_topic}/compressed"
                
            self.rectified_pub = self.create_publisher(
                CompressedImage,
                compressed_output,
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
            camera_info_topic,
            self.camera_info_callback,
            qos_profile)
            
        self.get_logger().info(f'Image rectifier node initialized with {"compressed" if self.use_compressed else "uncompressed"} images')
        self.get_logger().info(f'Input topic: {self.image_sub.topic_name}')
        self.get_logger().info(f'Output topic: {self.rectified_pub.topic_name}')
        self.get_logger().info(f'Camera info topic: {camera_info_topic}')
        self.get_logger().info(f'QoS: {"reliable" if reliable_qos else "best effort"}')
        self.get_logger().info(f'Using fisheye equidistant distortion model with balance: {self.balance}')
        
        # Flag to check if camera info is received
        self.camera_info_received = False
        
    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            # Extract camera matrix and distortion coefficients
            self.K = np.array(msg.k).reshape(3, 3)
            self.D = np.array(msg.d)
            
            # Check distortion model
            if msg.distortion_model != "equidistant":
                self.get_logger().warn(f"Expected distortion model 'equidistant', but got '{msg.distortion_model}'. Will try to proceed anyway.")
            
            # Store image dimensions for map creation
            self.img_size = (msg.width, msg.height)
            
            # Create undistortion maps for fisheye/equidistant model
            Knew = self.K.copy()
            if self.balance > 0:
                # Adjust camera matrix based on balance parameter
                # Lower focal length = more of the original image preserved
                Knew[(0, 1), (0, 1)] = self.K[(0, 1), (0, 1)] * (1 - self.balance)
            
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                self.K, self.D[:4], np.eye(3), Knew, self.img_size, cv2.CV_16SC2)
            
            self.camera_info_received = True
            self.get_logger().info('Camera info received and undistortion maps created')
            
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
            
            # Update maps if image size doesn't match
            if self.img_size != (cv_image.shape[1], cv_image.shape[0]):
                self.img_size = (cv_image.shape[1], cv_image.shape[0])
                self.get_logger().info(f'Updating undistortion maps for image size: {self.img_size}')
                
                Knew = self.K.copy()
                if self.balance > 0:
                    Knew[(0, 1), (0, 1)] = self.K[(0, 1), (0, 1)] * (1 - self.balance)
                
                self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
                    self.K, self.D[:4], np.eye(3), Knew, self.img_size, cv2.CV_16SC2)
            
            # Rectify the image using the fisheye undistortion maps
            rectified_image = cv2.remap(cv_image, self.map1, self.map2, 
                                       interpolation=cv2.INTER_LINEAR, 
                                       borderMode=cv2.BORDER_CONSTANT)
            
            # Convert back to ROS message
            if self.use_compressed:
                rectified_msg = self.bridge.cv2_to_compressed_imgmsg(rectified_image)
                # Copy the format from the original message if available
                if hasattr(msg, 'format'):
                    rectified_msg.format = msg.format
            else:
                rectified_msg = self.bridge.cv2_to_imgmsg(rectified_image)
                # Copy the encoding from the original message
                if hasattr(msg, 'encoding'):
                    rectified_msg.encoding = msg.encoding
                    
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