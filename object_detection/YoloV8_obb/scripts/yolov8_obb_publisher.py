#!/usr/bin/env python3

from ultralytics import YOLO
import os
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('/Users/anmolchalise/Downloads/moveit2_obb/src/yolov8_obb/scripts/best.pt')

        self.yolov8_inference = Yolov8Inference()
        
        # Camera intrinsics (will be updated from camera_info)
        self.fx = 0.0  # focal length x
        self.fy = 0.0  # focal length y
        self.cx = 0.0  # principal point x
        self.cy = 0.0  # principal point y
        
        # Subscribe to camera info to get intrinsics
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10)
            
        # Using message filters to synchronize RGB and depth images
        self.rgb_sub = Subscriber(self, Image, '/image_raw')
        self.depth_sub = Subscriber(self, Image, '/depth_image')
        
        # Synchronize messages with a 0.1 second tolerance
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.ts.registerCallback(self.synchronized_callback)

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
    
    def camera_info_callback(self, msg):
        # Extract camera intrinsics from camera_info message
        self.fx = msg.k[0]  # Focal length x
        self.fy = msg.k[4]  # Focal length y
        self.cx = msg.k[2]  # Principal point x
        self.cy = msg.k[5]  # Principal point y
        self.get_logger().info(f"Camera intrinsics updated: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def synchronized_callback(self, rgb_msg, depth_msg):
        # Convert RGB image
        img = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        
        # Convert depth image
        # Note: The encoding might be different based on your depth camera (16UC1, 32FC1, etc.)
        try:
            depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")
            return
            
        # Run inference on RGB image
        results = self.model(img, conf=0.90)
        
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()
        
        for r in results:
            if(r.obb is not None):
                boxes = r.obb
                for box in boxes:
                    self.inference_result = InferenceResult()
                    b = box.xyxyxyxy[0].to('cpu').detach().numpy().copy()
                    c = box.cls
                    self.inference_result.class_name = self.model.names[int(c)]
                    
                    # Get 2D coordinates from the detection
                    a = b.reshape(1,8)
                    points_2d = a[0].reshape(4, 2)
                    
                    # Calculate center point of the detected object
                    center_x = np.mean(points_2d[:, 0])
                    center_y = np.mean(points_2d[:, 1])
                    
                    # Get depth at the center point (convert to integer indices)
                    center_x_int = int(center_x)
                    center_y_int = int(center_y)
                    
                    # Make sure coordinates are within image bounds
                    if (0 <= center_x_int < depth_img.shape[1] and 
                        0 <= center_y_int < depth_img.shape[0]):
                        
                        # Get depth value (may need conversion based on your depth image format)
                        depth_value = depth_img[center_y_int, center_x_int]
                        
                        # If depth is in millimeters, convert to meters
                        if depth_img.dtype == np.uint16:
                            depth_value = depth_value / 1000.0
                            
                        # Check if depth is valid (not 0 or inf)
                        if depth_value > 0 and np.isfinite(depth_value):
                            # Perform perspective transformation for each corner point
                            points_3d = []
                            for point in points_2d:
                                # Perspective transformation from 2D to 3D
                                x = (point[0] - self.cx) * depth_value / self.fx
                                y = (point[1] - self.cy) * depth_value / self.fy
                                z = depth_value
                                points_3d.extend([x, y, z])
                                
                            # Store original 2D coordinates
                            self.inference_result.coordinates = copy.copy(a[0].tolist())
                            
                            # Store 3D coordinates (x,y,z for each corner)
                            self.inference_result.coordinates_3d = points_3d
                            
                            # Store depth value
                            self.inference_result.depth = depth_value
                            
                            self.yolov8_inference.yolov8_inference.append(self.inference_result)
                        else:
                            self.get_logger().info(f"Invalid depth value: {depth_value}")
                    else:
                        self.get_logger().info(f"Center point out of bounds: ({center_x_int}, {center_y_int})")
            else:
                self.get_logger().info(f"No results detected")
                
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()
        
        # Visualize results
        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)
        self.img_pub.publish(img_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()