#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# DEBUG FLAG - Set to True to enable visualization
DEBUG = False

import rospy
import numpy as np
import math
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from gokhas_perception.msg import YoloResult
from gokhas_perception.srv import PixelTo3D, PixelTo3DResponse
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class YoloTo3DDetector:
    def __init__(self):
        rospy.init_node('yolo_to_3d_detector', anonymous=True)
        
        # Initialize CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribers
        self.depth_sub = rospy.Subscriber(
            "/zedm/zed_node/depth/depth_registered", 
            Image, 
            self.depth_callback
        )
        
        self.camera_info_sub = rospy.Subscriber(
            "/zedm/zed_node/rgb/camera_info",
            CameraInfo,
            self.camera_info_callback
        )
        
        self.yolo_sub = rospy.Subscriber(
            "/zedm/zed_node/rgb/yolo_result", 
            YoloResult, 
            self.yolo_callback
        )
        
        # Publisher for visualization (only if DEBUG is enabled)
        if DEBUG:
            self.debug_image_pub = rospy.Publisher(
                "/debug_image",
                Image,
                queue_size=1
            )
            rospy.loginfo("DEBUG mode enabled - Visualization activated")
        else:
            self.debug_image_pub = None
            rospy.loginfo("DEBUG mode disabled - Visualization deactivated")
        
        # Store latest data
        self.latest_depth_image = None
        self.latest_camera_info = None
        self.latest_yolo_detection = None
        
        # Extract camera matrix when camera info is received
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Create service
        self.pixel_to_3d_service = rospy.Service(
            'pixel_to_3d_angles',
            PixelTo3D,
            self.handle_pixel_to_3d_request
        )
        
        rospy.loginfo("YoloTo3DDetector initialized")
        rospy.loginfo("Service 'pixel_to_3d_angles' is now available")

    def camera_info_callback(self, msg):
        """Store camera information and extract camera matrix"""
        self.latest_camera_info = msg
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        
    def depth_callback(self, msg):
        """Store the latest depth image"""
        self.latest_depth_image = msg
    
    def handle_pixel_to_3d_request(self, req):
        """Handle service request"""
        response = PixelTo3DResponse()
        
        if self.latest_depth_image is None or self.latest_camera_info is None:
            rospy.logwarn("No depth image or camera info available to process service request")
            # Return zeros for failure
            response.joint1degree = 0
            response.joint2degree = 0
            return response
        
        # CASE 1: Autonomous mode - use YOLO detection
        if req.autonomous:
            if self.latest_yolo_detection is None or len(self.latest_yolo_detection.detections.detections) == 0:
                rospy.logwarn("No YOLO detection available for autonomous mode")
                response.joint1degree = 0
                response.joint2degree = 0
                return response
                
            # Use latest YOLO detection
            first_detection = self.latest_yolo_detection.detections.detections[0]
            bbox = first_detection.bbox
            center_x = int(bbox.center.x)
            center_y = int(bbox.center.y)
    
        # CASE 2: Manual mode - use provided pixel coordinates
        else:
            center_x = req.pixel_x
            center_y = req.pixel_y
        
        # Get 3D point using camera model
        point_3d = self.get_3d_point_from_depth(center_x, center_y)
        
        if point_3d is not None:
            # Calculate angles from 3D coordinates
            h_angle, v_angle = self.calculate_angles_3d(point_3d)
            
            # Convert angles to int16 (keeping the original sign)
            # int16 range is -32768 to 32767 which is more than enough for our angles
            joint1degree = int(round(h_angle))
            joint2degree = int(round(v_angle))
            
            response.joint1degree = joint1degree
            response.joint2degree = joint2degree
            
            # Log for debugging
            rospy.loginfo(f"Service request: autonomous={req.autonomous}")
            if req.autonomous:
                rospy.loginfo(f"Using YOLO detection at ({center_x}, {center_y})")
            else:
                rospy.loginfo(f"Using provided pixel coordinates ({center_x}, {center_y})")
            
            rospy.loginfo(f"Joint angles - joint1: {joint1degree}, joint2: {joint2degree}")
            
            # Debug visualization (if enabled)
            if DEBUG:
                self.visualize_point(center_x, center_y, point_3d)
                
            return response
        else:
            rospy.logwarn("No valid 3D point found at the specified pixel coordinates")
            response.joint1degree = 0
            response.joint2degree = 0
            return response
    
    def get_3d_point_from_depth(self, u, v):
        """
        Get 3D point at pixel (u,v) using depth image and camera intrinsics
        
        Returns:
            tuple (x, y, z) - 3D point in camera frame, or None if depth is invalid
        """
        try:
            # Convert depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth_image, desired_encoding="32FC1")
            
            # Check if coordinates are within image bounds
            height, width = depth_image.shape
            if u < 0 or u >= width or v < 0 or v >= height:
                rospy.logwarn(f"Pixel coordinates ({u}, {v}) are outside image bounds ({width}x{height})")
                return None
            
            # Get depth at the specified pixel (in meters)
            depth_value = depth_image[v, u]
            
            # Check for invalid depth values
            if not np.isfinite(depth_value) or depth_value <= 0:
                rospy.logwarn(f"Invalid depth value at pixel ({u}, {v}): {depth_value}")
                return None
            
            # Get camera intrinsics
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            
            # Calculate 3D coordinates using pinhole camera model
            # Z is the depth
            z = depth_value
            # X = (u - cx) * Z / fx
            x = (u - cx) * z / fx
            # Y = (v - cy) * Z / fy
            y = (v - cy) * z / fy
            
            # ZED camera's coordinate system: X (right), Y (down), Z (forward)
            # Convert to the coordinate system used in our calculations: X (forward), Y (left), Z (up)
            # This is a conversion from ZED's coordinate system to our system
            x_world = z      # Forward
            y_world = -x     # Left
            z_world = -y     # Up
            
            return (x_world, y_world, z_world)
            
        except Exception as e:
            rospy.logerr(f"Error getting 3D point from depth: {e}")
            return None
    
    def calculate_angles_3d(self, point_3d):
        """Calculate horizontal and vertical angles using 3D coordinates"""
        x, y, z = point_3d
        
        # Coordinate system: X forward, Y to the left, Z upward
        # Horizontal angle: atan2(-y, x) where x is forward, -y is right 
        # Negative y = right side = positive angle
        horizontal_angle_rad = math.atan2(-y, abs(x))
        horizontal_angle_deg = math.degrees(horizontal_angle_rad)
        
        # Vertical angle: atan2(z, x) where x is forward, z is up
        # Positive z = up = positive angle
        vertical_angle_rad = math.atan2(z, abs(x))
        vertical_angle_deg = math.degrees(vertical_angle_rad)
        
        return horizontal_angle_deg, vertical_angle_deg
    
    def calculate_pixel_angle(self, u1, v1, u2, v2):
        """
        Calculate the horizontal and vertical angles between two pixels using camera intrinsics.

        Args:
            u1, v1 (int): Pixel coordinates of the first point.
            u2, v2 (int): Pixel coordinates of the second point.

        Returns:
            tuple: (horizontal_angle, vertical_angle) in degrees.
        """
        try:
            # Ensure camera matrix is available
            if self.camera_matrix is None:
                rospy.logerr("Camera matrix is not available. Cannot calculate angles.")
                return None, None

            # Extract focal lengths from the camera matrix
            fx = self.camera_matrix[0, 0]  # Focal length in x direction
            fy = self.camera_matrix[1, 1]  # Focal length in y direction

            # Calculate horizontal and vertical angles
            horizontal_angle_rad = math.atan2(u2 - u1, fx)
            vertical_angle_rad = math.atan2(v2 - v1, fy)

            # Convert radians to degrees
            horizontal_angle_deg = math.degrees(horizontal_angle_rad)
            vertical_angle_deg = math.degrees(vertical_angle_rad)

            return horizontal_angle_deg, vertical_angle_deg

        except Exception as e:
            rospy.logerr(f"Error calculating pixel angles: {e}")
            return None, None

    def visualize_point(self, pixel_x, pixel_y, point_3d):
        """Visualize detected point (for debugging)"""
        if not DEBUG or self.debug_image_pub is None:
            return
            
        try:
            # Convert depth image to color visualization
            depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth_image, desired_encoding="32FC1")
            
            # Normalize depth for visualization
            depth_colormap = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(depth_colormap, cv2.COLORMAP_JET)
            
            # Draw marker at the specified pixel
            cv2.drawMarker(depth_colormap, (pixel_x, pixel_y), (255, 255, 255), 
                          markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            
            # Add text with 3D coordinates and angles
            x, y, z = point_3d
            h_angle, v_angle = self.calculate_angles_3d(point_3d)
            
            cv2.putText(depth_colormap, f"3D: ({x:.2f}, {y:.2f}, {z:.2f})", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(depth_colormap, f"Angles: H={h_angle:.1f}°, V={v_angle:.1f}°", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Publish the visualization
            debug_msg = self.bridge.cv2_to_imgmsg(depth_colormap, encoding="bgr8")
            self.debug_image_pub.publish(debug_msg)
            
            rospy.loginfo(f"Published visualization for point at ({pixel_x}, {pixel_y})")
            
        except Exception as e:
            rospy.logerr(f"Error creating visualization: {e}")
    
    def yolo_callback(self, msg):
        """Process YOLO detection"""
        # Store latest YOLO detection for service use
        self.latest_yolo_detection = msg
        
        if self.latest_depth_image is None or self.latest_camera_info is None:
            rospy.logwarn("No depth image or camera info available for YOLO processing")
            return
        
        if len(msg.detections.detections) == 0:
            return
        
        # Process first bounding box only
        first_detection = msg.detections.detections[0]
        bbox = first_detection.bbox
        
        # Get center coordinates directly
        center_x = int(bbox.center.x)
        center_y = int(bbox.center.y)
        
        # Get 3D point at center coordinates
        point_3d = self.get_3d_point_from_depth(center_x, center_y)
        
        if point_3d is not None:
            x, y, z = point_3d
            
            # Calculate distance
            euclidean_distance = np.sqrt(x*x + y*y + z*z)
            forward_distance = abs(x)  # forward distance is the absolute value of x
            
            # Calculate angles from 3D coordinates
            h_angle, v_angle = self.calculate_angles_3d(point_3d)
            
            print("=" * 50)
            print(f"3D Detection Result:")
            print(f"  Position: x={x:.3f}m (forward), y={y:.3f}m (left), z={z:.3f}m (up)")
            print(f"  Forward distance: {forward_distance:.3f} meters")
            print(f"  Total distance: {euclidean_distance:.3f} meters")
            print(f"")
            print(f"Angles from center:")
            print(f"  Horizontal: {h_angle:+.2f}° ({'RIGHT' if h_angle > 0 else 'LEFT' if h_angle < 0 else 'CENTER'})")
            print(f"  Vertical:   {v_angle:+.2f}° ({'UP' if v_angle > 0 else 'DOWN' if v_angle < 0 else 'CENTER'})")
            print("=" * 50)
            
            # Debug visualization (if enabled)
            if DEBUG:
                self.visualize_point(center_x, center_y, point_3d)
            
        else:
            rospy.logwarn("No valid 3D point found at detection center")
    
    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = YoloTo3DDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass



