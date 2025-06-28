#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - 2D to 3D Conversion Node
=========================================

Copyright (c) 2025 Ahmet Ceyhun Bilir
Author: Ahmet Ceyhun Bilir <ahmetceyhunbilir16@gmail.com>

This file is part of the GokHAS project, developed as a graduation thesis project.

License: MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

==============================================================================

2D to 3D Conversion Node
========================

This node converts 2D pixel coordinates from YOLO detections to 3D world coordinates
using ZED camera depth information and camera calibration parameters.
"""

# DEBUG FLAG - Set to True to enable point cloud publishing
DEBUG = False

import rospy
import numpy as np
import math
from sensor_msgs.msg import PointCloud2, PointField
from gokhas_perception.msg import YoloResult
from gokhas_perception.srv import PixelTo3D, PixelTo3DResponse
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class YoloTo3DDetector:
    def __init__(self):
        rospy.init_node('yolo_to_3d_detector', anonymous=True)
        
        # Subscribers
        self.pointcloud_sub = rospy.Subscriber(
            "/zedm/zed_node/point_cloud/cloud_registered", 
            PointCloud2, 
            self.pointcloud_callback
        )
        
        self.yolo_sub = rospy.Subscriber(
            "/zedm/zed_node/rgb/yolo_result", 
            YoloResult, 
            self.yolo_callback
        )
        
        # Publisher for detected point (only if DEBUG is enabled)
        if DEBUG:
            self.detected_point_pub = rospy.Publisher(
                "/detected_point_cloud",
                PointCloud2,
                queue_size=1
            )
            rospy.loginfo("DEBUG mode enabled - Point cloud publishing activated")
        else:
            self.detected_point_pub = None
            rospy.loginfo("DEBUG mode disabled - Point cloud publishing deactivated")
        
        # Store latest point cloud data and YOLO detection
        self.latest_pointcloud = None
        self.latest_yolo_detection = None
        
        # Create service
        self.pixel_to_3d_service = rospy.Service(
            'pixel_to_3d_angles',
            PixelTo3D,
            self.handle_pixel_to_3d_request
        )
        
        rospy.loginfo("YoloTo3DDetector initialized")
        rospy.loginfo("Service 'pixel_to_3d_angles' is now available")

    def pointcloud_callback(self, msg):
        """Store the latest point cloud data"""
        self.latest_pointcloud = msg
    
    def handle_pixel_to_3d_request(self, req):
        """Handle service request to convert pixel coordinates to joint angles"""
        response = PixelTo3DResponse()
        
        # Validate point cloud availability
        if self.latest_pointcloud is None:
            rospy.logwarn("No point cloud data available")
            response.joint1degree = 0
            response.joint2degree = 0
            return response
        
        # Use provided pixel coordinates (same for both autonomous and manual modes)
        center_x = req.pixel_x
        center_y = req.pixel_y
        
        mode_str = "Autonomous" if req.autonomous else "Manual"
        rospy.logdebug(f"{mode_str} mode: Processing pixel ({center_x}, {center_y})")
        
        # Get 3D point at coordinates
        point_3d = self.get_3d_point_at_pixel(center_x, center_y)
        
        if point_3d is not None:
            # Calculate angles from 3D coordinates
            h_angle, v_angle = self.calculate_angles_3d(point_3d)
            
            # Convert to integer degrees
            joint1degree = int(round(h_angle))
            joint2degree = int(round(v_angle))
            
            response.joint1degree = joint1degree
            response.joint2degree = joint2degree
            
            rospy.loginfo(f"3D→Angles: ({center_x},{center_y}) → J1={joint1degree}° J2={joint2degree}°")
            
            # Debug visualization (if enabled)
            if DEBUG:
                self.publish_detected_point(point_3d)
                
            return response
        else:
            rospy.logwarn(f"No valid 3D point at pixel ({center_x}, {center_y})")
            response.joint1degree = 0
            response.joint2degree = 0
            return response
    
    def get_3d_point_at_pixel(self, u, v):
        """Get 3D point at specific pixel coordinates from organized point cloud"""
        try:
            # Validate inputs
            if self.latest_pointcloud is None:
                return None
            
            # Check bounds
            if u < 0 or v < 0 or u >= self.latest_pointcloud.width or v >= self.latest_pointcloud.height:
                rospy.logwarn(f"Pixel ({u}, {v}) out of bounds [{self.latest_pointcloud.width}x{self.latest_pointcloud.height}]")
                return None
            
            # Method 1: Direct pixel lookup
            try:
                gen = pc2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), 
                                    skip_nans=True, uvs=[(u, v)])
                point_data = next(gen, None)
                
                if point_data is not None:
                    x, y, z = point_data
                    rospy.logdebug(f"Direct lookup SUCCESS: ({u},{v}) → 3D({x:.3f},{y:.3f},{z:.3f})")
                    return (x, y, z)
            except Exception as e:
                rospy.logdebug(f"Direct lookup failed: {e}")
            
            # Method 2: Search nearby pixels (5px radius)
            rospy.logdebug(f"Searching nearby pixels around ({u}, {v})")
            
            for radius in range(1, 6):  # Search in 5 pixel radius
                for du in range(-radius, radius + 1):
                    for dv in range(-radius, radius + 1):
                        test_u = u + du
                        test_v = v + dv
                        
                        # Check bounds
                        if (test_u < 0 or test_v < 0 or 
                            test_u >= self.latest_pointcloud.width or 
                            test_v >= self.latest_pointcloud.height):
                            continue
                        
                        try:
                            gen = pc2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), 
                                                skip_nans=True, uvs=[(test_u, test_v)])
                            point_data = next(gen, None)
                            
                            if point_data is not None:
                                x, y, z = point_data
                                rospy.logdebug(f"Nearby search SUCCESS: ({test_u},{test_v}) → 3D({x:.3f},{y:.3f},{z:.3f})")
                                return (x, y, z)
                        except:
                            continue
            
            rospy.logdebug(f"No valid 3D point found around ({u}, {v})")
            return None
            
        except Exception as e:
            rospy.logerr(f"Error in get_3d_point_at_pixel: {e}")
            return None
    
    def calculate_angles_3d(self, point_3d):
        """Calculate horizontal and vertical angles using 3D coordinates"""
        x, y, z = point_3d
        
        # ZED coordinate system: X forward, Y to the left, Z upward
        # Horizontal angle: atan2(-y, x) where x is forward, -y is right 
        # Negative y = right side = positive angle
        horizontal_angle_rad = math.atan2(-y, abs(x))
        horizontal_angle_deg = math.degrees(horizontal_angle_rad)
        
        # Vertical angle: atan2(z, x) where x is forward, z is up
        # Positive z = up = positive angle
        vertical_angle_rad = math.atan2(z, abs(x))
        vertical_angle_deg = math.degrees(vertical_angle_rad)
        
        return horizontal_angle_deg, vertical_angle_deg
    
    def publish_detected_point(self, point_3d):
        """Publish detected point as PointCloud2 (only if DEBUG is enabled)"""
        if not DEBUG or self.detected_point_pub is None:
            return
            
        try:
            x, y, z = point_3d
            
            # Create point cloud header
            header = Header()
            header.stamp = rospy.Time.now()
            # Check if we have a point cloud to get frame_id from
            if self.latest_pointcloud is not None:
                header.frame_id = self.latest_pointcloud.header.frame_id
            else:
                header.frame_id = "zed_left_camera_frame"  # Default frame
            
            # Define point fields
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
            ]
            
            # Create point data
            points = [[x, y, z]]
            
            # Create PointCloud2 message
            point_cloud_msg = pc2.create_cloud(header, fields, points)
            
            # Publish the point cloud
            self.detected_point_pub.publish(point_cloud_msg)
            
            rospy.loginfo(f"Published detected point: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            
        except Exception as e:
            rospy.logerr(f"Error publishing detected point: {e}")
    
    def yolo_callback(self, msg):
        """Process YOLO detection and convert to 3D (for non-service mode)"""
        self.latest_yolo_detection = msg
        
        if self.latest_pointcloud is None or len(msg.detections.detections) == 0:
            return
        
        # Process first detection only
        first_detection = msg.detections.detections[0]
        bbox = first_detection.bbox
        
        center_x = int(bbox.center.x)
        center_y = int(bbox.center.y)
        
        point_3d = self.get_3d_point_at_pixel(center_x, center_y)
        
        if point_3d is not None:
            x, y, z = point_3d
            euclidean_distance = np.sqrt(x*x + y*y + z*z)
            forward_distance = abs(x)
            
            h_angle, v_angle = self.calculate_angles_3d(point_3d)
            
            print("=" * 50)
            print(f"3D Detection Result:")
            print(f"  Position: x={x:.3f}m (forward), y={y:.3f}m (left), z={z:.3f}m (up)")
            print(f"  Distance: {forward_distance:.3f}m forward, {euclidean_distance:.3f}m total")
            print(f"  Angles: H={h_angle:+.2f}° ({'RIGHT' if h_angle > 0 else 'LEFT' if h_angle < 0 else 'CENTER'}), " +
                  f"V={v_angle:+.2f}° ({'UP' if v_angle > 0 else 'DOWN' if v_angle < 0 else 'CENTER'})")
            print("=" * 50)
            
            if DEBUG:
                self.publish_detected_point(point_3d)
    
    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = YoloTo3DDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass



