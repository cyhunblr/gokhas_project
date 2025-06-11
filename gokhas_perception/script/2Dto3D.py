#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# DEBUG FLAG - Set to True to enable point cloud publishing
DEBUG = False

import rospy
import numpy as np
import math
from sensor_msgs.msg import PointCloud2, PointField
from gokhas_perception.msg import YoloResult
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
        
        # Store latest point cloud data
        self.latest_pointcloud = None
        rospy.loginfo("YoloTo3DDetector initialized")

    def pointcloud_callback(self, msg):
        """Store the latest point cloud data"""
        self.latest_pointcloud = msg
    
    def get_3d_point_at_pixel(self, u, v):
        """Get 3D point at specific pixel coordinates from organized point cloud"""
        try:
            # Extract x,y,z directly from raw data
            # Get point data directly from the point cloud at the offset
            gen = pc2.read_points(self.latest_pointcloud, field_names=("x", "y", "z"), 
                                skip_nans=True, uvs=[(u, v)])
            point_data = next(gen, None)
            
            if point_data is not None:
                x, y, z = point_data
                return (x, y, z)
            else:
                return None
            
        except Exception as e:
            rospy.logerr(f"Error getting 3D point: {e}")
            return None
    
    def calculate_angles_3d(self, point_3d):
        """Calculate horizontal and vertical angles using 3D coordinates"""
        x, y, z = point_3d
        
        # ZED koordinat sistemi: X ileri, Y sola, Z yukarı
        # Horizontal angle: atan2(-y, x) where x is forward, -y is right (pozitif sağ)
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
            header.frame_id = self.latest_pointcloud.header.frame_id
            
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
        """Process YOLO detection and convert to 3D"""
        
        if self.latest_pointcloud is None:
            rospy.logwarn("No point cloud data available")
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
        point_3d = self.get_3d_point_at_pixel(center_x, center_y)
        
        if point_3d is not None:
            x, y, z = point_3d
            
            # ZED koordinat sistemi: X ileri, Y sola, Z yukarı
            euclidean_distance = np.sqrt(x*x + y*y + z*z)
            forward_distance = abs(x)  # İleri mesafe
            
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
            
            # Publish the detected point as PointCloud2 (only if DEBUG is enabled)
            if DEBUG:
                self.publish_detected_point(point_3d)
            
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



