#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - Legacy Autonomous Targeting System
===================================================

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

Legacy Autonomous Targeting System for GokHAS Project (Service-Based)
=====================================================================

This is the legacy autonomous targeting system that uses ROS services to convert
2D pixel coordinates to 3D world angles. It includes bbox filtering and smooth
target tracking for improved stability. This version is kept for compatibility
with service-based 3D conversion systems.

Note: For new implementations, use autonomous_targeting_2d.py or autonomous_targeting_3d.py

Features:
- Service-based 3D coordinate conversion
- Bounding box filtering and smoothing
- Target tracking with stability control
- Legacy service interface compatibility
"""

import rospy
from gokhas_perception.msg import YoloResult
from gokhas_perception.srv import PixelTo3D, PixelTo3DRequest
from gokhas_communication.msg import JointMessage
from std_msgs.msg import Bool

class AutonomousTargetingNode:
    """
    Legacy Autonomous Targeting Node with service-based 3D conversion
    
    Provides autonomous targeting using ROS services for 3D calculations
    """
    def __init__(self):
        rospy.init_node('autonomous_targeting_node', anonymous=True)
        
        # Publishers
        self.joint_pub = rospy.Publisher('/gokhas/joint_commands', JointMessage, queue_size=1)
        
        # Subscribers
        self.yolo_sub = rospy.Subscriber('/zedm/zed_node/rgb/yolo_result', YoloResult, self.yolo_callback)
        self.autonomous_mode_sub = rospy.Subscriber('/gokhas/autonomous_mode', Bool, self.autonomous_mode_callback)
        
        # Service client for 2D to 3D conversion
        self.pixel_to_3d_client = None
        
        # State variables
        self.autonomous_mode_active = False
        self.target_id = 0  # ID of the target to track (person = 0 in YOLO COCO dataset)
        
        # Targeting control parameters
        self.angle_tolerance = 5.0  # Degrees - stop moving if target is within this range (increased from 2.0)
        self.min_command_interval = 0.5  # Seconds - minimum time between joint commands
        self.last_command_time = 0.0
        self.last_joint_angles = {'j1': 0, 'j2': 0}  # Track last sent angles
        
        # Bounding box filtering parameters
        self.bbox_filter_alpha = 0.7  # Smoothing factor (0.5-0.9, higher = more smoothing)
        self.filtered_center_x = None
        self.filtered_center_y = None
        self.min_bbox_movement = 10  # Minimum pixel movement to trigger update
        
        # Initialize service client
        self.init_service_client()
        
        rospy.loginfo("Autonomous Targeting Node initialized")
        rospy.loginfo(f"Tracking target ID: {self.target_id}")
    
    def init_service_client(self):
        """Initialize service client for pixel to 3D conversion"""
        try:
            rospy.loginfo("Waiting for pixel_to_3d_angles service...")
            rospy.wait_for_service('pixel_to_3d_angles', timeout=10.0)
            self.pixel_to_3d_client = rospy.ServiceProxy('pixel_to_3d_angles', PixelTo3D)
            rospy.loginfo("Successfully connected to pixel_to_3d_angles service")
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to connect to pixel_to_3d_angles service: {e}")
            self.pixel_to_3d_client = None
    
    def autonomous_mode_callback(self, msg):
        """Handle autonomous mode activation/deactivation"""
        self.autonomous_mode_active = msg.data
        if self.autonomous_mode_active:
            rospy.loginfo("Autonomous mode ACTIVATED - Starting target tracking")
        else:
            rospy.loginfo("Autonomous mode DEACTIVATED - Stopping target tracking")
    
    def yolo_callback(self, msg):
        """Process YOLO detection results for autonomous targeting"""
        if not self.autonomous_mode_active:
            return
        
        # Check and reinitialize service client if needed
        if self.pixel_to_3d_client is None:
            rospy.logwarn("2D to 3D service not available - attempting to reconnect...")
            self.init_service_client()
            if self.pixel_to_3d_client is None:
                rospy.logwarn("Failed to reconnect to 2D to 3D service")
                return
        
        # Find target with specified ID
        target_detection = self.find_target_by_id(msg, self.target_id)
        
        if target_detection is None:
            rospy.logdebug(f"No target with ID {self.target_id} found in current frame")
            return
        
        # Get pixel coordinates from detection
        bbox = target_detection.bbox
        raw_center_x = int(bbox.center.x)
        raw_center_y = int(bbox.center.y)
        
        # Apply smoothing filter to bounding box center
        if self.filtered_center_x is None:
            # First detection - initialize filter
            self.filtered_center_x = float(raw_center_x)
            self.filtered_center_y = float(raw_center_y)
        else:
            # Apply exponential moving average filter
            self.filtered_center_x = (self.bbox_filter_alpha * self.filtered_center_x + 
                                    (1.0 - self.bbox_filter_alpha) * float(raw_center_x))
            self.filtered_center_y = (self.bbox_filter_alpha * self.filtered_center_y + 
                                    (1.0 - self.bbox_filter_alpha) * float(raw_center_y))
        
        # Use filtered coordinates
        center_x = int(self.filtered_center_x)
        center_y = int(self.filtered_center_y)
        
        # Check if movement is significant enough
        pixel_movement = ((float(raw_center_x) - float(center_x))**2 + (float(raw_center_y) - float(center_y))**2)**0.5
        if pixel_movement < self.min_bbox_movement:
            rospy.logdebug(f"Small bounding box movement: {pixel_movement:.1f} pixels - Using filtered center")
        
        # Debug: Log detection info
        rospy.loginfo(f"Target at ({center_x}, {center_y}) - Image center: (960, 540)")
        
        # Additional validation
        if center_x < 0 or center_y < 0 or center_x > 1920 or center_y > 1080:
            rospy.logwarn(f"Suspicious coordinates: ({center_x}, {center_y}) - out of bounds")
            return
        
        # Call 2D to 3D service
        try:
            request = PixelTo3DRequest()
            request.autonomous = True
            request.pixel_x = center_x
            request.pixel_y = center_y
            
            response = self.pixel_to_3d_client(request)
            
            # Validate response - skip if no valid 3D point found
            if response.joint1degree == 0 and response.joint2degree == 0:
                rospy.logdebug(f"No valid 3D point at ({center_x}, {center_y}) - skipping")
                return
            
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed: {e} - attempting reconnect")
            self.init_service_client()
            if self.pixel_to_3d_client is not None:
                try:
                    response = self.pixel_to_3d_client(request)
                except rospy.ServiceException as e2:
                    rospy.logerr(f"Service call failed after reconnect: {e2}")
                    return
            else:
                return
        except Exception as e:
            rospy.logerr(f"Unexpected service error: {e}")
            return
        
        # Check if target is within tolerance (dead zone)
        if (abs(response.joint1degree) <= self.angle_tolerance and 
            abs(response.joint2degree) <= self.angle_tolerance):
            rospy.logdebug(f"Target within tolerance: J1={response.joint1degree}° J2={response.joint2degree}°")
            return
        
        # Rate limiting - prevent too frequent commands
        current_time = rospy.get_time()
        if (current_time - self.last_command_time) < self.min_command_interval:
            return
        
        # Check if angles have significantly changed
        angle_change_j1 = abs(response.joint1degree - self.last_joint_angles['j1'])
        angle_change_j2 = abs(response.joint2degree - self.last_joint_angles['j2'])
        
        if (angle_change_j1 < 1.0 and angle_change_j2 < 1.0):
            rospy.logdebug(f"Small angle change: ΔJ1={angle_change_j1:.1f}° ΔJ2={angle_change_j2:.1f}°")
            return
        
        # Update tracking variables
        self.last_command_time = current_time
        self.last_joint_angles['j1'] = response.joint1degree
        self.last_joint_angles['j2'] = response.joint2degree
        
        # Create and publish joint command
        joint_msg = JointMessage()
        joint_msg.control_bits = 0  # Initialize control bits
        
        # Joint 1 (Horizontal) - Handle sign and value
        if response.joint1degree < 0:
            joint_msg.control_bits |= 0b00000010  # Set j1p_sign bit (bit 1) for negative
            joint_msg.j1p = min(abs(response.joint1degree), 135)  # Clamp to 0-135 range
        else:
            joint_msg.j1p = min(response.joint1degree, 135)  # Clamp to 0-135 range
        
        # Joint 2 (Vertical) - Handle sign and value  
        if response.joint2degree < 0:
            joint_msg.control_bits |= 0b00000100  # Set j2p_sign bit (bit 2) for negative
            joint_msg.j2p = min(abs(response.joint2degree), 135)  # Clamp to 0-135 range
        else:
            joint_msg.j2p = min(response.joint2degree, 135)  # Clamp to 0-135 range
        
        # Set speed and airsoft parameters
        joint_msg.j1s = 50  # Default speed for autonomous movement
        joint_msg.j2s = 50  # Default speed for autonomous movement
        joint_msg.ap = 0    # No airsoft activation in autonomous mode
        
        # Publish joint command
        self.joint_pub.publish(joint_msg)
        
        rospy.loginfo(f"→ TARGETING: J1={response.joint1degree}° J2={response.joint2degree}° | MSG: j1p={joint_msg.j1p} j2p={joint_msg.j2p} bits={joint_msg.control_bits:08b}")
    
    def find_target_by_id(self, yolo_result, target_id):
        """Find detection with specified class ID"""        
        for detection in yolo_result.detections.detections:
            for result in detection.results:
                if result.id == target_id:
                    rospy.logdebug(f"Found target ID {target_id} with score {result.score:.3f}")
                    return detection
        return None
    
    def run(self):
        """Main loop"""
        rospy.loginfo("Autonomous Targeting Node is running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        node = AutonomousTargetingNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Autonomous Targeting Node stopped")
