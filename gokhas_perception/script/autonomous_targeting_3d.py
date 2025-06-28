#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - Autonomous 3D Targeting System
===============================================

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

Autonomous 3D Targeting System for GokHAS Project
=================================================

This script implements a 3D autonomous targeting system that converts 2D pixel coordinates
to real-world 3D angles using camera calibration parameters. It provides precise targeting
by calculating the actual angular displacement needed to center detected objects in 3D space.

Features:
- 3D coordinate transformation using camera intrinsics
- Real-world angle calculation from pixel positions
- Camera calibration parameter integration
- Fixed depth assumption for targeting calculations
- Precision targeting with angle tolerance control
"""

import rospy
import math
from std_msgs.msg import Bool
from gokhas_communication.msg import JointMessage
from gokhas_perception.msg import YoloResult
from sensor_msgs.msg import CameraInfo

class AutonomousTargeting3D:
    def __init__(self):
        """
        Initialize the 3D Autonomous Targeting System
        
        Sets up ROS node, camera parameters, and 3D coordinate transformation
        """
        rospy.init_node('autonomous_targeting_3d', anonymous=True)
        
        # Publishers
        self.joint_pub = rospy.Publisher('/gokhas/joint_commands', JointMessage, queue_size=10)
        
        # Subscribers
        self.autonomous_sub = rospy.Subscriber('/gokhas/autonomous_mode', Bool, self.autonomous_callback)
        self.yolo_sub = rospy.Subscriber('/zedm/zed_node/rgb/yolo_result', YoloResult, self.yolo_callback)
        self.camera_info_sub = rospy.Subscriber('/zedm/zed_node/rgb/camera_info', CameraInfo, self.camera_info_callback)
        
        # State variables
        self.autonomous_mode = False
        self.camera_info = None
        self.fx = None  # Focal length x
        self.fy = None  # Focal length y
        self.cx = None  # Principal point x
        self.cy = None  # Principal point y
        self.target_depth = 10.0  # Fixed depth (meters)
        self.image_width = 1920  # YOLOv8 image width
        self.image_height = 1080  # YOLOv8 image height
        self.center_x = self.image_width // 2   # Image center X (960)
        self.center_y = self.image_height // 2  # Image center Y (540)
        
        # Control parameters
        self.angle_tolerance = 2.0  # Degree tolerance - stops if smaller than this angle
        self.last_command_time = 0.0  # Last command time
        self.command_interval = 0.3  # Minimum wait between commands (seconds)
        
        # Debug counters
        self.yolo_msg_count = 0
        self.camera_info_received = False
        
        rospy.loginfo("Autonomous Targeting 3D initialized")
        rospy.loginfo(f"Image center: ({self.center_x}, {self.center_y})")
        rospy.loginfo(f"Target depth: {self.target_depth}m")
        rospy.loginfo(f"Angle tolerance: ±{self.angle_tolerance}°")
        
        # Start a timer to check system status
        rospy.Timer(rospy.Duration(5), self.status_check_callback)
        
    def autonomous_callback(self, msg):
        """
        Autonomous mode toggle callback
        
        Activates/deactivates autonomous 3D targeting mode
        """
        self.autonomous_mode = msg.data
        if self.autonomous_mode:
            rospy.loginfo("Autonomous 3D targeting ACTIVATED")
        else:
            rospy.loginfo("Autonomous 3D targeting DEACTIVATED")
    
    def camera_info_callback(self, msg):
        """
        Store camera calibration info
        
        Extracts and stores camera intrinsic parameters for 3D calculations
        """
        if self.camera_info is None:
            self.camera_info = msg
            # Extract camera intrinsic parameters
            # Camera matrix: [fx 0 cx; 0 fy cy; 0 0 1]
            self.fx = msg.K[0]  # K[0] = fx (focal length x)
            self.fy = msg.K[4]  # K[4] = fy (focal length y)
            self.cx = msg.K[2]  # K[2] = cx (principal point x)
            self.cy = msg.K[5]  # K[5] = cy (principal point y)
            self.camera_info_received = True
            
            rospy.loginfo(f"Camera info received - fx:{self.fx:.1f} fy:{self.fy:.1f} cx:{self.cx:.1f} cy:{self.cy:.1f}")
    
    def status_check_callback(self, event):
        """Periodic status check"""
        rospy.loginfo(f"System Status - Camera info: {self.camera_info_received}, YOLO messages: {self.yolo_msg_count}, Autonomous mode: {self.autonomous_mode}")
    
    def pixel_to_3d_angle(self, pixel_x, pixel_y):
        """Convert pixel coordinates to 3D angles using camera info"""
        if self.fx is None or self.fy is None:
            rospy.logwarn("Camera info not available yet")
            return 0.0, 0.0
        
        # Convert pixel to normalized coordinates
        x_norm = (pixel_x - self.cx) / self.fx
        y_norm = (pixel_y - self.cy) / self.fy
        
        # Calculate 3D coordinates assuming fixed depth
        x_3d = x_norm * self.target_depth
        y_3d = y_norm * self.target_depth
        z_3d = self.target_depth
        
        # Calculate angles from camera center
        # Horizontal angle (yaw) - positive = right
        horizontal_angle = math.degrees(math.atan2(x_3d, z_3d))
        
        # Vertical angle (pitch) - positive = up  
        vertical_angle = math.degrees(math.atan2(-y_3d, z_3d))
        
        return horizontal_angle, vertical_angle
    
    def yolo_callback(self, msg):
        """Process YOLO detection results for 3D targeting"""
        self.yolo_msg_count += 1
        
        if not self.autonomous_mode:
            if self.yolo_msg_count % 50 == 0:  # Log every 50 messages when not autonomous
                rospy.loginfo(f"YOLO messages received: {self.yolo_msg_count} (autonomous mode disabled)")
            return
            
        # Check if camera info is available
        if self.fx is None:
            rospy.logwarn("Camera info not available yet")
            return
            
        # Find first detected object from detections array
        target_detection = None
        if msg.detections.detections:  # If there are any detections
            target_detection = msg.detections.detections[0]  # Take first detection
        
        if target_detection is None:
            rospy.logwarn("No target detected")
            return
            
        # Calculate object center from bounding box
        obj_center_x = target_detection.bbox.center.x
        obj_center_y = target_detection.bbox.center.y
        
        rospy.loginfo(f"Target center: ({obj_center_x:.1f}, {obj_center_y:.1f})")
        
        # Rate limiting - prevent too frequent commands
        current_time = rospy.get_time()
        if (current_time - self.last_command_time) < self.command_interval:
            return
        
        # Convert pixel coordinates to 3D angles
        horizontal_angle, vertical_angle = self.pixel_to_3d_angle(obj_center_x, obj_center_y)
        
        rospy.loginfo(f"Calculated angles: H={horizontal_angle:.2f}°, V={vertical_angle:.2f}°")
        
        # Check if target is within tolerance (dead zone)
        if (abs(horizontal_angle) <= self.angle_tolerance and 
            abs(vertical_angle) <= self.angle_tolerance):
            rospy.loginfo(f"Target within tolerance - No movement needed")
            return
            
        # Update last command time
        self.last_command_time = current_time
        
        # Send joint commands with calculated angles
        self.send_joint_command(horizontal_angle, vertical_angle)
    
    def send_joint_command(self, j1_angle, j2_angle):
        """Send joint movement commands to the robot with calculated angles"""
        joint_msg = JointMessage()
        joint_msg.control_bits = 0
        
        # Joint 1 (Horizontal) - Convert to unsigned integer and handle sign
        if j1_angle < 0:
            joint_msg.control_bits |= 0b00000010  # Set bit 1 for negative
            joint_msg.j1p = min(int(abs(j1_angle)), 135)  # Clamp to max range
        else:
            joint_msg.j1p = min(int(j1_angle), 135)  # Clamp to max range
            
        # Joint 2 (Vertical) - Convert to unsigned integer and handle sign
        if j2_angle < 0:
            joint_msg.control_bits |= 0b00000100  # Set bit 2 for negative
            joint_msg.j2p = min(int(abs(j2_angle)), 135)  # Clamp to max range
        else:
            joint_msg.j2p = min(int(j2_angle), 135)  # Clamp to max range
        
        # Set speed and airsoft parameters
        joint_msg.j1s = 50  # Joint 1 speed (0-100)
        joint_msg.j2s = 50  # Joint 2 speed (0-100)
        joint_msg.ap = 0    # Airsoft power (0-100)
        
        # Publish command
        self.joint_pub.publish(joint_msg)
        
        rospy.loginfo(f"Joint command sent: J1={j1_angle:.2f}°, J2={j2_angle:.2f}°, control_bits=0b{joint_msg.control_bits:08b}")

def main():
    try:
        targeting = AutonomousTargeting3D()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Autonomous Targeting 3D node terminated")

if __name__ == '__main__':
    main()