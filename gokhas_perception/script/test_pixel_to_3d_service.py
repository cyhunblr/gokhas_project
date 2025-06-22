#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gokhas_perception.srv import PixelTo3D

def test_pixel_to_3d_service():
    rospy.init_node('pixel_to_3d_test_client', anonymous=True)
    rospy.wait_for_service('pixel_to_3d_angles')
    
    try:
        # Get service proxy
        pixel_to_3d = rospy.ServiceProxy('pixel_to_3d_angles', PixelTo3D)
        
        # Test Case 1: Autonomous mode
        print("Testing autonomous mode...")
        response = pixel_to_3d(True, 0, 0)
        print(f"Response: joint1degree={response.joint1degree}°, joint2degree={response.joint2degree}°")
        
        # Test Case 2: Manual mode with specific pixel coordinates
        print("Testing manual mode...")
        response = pixel_to_3d(False, 320, 240)
        print(f"Response: joint1degree={response.joint1degree}°, joint2degree={response.joint2degree}°")
        
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == '__main__':
    test_pixel_to_3d_service()