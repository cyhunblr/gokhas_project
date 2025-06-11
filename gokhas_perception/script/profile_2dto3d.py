#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import Float32MultiArray

class PerformanceMonitor:
    def __init__(self):
        rospy.init_node('performance_monitor')
        
        # Subscribe to original node's performance data
        self.perf_pub = rospy.Publisher('/performance_stats', Float32MultiArray, queue_size=1)
        
        self.start_time = time.time()
        self.message_count = 0
        
    def monitor_topic(self, topic_name):
        def callback(msg):
            self.message_count += 1
            current_time = time.time()
            elapsed = current_time - self.start_time
            
            if elapsed >= 10.0:  # Every 10 seconds
                frequency = self.message_count / elapsed
                rospy.loginfo(f"Topic {topic_name}: {frequency:.2f} Hz")
                
                # Publish performance data
                perf_msg = Float32MultiArray()
                perf_msg.data = [frequency, elapsed, self.message_count]
                self.perf_pub.publish(perf_msg)
                
                # Reset counters
                self.start_time = current_time
                self.message_count = 0
        
        return callback

if __name__ == '__main__':
    monitor = PerformanceMonitor()
    
    # Monitor YOLO topic
    rospy.Subscriber('/detected_point_cloud', rospy.AnyMsg, 
                     monitor.monitor_topic('/detected_point_cloud'))
    
    rospy.spin()