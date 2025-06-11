#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Coordinate Publisher Module:
Publishes clicked coordinates and trigger status from the interface using gokhas_communication.msg.
"""

import rospy
from gokhas_communication.msg import communication


class CoordinatePublisher:
    """Class that publishes coordinate and trigger information using communication.msg"""
    
    def __init__(self):
        """Initialize the publisher"""
        # Start ROS node if not already initialized
        if not rospy.get_node_uri():
            rospy.init_node('coordinate_publisher', anonymous=True)
        
        # Single publisher for communication message
        self.comm_pub = rospy.Publisher('/gokhas/communication', communication, queue_size=10)
        
        # Default communication message template
        self.comm_msg = communication()
        self._init_default_message()
        
        rospy.loginfo("Coordinate Publisher initialized")
        print("[CoordinatePublisher] Publisher started - using gokhas_communication.msg")
    
    def _init_default_message(self):
        """Set default message values"""
        # MOD STATES - default values for system mode
        self.comm_msg.comStat = True   # Communication active
        self.comm_msg.modStat = False  # Manual mode
        
        # RECEIVE STATES - not used for coordinates, set to 0/false
        self.comm_msg.rJ1p = 0
        self.comm_msg.rJ1s = 0
        self.comm_msg.rJ2p = 0
        self.comm_msg.rJ2s = 0
        self.comm_msg.rAp = 0
        self.comm_msg.rAt = False
        
        # TRANSMIT STATES - will be used for coordinates and trigger
        self.comm_msg.tJ1p = 0    # For X coordinate
        self.comm_msg.tJ1s = 0    # Not used
        self.comm_msg.tJ2p = 0    # For Y coordinate
        self.comm_msg.tJ2s = 0    # Not used
        self.comm_msg.tAp = 0     # Not used
        self.comm_msg.tAt = False # For trigger status
        
        # CALIBRATION STATES - not used, set to false
        self.comm_msg.cJ1 = False
        self.comm_msg.cJ2 = False
    
    def publish_coordinates(self, x, y):
        """
        Print clicked coordinates to terminal and publish to communication topic
        
        Args:
            x (float): X coordinate of mouse click
            y (float): Y coordinate of mouse click
        """
        try:
            # Print to terminal for debugging
            print(f"[CLICK] Coordinates: x={int(x)}, y={int(y)}")
            
            # Update communication message with coordinates
            self.comm_msg.tJ1p = int(x)  # Assign X coordinate to tJ1p
            self.comm_msg.tJ2p = int(y)  # Assign Y coordinate to tJ2p
            self.comm_msg.tAt = False    # Set trigger to false when publishing coordinates
            
            # Add timestamp to message header if available
            self.comm_msg.header.stamp = rospy.Time.now() if hasattr(self.comm_msg, 'header') else rospy.Time.now()
            
            # Publish to ROS topic
            self.comm_pub.publish(self.comm_msg)
            
            rospy.loginfo(f"Published coordinates via communication.msg: tJ1p={int(x)}, tJ2p={int(y)}")
            print(f"[COMM_MSG] tJ1p(X)={int(x)}, tJ2p(Y)={int(y)}, tAt=False")
            
        except Exception as e:
            print(f"[ERROR] Coordinate publishing error: {e}")
            rospy.logerr(f"Coordinate publishing error: {e}")
    
    def publish_trigger(self, triggered=True):
        """
        Print trigger status to terminal and publish to communication topic
        
        Args:
            triggered (bool): Trigger status (True for triggered, False for not triggered)
        """
        try:
            # Print to terminal for debugging
            trigger_text = "TRIGGERED" if triggered else "NOT_TRIGGERED"
            print(f"[TRIGGER] Status: {trigger_text}")
            
            # Update communication message with trigger status
            self.comm_msg.tAt = triggered  # Assign trigger status to tAt
            # Reset coordinates (only trigger is being published)
            self.comm_msg.tJ1p = 0
            self.comm_msg.tJ2p = 0
            
            # Publish to ROS topic
            self.comm_pub.publish(self.comm_msg)
            
            rospy.loginfo(f"Published trigger via communication.msg: tAt={triggered}")
            print(f"[COMM_MSG] tAt={triggered}, tJ1p=0, tJ2p=0")
            
        except Exception as e:
            print(f"[ERROR] Trigger publishing error: {e}")
            rospy.logerr(f"Trigger publishing error: {e}")
    
    def publish_coordinates_and_trigger(self, x, y, triggered=False):
        """
        Publish coordinates and trigger status simultaneously
        
        This method is useful when you want to send both the target coordinates
        and the trigger command in a single message.
        
        Args:
            x (float): X coordinate of mouse click
            y (float): Y coordinate of mouse click
            triggered (bool): Trigger status
        """
        try:
            # Print to terminal for debugging
            print(f"[CLICK+TRIGGER] Coordinates: x={int(x)}, y={int(y)}, Trigger: {triggered}")
            
            # Update communication message with both coordinates and trigger
            self.comm_msg.tJ1p = int(x)      # X coordinate
            self.comm_msg.tJ2p = int(y)      # Y coordinate
            self.comm_msg.tAt = triggered    # Trigger status
            
            # Publish to ROS topic
            self.comm_pub.publish(self.comm_msg)
            
            rospy.loginfo(f"Published coordinates+trigger via communication.msg: tJ1p={int(x)}, tJ2p={int(y)}, tAt={triggered}")
            print(f"[COMM_MSG] tJ1p(X)={int(x)}, tJ2p(Y)={int(y)}, tAt={triggered}")
            
        except Exception as e:
            print(f"[ERROR] Coordinate+Trigger publishing error: {e}")
            rospy.logerr(f"Coordinate+Trigger publishing error: {e}")
    
    def print_mouse_event(self, event_type, x=None, y=None):
        """
        Print mouse events in detail to terminal and publish them
        
        This method provides a unified interface for handling different types
        of mouse events and automatically publishes the appropriate messages.
        
        Args:
            event_type (str): Event type ('left_click', 'right_click', 'move')
            x (float, optional): X coordinate (required for click events)
            y (float, optional): Y coordinate (required for click events)
        """
        try:
            timestamp = rospy.Time.now()
            
            if event_type == "left_click" and x is not None and y is not None:
                print(f"[{timestamp}] LEFT CLICK - X: {int(x)}, Y: {int(y)}")
                self.publish_coordinates(x, y)
                
            elif event_type == "right_click":
                print(f"[{timestamp}] RIGHT CLICK - TRIGGER ACTIVATED")
                self.publish_trigger(True)
                
            elif event_type == "left_click_with_trigger" and x is not None and y is not None:
                print(f"[{timestamp}] LEFT CLICK + TRIGGER - X: {int(x)}, Y: {int(y)}")
                self.publish_coordinates_and_trigger(x, y, True)
                
            elif event_type == "move" and x is not None and y is not None:
                # Only print move events in debug mode to avoid spam
                if rospy.get_param('~debug_mouse_move', False):
                    print(f"[{timestamp}] MOUSE MOVE - X: {int(x)}, Y: {int(y)}")
                    
        except Exception as e:
            print(f"[ERROR] Mouse event print error: {e}")
            rospy.logerr(f"Mouse event print error: {e}")
    
    def print_communication_status(self):
        """Print current communication message status for debugging"""
        try:
            print("\n[COMM_STATUS] Current communication message state:")
            print(f"  MOD STATES: comStat={self.comm_msg.comStat}, modStat={self.comm_msg.modStat}")
            print(f"  RECEIVE: rJ1p={self.comm_msg.rJ1p}, rJ1s={self.comm_msg.rJ1s}, rJ2p={self.comm_msg.rJ2p}, rJ2s={self.comm_msg.rJ2s}")
            print(f"           rAp={self.comm_msg.rAp}, rAt={self.comm_msg.rAt}")
            print(f"  TRANSMIT: tJ1p={self.comm_msg.tJ1p}, tJ1s={self.comm_msg.tJ1s}, tJ2p={self.comm_msg.tJ2p}, tJ2s={self.comm_msg.tJ2s}")
            print(f"            tAp={self.comm_msg.tAp}, tAt={self.comm_msg.tAt}")
            print(f"  CALIBRATION: cJ1={self.comm_msg.cJ1}, cJ2={self.comm_msg.cJ2}")
            print("")
        except Exception as e:
            print(f"[ERROR] Status print error: {e}")
    
    def shutdown(self):
        """Close the publisher and clean up resources"""
        try:
            print("[CoordinatePublisher] Shutting down...")
            rospy.loginfo("Coordinate Publisher shutting down")
        except Exception as e:
            print(f"[ERROR] Shutdown error: {e}")


# Global publisher instance for singleton pattern
coordinate_publisher = None

def get_coordinate_publisher():
    """Return global coordinate publisher instance (singleton pattern)"""
    global coordinate_publisher
    if coordinate_publisher is None:
        coordinate_publisher = CoordinatePublisher()
    return coordinate_publisher

def publish_click_coordinates(x, y):
    """
    Convenience function: Publish clicked coordinates
    
    Args:
        x (float): X coordinate of mouse click
        y (float): Y coordinate of mouse click
    """
    publisher = get_coordinate_publisher()
    publisher.publish_coordinates(x, y)

def publish_trigger_event():
    """Convenience function: Publish trigger event"""
    publisher = get_coordinate_publisher()
    publisher.publish_trigger(True)

def publish_click_with_trigger(x, y):
    """
    Convenience function: Publish coordinates and trigger together
    
    Args:
        x (float): X coordinate of mouse click
        y (float): Y coordinate of mouse click
    """
    publisher = get_coordinate_publisher()
    publisher.publish_coordinates_and_trigger(x, y, True)

def print_click_event(x, y):
    """
    Convenience function: Print left click event
    
    Args:
        x (float): X coordinate of mouse click
        y (float): Y coordinate of mouse click
    """
    publisher = get_coordinate_publisher()
    publisher.print_mouse_event("left_click", x, y)

def print_trigger_event():
    """Convenience function: Print right click (trigger) event"""
    publisher = get_coordinate_publisher()
    publisher.print_mouse_event("right_click")

def print_click_trigger_event(x, y):
    """
    Convenience function: Print left click + trigger event
    
    Args:
        x (float): X coordinate of mouse click
        y (float): Y coordinate of mouse click
    """
    publisher = get_coordinate_publisher()
    publisher.print_mouse_event("left_click_with_trigger", x, y)


if __name__ == "__main__":
    """Main function for testing purposes"""
    try:
        # Initialize publisher
        pub = CoordinatePublisher()
        
        # Show initial status
        pub.print_communication_status()
        
        # Test coordinates for demonstration
        test_coordinates = [(100, 150), (250, 300), (400, 200)]
        
        print("Testing coordinate publishing with communication.msg...")
        
        # Publish test coordinates
        for i, (x, y) in enumerate(test_coordinates):
            print(f"\n--- Test {i+1} ---")
            pub.print_mouse_event("left_click", x, y)
            rospy.sleep(1)
            pub.print_communication_status()
        
        # Test trigger functionality
        print("\n--- Trigger Test ---")
        pub.print_mouse_event("right_click")
        rospy.sleep(1)
        pub.print_communication_status()
        
        # Test coordinate + trigger combination
        print("\n--- Coordinate + Trigger Test ---")
        pub.print_mouse_event("left_click_with_trigger", 500, 400)
        rospy.sleep(1)
        pub.print_communication_status()
        
        print("Test completed successfully.")
        
    except rospy.ROSInterruptException:
        print("ROS interrupted")
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f"Test error: {e}")