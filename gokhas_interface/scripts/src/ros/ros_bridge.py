#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROSBridge Module:
This module creates a bridge between ROS (Robot Operating System) and PyQt6 GUI.
It receives YOLOv8 detection images from ROS topics and displays them in the Qt interface.

Main responsibilities:
- Subscribe to ROS image topics
- Convert ROS Image messages to Qt-compatible QPixmap format
- Handle connection monitoring and status updates
- Provide thread-safe communication between ROS and GUI threads

Author: Ahmet Ceyhun Bilir <ahmetceyhunbilir16@gmail.com>
Date: 06.08.2025
"""

import rospy          # ROS Python library for node communication
import rosnode        # ROS library for checking active nodes
from sensor_msgs.msg import Image        # ROS message type for images
from cv_bridge import CvBridge, CvBridgeError  # Convert between ROS and OpenCV images

from PyQt6.QtCore import Qt, QObject, pyqtSignal, QTimer  # Qt core components
from PyQt6.QtGui import QImage, QPixmap                   # Qt image handling
from gokhas_communication.msg import communication

class ROSBridge(QObject):
    """
    Bridge class between ROS system and PyQt6 GUI
    
    This class inherits from QObject to enable Qt signal/slot mechanism
    for thread-safe communication between ROS callbacks and GUI updates.
    
    The bridge handles:
    - Image data flow from robot's camera to GUI display
    - Communication messages between GUI and robot control system
    - Connection monitoring to detect if robot systems are online
    - Thread-safe data transfer between ROS and Qt frameworks
    """
    
    # Define Qt signals for thread-safe GUI updates
    new_frame = pyqtSignal(QPixmap)              # Signal emitted when new camera image arrives
    log_message = pyqtSignal(str)                # Signal emitted for status/error messages
    communication_received = pyqtSignal(object)  # Signal for communication messages from robot

    def __init__(self, main_window):
        """
        Initialize the ROS bridge
        
        Args:
            main_window: Reference to the main Qt window for GUI updates
        """
        super().__init__()  # Initialize parent QObject
        
        # Shutdown flag to prevent operations during application closure
        self.is_shutting_down = False
        
        # Store reference to main window for direct access to GUI components
        self.main_window = main_window
        
        # Status tracking variables for robot connection monitoring
        self.first_image_received = False    # Flag to track if any image has been received
        self.detection_running = False       # Flag to track if detection is currently active
        self._last_node_check = set()        # Cache for node list to optimize repeated checks

        # Initialize ROS components (subscriber, bridge, etc.)
        if not self._initialize_ros():
            # If ROS initialization fails, exit constructor early
            return

        # Connect Qt signals to main window methods for thread-safe GUI updates
        self.new_frame.connect(main_window.update_image)        # Connect image updates
        self.log_message.connect(main_window.add_log_message)   # Connect log messages
        
        # Start monitoring timers and initial checks
        self._start_monitoring()

    def _initialize_ros(self):
        """
        Initialize all ROS-related components
        
        This method sets up:
        - Communication with robot control system
        - Subscription to camera image feed
        - Error handling for connection failures
        
        Returns:
            bool: True if initialization successful, False otherwise
        """
        # Check if ROS node is properly initialized
        try:
            rospy.get_name()  # This will throw exception if node not initialized
        except Exception as e:
            rospy.logerr(f"ROS node not started! Error: {e}")
            return False
        
        # Initialize OpenCV-ROS bridge for image conversion
        # This bridge converts between ROS image messages and OpenCV/Qt formats
        self.bridge = CvBridge()
        
        # Communication publisher/subscriber - INITIALIZE FIRST
        # These handle command/status communication with robot control system
        try:
            # Publisher: Send commands from GUI to robot
            self.comm_pub = rospy.Publisher('/gokhas/interface_commands', communication, queue_size=1)
            # Subscriber: Receive status updates from robot
            self.comm_sub = rospy.Subscriber('/gokhas/communication', communication, self.communication_callback)
            rospy.loginfo("ROSBridge: Communication publisher/subscriber initialized.")
        except Exception as e:
            rospy.logerr(f"Could not create communication pub/sub: {e}")
            return False
        
        # Create ROS subscriber for YOLOv8 detection images
        # This receives processed camera images with object detection overlays
        try:
            self.image_sub = rospy.Subscriber(
                '/zedm/zed_node/rgb/yolo_image',  # Topic name for YOLOv8 detection results
                Image,                              # Message type (sensor_msgs/Image)
                self._on_image,                     # Callback function when message received
                queue_size=1,                       # Keep only latest message (real-time display)
                buff_size=2**24                     # 16MB buffer for better performance with large images
            )
            rospy.loginfo("ROSBridge: Successfully subscribed to YOLOv8 image topic.")
            return True
        except Exception as e:
            rospy.logerr(f"Could not create image subscriber: {e}")
            return False

    def _start_monitoring(self):
        """
        Start monitoring timers for connection status
        
        Sets up periodic checks to monitor if the detection node is still active
        and updates GUI status accordingly. This helps users know if the robot
        vision system is working properly.
        """
        # Create timer for periodic connection checks
        # Using 2-second interval instead of 1 second to reduce CPU overhead
        self.connection_timer = QTimer()
        self.connection_timer.timeout.connect(self._check_connection_callback)
        self.connection_timer.start(2000)  # Check every 2 seconds
        
        # Perform initial status check after short delay to ensure GUI is ready
        QTimer.singleShot(100, self._initial_check)

    def _initial_check(self):
        """
        Perform initial status check at startup
        
        Checks if the tracker node is already running when the interface starts
        and displays appropriate status message to inform user of system state.
        """
        try:
            # Check if tracker node is already active
            if not self._is_tracker_node_active():
                # If no tracker node found, display waiting message
                self.log_message.emit("Waiting for camera image...")
        except Exception:
            # If node checking fails, display error message
            self.log_message.emit("Node connection error!")

    def _is_tracker_node_active(self):
        """
        Check if the tracker node is currently active
        
        The tracker node handles object detection and image processing.
        Uses caching optimization to avoid repeated expensive ROS API calls
        when the node list hasn't changed.
        
        Returns:
            bool: True if tracker node is active, False otherwise
        """
        try:
            # Get list of all currently active ROS nodes
            active_nodes = set(rosnode.get_node_names())
            
            # Optimization: Only update cache if node list has changed
            # This prevents unnecessary processing on every check
            if active_nodes != self._last_node_check:
                self._last_node_check = active_nodes
            
            # Check if our specific tracker node is in the active list
            return '/tracker_node' in active_nodes
        except Exception:
            # If ROS API call fails, assume node is not active
            return False

    def _on_image(self, msg):
        """
        Callback function triggered when new image message arrives from ROS
        
        This function processes camera images with object detection overlays
        and converts them for display in the Qt GUI. It runs in ROS callback 
        thread, so all GUI updates must be done through Qt signals for thread safety.
        
        Args:
            msg: ROS Image message containing the detection result image
        """
        # Don't process images if system is shutting down
        if self.is_shutting_down:
            return
            
        try:
            # Step 1: Convert ROS Image message to OpenCV format
            # ROS uses its own image format, OpenCV uses numpy arrays
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Step 2: Convert OpenCV image to Qt QImage format
            # Qt requires specific format for proper display
            h, w, ch = cv_img.shape  # Get image dimensions and channels
            qt_img = QImage(
                cv_img.data,                    # Raw image data pointer
                w, h,                           # Width and height
                ch * w,                         # Bytes per line (stride)
                QImage.Format.Format_RGB888     # Color format
            ).rgbSwapped()  # Convert from BGR (OpenCV) to RGB (Qt)
            
            # Step 3: Create QPixmap from QImage for display
            # QPixmap is optimized for display in Qt widgets
            pix = QPixmap.fromImage(qt_img)
            
            # Step 4: Emit signal to update GUI (thread-safe) - display image as-is
            # This crosses thread boundaries safely using Qt's signal/slot system
            self.new_frame.emit(pix)
            
            # Step 5: Update detection status messages
            self._update_detection_status()
            
        except CvBridgeError as e:
            # Handle specific OpenCV-ROS conversion errors
            if not self.is_shutting_down:
                rospy.logerr(f"CV Bridge error: {e}")
                self.log_message.emit(f"ERROR: Image conversion failed - {e}")
        except Exception as e:
            # Handle any other unexpected errors during image processing
            if not self.is_shutting_down:
                rospy.logerr(f"Image processing error: {e}")
                self.log_message.emit(f"ERROR: Image processing failed - {e}")

    def communication_callback(self, msg):
        """
        Handle communication messages received from robot control system
        
        This callback processes status updates and responses from the robot,
        such as confirmation of commands or system state changes.
        
        Args:
            msg: Communication message from robot control system
        """
        # Forward communication message to main interface through signal
        # This ensures thread-safe delivery to GUI components
        self.communication_received.emit(msg)

    def publish_communication(self, comm_msg):
        """
        Send communication message to robot control system
        
        This method publishes commands and requests from the GUI
        to the robot control system for execution.
        
        Args:
            comm_msg: Communication message to send to robot
        """
        # Publish message to robot control system
        self.comm_pub.publish(comm_msg)

    def _update_detection_status(self):
        """
        Update detection status messages based on current state
        
        This function manages the status messages displayed in the GUI:
        - "Detection Started!" when first image arrives after startup
        - "Detection Restarted!" when detection resumes after interruption
        
        These messages help users understand when the robot vision system
        is active and processing camera data.
        """
        if not self.first_image_received:
            # First image ever received since startup
            self.log_message.emit("Detection Started!")
            self.first_image_received = True
            self.detection_running = True
        elif not self.detection_running:
            # Detection was stopped but now resumed
            self.log_message.emit("Detection Restarted!")
            self.detection_running = True

    def _check_connection_callback(self):
        """
        Timer callback function for periodic connection monitoring
        
        This function is called every 2 seconds by the QTimer to check
        if the detection node is still active. If the node stops, it
        updates the GUI status accordingly to inform the user.
        
        This monitoring helps detect network issues, node crashes,
        or intentional shutdowns of the robot vision system.
        """
        # Don't check during shutdown process
        if self.is_shutting_down:
            return
            
        try:
            # Optimization: Only check if detection was previously running
            # No need to check if we already know detection is stopped
            if self.detection_running and not self._is_tracker_node_active():
                # Tracker node is no longer active, update status
                self.log_message.emit("Detection stopped")
                self.detection_running = False
        except Exception as e:
            # Log any errors during node checking (but don't crash)
            rospy.logwarn(f"Node check error: {e}")

    def shutdown(self):
        """
        Properly shut down the ROS bridge and clean up resources
        
        This function ensures all ROS subscriptions are properly closed
        and memory is freed when the application exits. Proper cleanup
        prevents resource leaks and ensures clean shutdown.
        """
        # Prevent multiple shutdown calls
        if self.is_shutting_down:
            return
        
        # Set shutdown flag to stop all operations
        self.is_shutting_down = True
        
        # Step 1: Stop the monitoring timer
        if hasattr(self, 'connection_timer') and self.connection_timer:
            self.connection_timer.stop()
            self.connection_timer = None  # Clear reference
        
        # Step 2: Unregister ROS subscribers and publishers
        # This stops all data flow between ROS and the application
        if hasattr(self, 'image_sub') and self.image_sub:
            try:
                self.image_sub.unregister()  # Stop receiving camera images
            except Exception:
                pass  # Ignore errors during shutdown
            self.image_sub = None  # Clear reference
        
        if hasattr(self, 'comm_sub') and self.comm_sub:
            try:
                self.comm_sub.unregister()  # Stop receiving communication messages
            except Exception:
                pass
            self.comm_sub = None
        
        if hasattr(self, 'comm_pub') and self.comm_pub:
            try:
                self.comm_pub.unregister()  # Stop publishing commands
            except Exception:
                pass
            self.comm_pub = None
        
        # Step 3: Clear object references to prevent memory leaks
        self.main_window = None
        self._last_node_check.clear()
        
        rospy.loginfo("ROS Bridge successfully closed.")