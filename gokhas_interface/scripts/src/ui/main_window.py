#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - Main Window Module
===================================

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

Main window module:
Initializes and manages the GokHAS Central Control Interface.
"""
import sys
import rospy

# PyQt6 components
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QLabel, QFrame,
    QPushButton, QSlider, QGridLayout, QHBoxLayout, QVBoxLayout,
    QTextEdit, QApplication
)
from PyQt6.QtCore import Qt, QSize, QTimer
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import QSizePolicy, QStyle

# Internal application modules
from ui.resources.resource import resource_manager
from ui.control_handlers import ControlHandlers
from ros.ros_bridge import ROSBridge
from ui.custom_widgets import ClickableLabel, HeightSyncWidget 

class MainWindow(QMainWindow):
    """Main application window for robotics control interface"""
    
    def __init__(self):
        super().__init__()
        
        # System state flags - track current system status
        self.system_activated = False           # Whether STM32 communication is active
        self.manual_mode_active = True          # Start in MANUAL mode (vs AUTONOMOUS)
        self.joint_controls_active = False      # Whether any joint is currently being controlled
        self.calibration_active = False         # Whether calibration is in progress
        
        # UI flags - control visual behavior
        self.show_borders = False               # Debug borders visibility flag
        self.shutdown_initiated = False         # Shutdown flag to prevent multiple shutdowns

        # Central UI element references - organized dictionary for easy access
        self.ui_elements = {
            "joint_buttons": [],                # List of QPushButton for joint control
            "calibration_buttons": [],          # List of QPushButton for calibration
            "position_sliders": {},             # {joint_name: QSlider} for position control
            "power_sliders": {},                # {joint_name: QSlider} for power control
            "arrow_buttons": [],                # All increment/decrement arrow buttons
            "mode_toggle_button": None,         # MANUAL/AUTONOMOUS toggle button
            "activation_toggle_button": None,   # DEACTIVATED/ACTIVATED toggle button
            "help_button": None,                # Help button over camera image
            "border_toggle_button": None,       # Debug border toggle button
            "panel_titles": [],                 # List of QLabel for panel titles
            # Other important widgets can be added here
        }
        
        # Initialize control handlers - manages all button clicks and system logic
        self.control_handlers = ControlHandlers(self)
        self._create_ui()  # This method will populate self.ui_elements
        self.apply_styles()
        
        # System starts in deactivated state - all controls disabled
        self.control_handlers._set_system_controls_enabled(False)
        
        # Start ROS bridge - handles communication with robotics system
        self.ros_bridge = ROSBridge(self)

        # Apply initial styling
        self.control_handlers.change_theme("light_theme")
        self.apply_styles()
        self.update_borders()  # Show borders initially

        # Connect communication signal - handle messages from STM32 microcontroller
        if hasattr(self, 'ros_bridge'):
            self.ros_bridge.communication_received.connect(self.handle_communication)

    def handle_communication(self, control_msg):
        """Process ControlMessage received from STM32 microcontroller"""
        # rospy.loginfo(f"Received ControlMessage: comStatus={control_msg.comStatus}, calibStatus={control_msg.calibStatus}")
        
        # Handle calibration completion (calibStatus=1 means calibration complete)
        if control_msg.calibStatus == 1:
            # STM32 completed calibration - this is the completion signal
            rospy.loginfo("STM32 calibration completed (calibStatus=1)")
            self.add_log_message("STM32 calibration completed successfully")
            
            # Complete calibration - set button to green
            if (hasattr(self.control_handlers, 'current_calibrating_button') and 
                self.control_handlers.current_calibrating_button and
                hasattr(self.control_handlers, 'current_calibrating_all_buttons') and
                self.control_handlers.current_calibrating_all_buttons and
                hasattr(self.control_handlers, 'calibration_in_progress') and
                self.control_handlers.calibration_in_progress):
                
                self.control_handlers.finish_calibration(
                    self.control_handlers.current_calibrating_button, 
                    self.control_handlers.current_calibrating_all_buttons
                )
        
        if control_msg.comStatus:
            # STM32 confirmed communication - set ACTIVATED button to green
            activation_button = None
            all_buttons = self.findChildren(QPushButton)
            for btn in all_buttons:
                if "ACTIVATED" in btn.text():
                    activation_button = btn
                    break
            
            if activation_button:
                # If button is in waiting (yellow) state, make it green
                if activation_button.objectName() == "toggle-waiting":
                    # Stop timeout timer - successful connection
                    if hasattr(self.control_handlers, '_stop_activation_timeout'):
                        self.control_handlers._stop_activation_timeout()
                    
                    activation_button.setObjectName("toggle-active")  # Green color
                    self.apply_styles()
                    self.add_log_message("STM32 communication established - System fully activated")
                    
                    # Enable system controls after successful communication
                    self.system_activated = True
                    self.control_handlers._set_system_controls_enabled(True)
                    
                    print("Button set to: ACTIVATED (confirmed/green)")

    def _create_ui(self):
        """Prepare main UI layout - creates all interface elements"""
        # Set window title
        self.setWindowTitle("GokHAS Control Interface")
        
        # Set window icon
        self._set_window_icon()

        # Central widget and grid layout - main container
        central = QWidget()
        self.setCentralWidget(central)
        grid = QGridLayout(central)
        # Set column and row stretch ratios for responsive layout
        grid.setColumnStretch(0, 5)  # Main area takes 5/6 of width
        grid.setColumnStretch(1, 1)  # Sidebar takes 1/6 of width
        grid.setRowStretch(0, 5)     # Camera area takes 5/7 of height
        grid.setRowStretch(1, 2)     # Control area takes 2/7 of height

        # Camera image area - use ClickableLabel for mouse interaction
        self.image_label = ClickableLabel(self, "Waiting for camera image...")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(320, 240)
        self.image_label.setScaledContents(False)  
        self.image_label.setStyleSheet("background-color: black;")
        self.image_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        grid.addWidget(self.image_label, 0, 0)

        # Bottom control panel - contains joint and calibration controls
        self.bottom_widget = QWidget()
        self.bottom_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        bottom_layout = QHBoxLayout(self.bottom_widget)

        # Left panel - joint controls
        self.left_panel = QFrame()
        self.left_panel.setFrameShape(QFrame.Shape.NoFrame)  # Remove default border
        self.left_panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Right panel - calibration controls
        self.right_panel = QFrame()
        self.right_panel.setFrameShape(QFrame.Shape.NoFrame)  # Remove default border
        self.right_panel.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
        self.right_panel.setFixedWidth(120)

        bottom_layout.addWidget(self.left_panel, stretch=3)
        bottom_layout.addWidget(self.right_panel, stretch=0)
        grid.addWidget(self.bottom_widget, 1, 0)

        # Create panel contents
        self._create_control_panel(self.left_panel, ["Joint1", "Joint2", "Effector"])
        self._create_calibration_panel(self.right_panel, ["Joints"]) 
        # Right sidebar - status, log, and action panels
        sidebar = QWidget()
        sidebar_layout = QVBoxLayout(sidebar)
        sidebar_layout.setContentsMargins(2, 2, 2, 2)
        sidebar_layout.setSpacing(0)

        # Create sidebar sections
        self.sidebar_toggle = self._create_toggle_section()    # Status controls
        self.sidebar_log = self._create_empty_section()        # Log display
        self.sidebar_action = self._create_action_section(self.bottom_widget)  # Action buttons

        # Add sections with appropriate stretch ratios
        sidebar_layout.addWidget(self.sidebar_toggle, stretch=2)
        sidebar_layout.addWidget(self.sidebar_log, stretch=5)
        sidebar_layout.addWidget(self.sidebar_action, stretch=2)

        grid.addWidget(sidebar, 0, 1, 2, 1)

        # Add help button overlay on camera image
        self._add_help_button()

    def _set_window_icon(self):
        """Set window icon - use logo if available, otherwise system icon"""
        icon = resource_manager.get_image("logo.png")
        if not icon.isNull():
            self.setWindowIcon(QIcon(icon))
        else:
            style = self.style()
            if style:
                self.setWindowIcon(style.standardIcon(QStyle.StandardPixmap.SP_ComputerIcon))

    def _create_control_panel(self, parent, names):
        """
        Create joint control panel with specified joint names.
        Each row contains button, position slider, and power slider.
        
        Args:
            parent: Parent widget to contain the controls
            names: List of joint names (e.g., ['Joint1', 'Joint2', 'Effector'])
        """
        layout = QVBoxLayout(parent)
        layout.setSpacing(25)
        layout.setContentsMargins(5, 5, 5, 5)        
        
        # Panel title
        title = QLabel("MANUAL JOINT and EFFECTOR CONTROL")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFixedHeight(30)
        title.setObjectName("panel-title")
        title.setStyleSheet("font-size: 16px; margin: 0; padding: 0; border: none;")
        layout.addWidget(title)
        
        # Create control row for each joint name
        for name in names:
            layout.addLayout(self._make_joint_row(name))

    def _make_joint_row(self, name):
        """
        Create control row for a single joint.
        Different configurations for Effector (power only) vs joints (position + power).
        
        Args:
            name: Joint name (Joint1, Joint2, or Effector)
            
        Returns:
            QHBoxLayout: Complete control row layout
        """
        row = QHBoxLayout()
        row.setSpacing(10)

        # 1. Joint activation button
        btn = QPushButton(name)
        btn.setObjectName("joint-button-inactive")  # Start in red (inactive)
        btn.setFixedSize(80, 40)
        # Connect to control handler for joint activation/deactivation
        btn.clicked.connect(lambda: self.control_handlers.handle_joint_click(btn))
        row.addWidget(btn)

        # Different control layouts: Effector has only Power, others have Position + Power
        if name == "Effector":
            # Effector configuration: Power control only
            power_group = QFrame()
            power_group.setObjectName("control-group")
            power_group.setStyleSheet("QFrame#control-group { border: 1px solid #606060; border-radius: 5px; padding: 5px; background-color: transparent; } QFrame#control-group * { border: none; }")
            pow_layout = QHBoxLayout(power_group)
            pow_layout.setContentsMargins(5, 5, 5, 5)
            pow_layout.setSpacing(5)
            
            # Power label - increased width for better alignment
            pow_label = QLabel("Power:")
            pow_label.setFixedWidth(70)
            pow_layout.addWidget(pow_label)
            
            # Left triangle button (decrease)
            pow_minus_btn = QPushButton("◀")
            pow_minus_btn.setFixedSize(25, 25)
            pow_minus_btn.setObjectName("arrow-button")
            pow_minus_btn.setEnabled(False)  # Disabled at startup
            pow_layout.addWidget(pow_minus_btn)
            
            # Power slider - wider for Effector (single slider)
            pow_slider = QSlider(Qt.Orientation.Horizontal)
            pow_slider.setRange(0, 100)
            pow_slider.setValue(0)
            pow_slider.setFixedWidth(240)  # Wider for Effector (only one slider)
            pow_slider.setEnabled(False)  # Disabled at startup
            pow_slider.setObjectName(f"power-slider-{name}")  # Unique object name for identification
            pow_layout.addWidget(pow_slider)
            
            # Right triangle button (increase)
            pow_plus_btn = QPushButton("▶")
            pow_plus_btn.setFixedSize(25, 25)
            pow_plus_btn.setObjectName("arrow-button")
            pow_plus_btn.setEnabled(False)  # Disabled at startup
            pow_layout.addWidget(pow_plus_btn)
            
            # Power value display - increased width
            pow_value = QLabel("0")
            pow_value.setFixedWidth(40)
            pow_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            pow_layout.addWidget(pow_value)
            
            # Add power group to row, taking full available width
            row.addWidget(power_group, stretch=2)

            # Connect power slider to value display and control handler
            pow_slider.valueChanged.connect(lambda v: pow_value.setText(str(v)))
            pow_slider.valueChanged.connect(lambda v, n=name: self.control_handlers.power_slider_changed(n, v))

            # Connect arrow buttons to slider adjustment
            pow_minus_btn.clicked.connect(lambda: self._adjust_slider(pow_slider, -1, pow_value))
            pow_plus_btn.clicked.connect(lambda: self._adjust_slider(pow_slider, +1, pow_value))

            # Add to UI element references - Effector has no position slider
            self.ui_elements["joint_buttons"].append(btn)
            self.ui_elements["power_sliders"][name] = pow_slider
            self.ui_elements["arrow_buttons"].extend([pow_minus_btn, pow_plus_btn])

        else:
            # Joint1 and Joint2 configuration: Position + Power controls
            
            # 2. Position control group - enclosed in single border
            position_group = QFrame()
            position_group.setObjectName("control-group")
            position_group.setStyleSheet("QFrame#control-group { border: 1px solid #606060; border-radius: 5px; padding: 5px; background-color: transparent; } QFrame#control-group * { border: none; }")
            pos_layout = QHBoxLayout(position_group)
            pos_layout.setContentsMargins(5, 5, 5, 5)
            pos_layout.setSpacing(5)
            
            # Position label - increased width
            pos_label = QLabel("Position:")
            pos_label.setFixedWidth(70)  # Increased from 60 to 70
            pos_layout.addWidget(pos_label)
            
            # Left triangle button (decrease position)
            pos_minus_btn = QPushButton("◀")
            pos_minus_btn.setFixedSize(25, 25)
            pos_minus_btn.setObjectName("arrow-button")
            pos_minus_btn.setEnabled(False)  # Disabled at startup
            pos_layout.addWidget(pos_minus_btn)
            
            # Position slider - range changed and default value 0
            pos_slider = QSlider(Qt.Orientation.Horizontal)
            pos_slider.setRange(-135, 135)  # Range: -135 to +135 degrees
            pos_slider.setValue(0)  # Start at center position (0 degrees)
            pos_slider.setFixedWidth(120)  # Reduced width to accommodate buttons
            pos_slider.setEnabled(False)  # Disabled at startup
            pos_slider.setObjectName(f"position-slider-{name}")  # Unique object name
            pos_layout.addWidget(pos_slider)
            
            # Right triangle button (increase position)
            pos_plus_btn = QPushButton("▶")
            pos_plus_btn.setFixedSize(25, 25)
            pos_plus_btn.setObjectName("arrow-button")
            pos_plus_btn.setEnabled(False)  # Disabled at startup
            pos_layout.addWidget(pos_plus_btn)
            
            # Position value display - increased width
            pos_value = QLabel("0")
            pos_value.setFixedWidth(40)  # Increased from 30 to 40
            pos_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            pos_layout.addWidget(pos_value)
            
            row.addWidget(position_group, stretch=1)

            # 3. Power control group - enclosed in single border
            power_group = QFrame()
            power_group.setObjectName("control-group")
            power_group.setStyleSheet("QFrame#control-group { border: 1px solid #606060; border-radius: 5px; padding: 5px; background-color: transparent; } QFrame#control-group * { border: none; }")
            pow_layout = QHBoxLayout(power_group)
            pow_layout.setContentsMargins(5, 5, 5, 5)
            pow_layout.setSpacing(5)
            
            # Power label - increased width
            pow_label = QLabel("Power:")
            pow_label.setFixedWidth(70)  # Increased from 60 to 70
            pow_layout.addWidget(pow_label)
            
            # Left triangle button (decrease power)
            pow_minus_btn = QPushButton("◀")
            pow_minus_btn.setFixedSize(25, 25)
            pow_minus_btn.setObjectName("arrow-button")
            pow_minus_btn.setEnabled(False)  # Disabled at startup
            pow_layout.addWidget(pow_minus_btn)
            
            # Power slider - increased width
            pow_slider = QSlider(Qt.Orientation.Horizontal)
            pow_slider.setRange(0, 100)  # Range: 0 to 100 percent
            pow_slider.setValue(0)
            pow_slider.setFixedWidth(120)  # Reduced width to accommodate buttons
            pow_slider.setEnabled(False)  # Disabled at startup
            pow_slider.setObjectName(f"power-slider-{name}")  # Unique object name
            pow_layout.addWidget(pow_slider)
            
            # Right triangle button (increase power)
            pow_plus_btn = QPushButton("▶")
            pow_plus_btn.setFixedSize(25, 25)
            pow_plus_btn.setObjectName("arrow-button")
            pow_plus_btn.setEnabled(False)  # Disabled at startup
            pow_layout.addWidget(pow_plus_btn)
            
            # Power value display - increased width
            pow_value = QLabel("0")
            pow_value.setFixedWidth(40)  # Increased from 30 to 40
            pow_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
            pow_layout.addWidget(pow_value)
            
            row.addWidget(power_group, stretch=1)

            # Connect sliders to value displays
            pos_slider.valueChanged.connect(lambda v: pos_value.setText(str(v)))
            pow_slider.valueChanged.connect(lambda v: pow_value.setText(str(v)))
            
            # Connect sliders to control handlers
            pos_slider.valueChanged.connect(lambda v, n=name: self.control_handlers.position_slider_changed(n, v))
            pow_slider.valueChanged.connect(lambda v, n=name: self.control_handlers.power_slider_changed(n, v))

            # Connect arrow buttons to slider adjustments
            pos_minus_btn.clicked.connect(lambda: self._adjust_slider(pos_slider, -1, pos_value))
            pos_plus_btn.clicked.connect(lambda: self._adjust_slider(pos_slider, +1, pos_value))
            pow_minus_btn.clicked.connect(lambda: self._adjust_slider(pow_slider, -1, pow_value))
            pow_plus_btn.clicked.connect(lambda: self._adjust_slider(pow_slider, +1, pow_value))

            # Add to UI element references
            self.ui_elements["joint_buttons"].append(btn)
            self.ui_elements["position_sliders"][name] = pos_slider
            self.ui_elements["power_sliders"][name] = pow_slider
            self.ui_elements["arrow_buttons"].extend([pos_minus_btn, pos_plus_btn, pow_minus_btn, pow_plus_btn])

        return row

    def _adjust_slider(self, slider, delta, value_label):
        """
        Adjust slider value by specified amount.
        
        Args:
            slider: QSlider to adjust
            delta: Amount to change (typically +1 or -1)
            value_label: QLabel to update with new value
        """
        current_value = slider.value()
        new_value = current_value + delta
        
        # Range check - ensure new value is within slider's min/max
        if new_value >= slider.minimum() and new_value <= slider.maximum():
            slider.setValue(new_value)
            value_label.setText(str(new_value))

    def _make_slider(self, min_val, max_val, name, attr):
        """
        Common slider factory method:
        - Range: min_val to max_val
        - Change event updates both label and calls handler
        
        Args:
            min_val: Minimum slider value
            max_val: Maximum slider value  
            name: Joint name for handler callback
            attr: Attribute type ("position" or "power")
            
        Returns:
            tuple: (slider, value_label)
        """
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue(0)
        slider.setFixedWidth(100 if attr == "position" else 90)

        value_label = QLabel("0")
        value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        value_label.setFixedWidth(40)

        # Connect label update
        slider.valueChanged.connect(lambda val, lbl=value_label: lbl.setText(str(val)))
        
        # Connect handler based on attribute type
        if attr == "position":
            slider.valueChanged.connect(lambda val, n=name: self.control_handlers.position_slider_changed(n, val))
        else:
            slider.valueChanged.connect(lambda val, n=name: self.control_handlers.power_slider_changed(n, val))

        return slider, value_label

    def _create_calibration_panel(self, parent, names):
        """
        Create calibration buttons panel.
        When button is clicked, calls handle_calibration_click method in control_handlers.
        
        Args:
            parent: Parent widget to contain calibration controls
            names: List of joint names to create calibration buttons for
        """
        layout = QVBoxLayout(parent)
        layout.setSpacing(10)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Panel title
        title = QLabel("CALIBRATION")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFixedHeight(30)
        title.setObjectName("panel-title")
        title.setStyleSheet("font-size: 16px; margin: 0; padding: 0; border: none;")
        layout.addWidget(title)

        # Create calibration buttons
        btns = []
        calib_layout = QVBoxLayout()
        calib_layout.setSpacing(10)
        for name in names:
            btn = QPushButton(name)
            btn.setObjectName("calibration-default")  # Start in default state (not gray)
            btn.setFixedSize(100, 35)
            # Connect to control handler for calibration process management
            btn.clicked.connect(lambda _, b=btn, btns=btns: self.control_handlers.handle_calibration_click(b, btns))
            calib_layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)
            btns.append(btn)
        layout.addLayout(calib_layout)

        # Add to UI element references
        self.ui_elements["calibration_buttons"].extend(btns)

    def _create_toggle_section(self):
        """
        Create toggle section in right sidebar with two toggle buttons:
        - Activation (DEACTIVATED/ACTIVATED) - controls STM32 communication
        - Mode (MANUAL/AUTONOMOUS) - controls operation mode
        
        Returns:
            QWidget: Complete toggle section widget
        """
        widget = QWidget()
        widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        layout = QVBoxLayout(widget)
        layout.setSpacing(5)

        # STATUS section title
        title = QLabel("STATUS")
        title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        title.setFixedHeight(20)
        title.setObjectName("panel-title")
        title.setStyleSheet("font-size: 16px; margin:0; padding:0; border: none;")
        layout.addWidget(title)

        # Toggle button configurations
        entries = [
            ("DEACTIVATED", "ACTIVATED", self.control_handlers.activation_button_clicked),
            ("MANUAL",      "AUTONOMOUS", self.control_handlers.mode_button_clicked),
        ]
        
        for i, (off_text, on_text, handler) in enumerate(entries):
            btn = QPushButton(off_text)
            btn.setCheckable(True)
            
            if i == 0:  # DEACTIVATED/ACTIVATED button
                btn.setChecked(False)
                btn.setText("DEACTIVATED")
                btn.setObjectName("toggle-inactive")  # Red color (inactive)
            else:  # MANUAL/AUTONOMOUS button
                btn.setChecked(False)  # Start as False
                btn.setText("MANUAL")  # Show MANUAL text
                btn.setObjectName("toggle-inactive")  # Red color (MANUAL mode)

            # Connect toggle handler with proper parameters
            btn.toggled.connect(
                lambda checked, b=btn, on=on_text, off=off_text, h=handler: 
                self.control_handlers.handle_toggle(b, checked, on, off, h)
            )
            layout.addWidget(btn)

            # Add to UI element references
            if "ACTIVATED" in on_text:
                self.ui_elements["activation_toggle_button"] = btn
            elif "MANUAL" in on_text:
                self.ui_elements["mode_toggle_button"] = btn

        layout.addSpacing(5) 
        return widget

    def _toggle_simulation_mode(self):
        """Simulation mode toggle handler - no longer used"""
        pass  # This method can be removed or kept empty

    def _create_empty_section(self):
        """Create LOG panel in right sidebar middle section."""
        empty = QFrame()
        empty.setFrameShape(QFrame.Shape.NoFrame)  # Remove default border
        
        # Add layout
        layout = QVBoxLayout(empty)
        layout.setSpacing(5)
        
        # Add LOG title
        title = QLabel("LOG")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFixedHeight(20)
        title.setObjectName("panel-title")
        title.setStyleSheet("font-size: 16px; margin: 0; padding: 0; border: none;")
        layout.addWidget(title)
        
        # Small spacing
        layout.addSpacing(10)
        
        # Terminal-like text area for system messages
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #00ff00;
                font-family: 'Courier New', monospace;
                font-size: 10px;
                border: 1px solid #333;
                border-radius: 5px;
                padding: 5px;
            }
        """)
        layout.addWidget(self.log_display)
        
        return empty

    def _create_action_section(self, reference):
        """
        Create action section in right sidebar bottom:
        Close button and theme selection buttons.
        
        Args:
            reference: Reference widget for height synchronization
            
        Returns:
            QWidget: Complete action section widget
        """
        widget = HeightSyncWidget(reference)
        layout = QVBoxLayout(widget)

        # Close application button
        btn = QPushButton()
        btn.setObjectName("close-button")
        btn.setIcon(resource_manager.get_icon("closeApp.png"))
        btn.setIconSize(QSize(120, 120))
        btn.setFixedSize(150, 150)
        btn.setToolTip("Close Application")
        btn.clicked.connect(self.control_handlers.close_app)
        layout.addWidget(btn, alignment=Qt.AlignmentFlag.AlignCenter)

        # Theme section title and buttons
        theme_lbl = QLabel("Theme")
        theme_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        theme_lbl.setObjectName("theme-label")
        theme_lbl.setStyleSheet("font-size: 12px; margin: 0; padding: 0;")
        layout.addWidget(theme_lbl)

        theme_layout = QHBoxLayout()

        # Theme toggle buttons
        for sym, theme in [("☀️", "light_theme"), ("🌙", "dark_theme")]:
            tbtn = QPushButton(sym)
            tbtn.setFixedSize(35, 35)
            tbtn.setToolTip("Light Theme" if theme == "light_theme" else "Dark Theme")
            tbtn.setObjectName("theme-button-active" if theme == "light_theme" else "theme-button-normal")
            tbtn.clicked.connect(lambda _, th=theme: self.control_handlers.handle_theme_change(th))
            theme_layout.addWidget(tbtn)

        # Debug border toggle button
        border_btn = QPushButton("B")
        border_btn.setFixedSize(35, 35)
        border_btn.setToolTip("Toggle Debug Borders On/Off")
        border_btn.setObjectName("border-button-normal")  # Start as off
        border_btn.clicked.connect(lambda: self.control_handlers.handle_border_toggle(border_btn))
        theme_layout.addWidget(border_btn)

        layout.addLayout(theme_layout)
        return widget

    def _add_help_button(self):
        """Add '?' help button overlay on camera image."""
        self.help_btn = self.control_handlers.create_help_button(self.image_label)

    def _reposition_help_btn(self):
        """Reposition help button to top-right corner of image_label."""
        if hasattr(self, 'help_btn'):
            self.control_handlers.reposition_help_button(self.help_btn, self.image_label)

    def resizeEvent(self, event):
        """Update help button position when window is resized."""
        super().resizeEvent(event)
        self._reposition_help_btn()

    def closeEvent(self, event):
        """Clean up when window is being closed."""
        if self.shutdown_initiated:
            event.accept()
            return
        self.shutdown_initiated = True
        self.ros_bridge.shutdown()
        if not rospy.is_shutdown():
            rospy.signal_shutdown("Window closed")
        event.accept()

    def apply_styles(self):
        """Apply styles to all widgets."""
        try:
            styles = "\n".join([
                resource_manager.get_current_theme(),
                resource_manager.get_joint_button_style(),
                resource_manager.get_style("calibration_buttons"),
                resource_manager.get_style("toggle_buttons"),
                resource_manager.get_close_button_style(),
                resource_manager.get_style("theme_buttons"),
                resource_manager.get_style("border_button"),
                resource_manager.get_helper_button_style(),  # Helper button style added
            ])
            self.setStyleSheet(styles)
            
            self.update_borders()  # Update borders after applying styles
            
        except Exception as e:
            print(f"Style application error: {e}")

    def add_log_message(self, message):
        """
        Add new message to LOG panel.
        
        Args:
            message: String message to display in log
        """
        if hasattr(self, 'log_display'):
            from datetime import datetime
            timestamp = datetime.now().strftime("%H:%M:%S")
            formatted_message = f"[{timestamp}] {message}"
            self.log_display.append(formatted_message)
            # Scroll to bottom to see latest message
            scrollbar = self.log_display.verticalScrollBar()
            if scrollbar is not None:
                scrollbar.setValue(scrollbar.maximum())
    
    def update_image(self, pixmap):
        """
        Update camera image while preserving aspect ratio.
        
        Args:
            pixmap: QPixmap containing the camera image
        """
        if hasattr(self, 'image_label'):
            # Get current QLabel size
            label_size = self.image_label.size()
            
            # Scale while preserving aspect ratio
            scaled_pixmap = pixmap.scaled(
                label_size, 
                Qt.AspectRatioMode.KeepAspectRatio,        # Preserve aspect ratio
                Qt.TransformationMode.FastTransformation  # Fast scaling
                # Qt.TransformationMode.SmoothTransformation  # Smooth scaling (slower)
            )
            
            # Set scaled image
            self.image_label.setPixmap(scaled_pixmap)
            # Center alignment - image stays centered in QLabel
            self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            
    def update_borders(self):
        """Update border styling for all components without affecting other styles."""
        if self.show_borders:
            # Debug mode: Red borders for visual debugging
            border = "2px solid red"
            
            # Camera label: Preserve background color
            self.image_label.setStyleSheet(
                f"background-color: black; border: {border};"
            )

            # Other widgets: border only
            other_widgets = [
                self.left_panel,
                self.right_panel,
                self.bottom_widget,
                self.sidebar_toggle,
                self.sidebar_log,
                self.sidebar_action,
            ]
            for widget in other_widgets:
                widget.setStyleSheet(f"border: {border};")
        else:
            # Normal mode: Theme-appropriate thin borders
            # Light theme uses #333333 (black), Dark theme uses #ffffff (white)
            current_theme = resource_manager.current_theme
            if "light" in current_theme:
                border_color = "#333333"  # Light theme text color
            else:
                border_color = "#ffffff"  # Dark theme text color
            
            border = f"1px solid {border_color}"
            
            # Camera label: Preserve background color
            self.image_label.setStyleSheet(
                f"background-color: black; border: {border};"
            )

            # Only main layout panels get borders
            main_widgets = [    
                self.left_panel,      # Control panel
                self.right_panel,     # Calibration panel
                self.sidebar_toggle,  # Status panel
                self.sidebar_log,     # Log panel
            ]
            for widget in main_widgets:
                widget.setStyleSheet(f"border: {border};")
            
            # Widgets that should not have borders
            no_border_widgets = [
                self.bottom_widget,   # Main container
                self.sidebar_action,  # Close app button area
            ]
            for widget in no_border_widgets:
                widget.setStyleSheet("border: none;")

    def _reconnect_mode_button(self):
        """Reconnect mode button click handler after reset."""
        mode_button = None
        
        # Find mode button in UI elements
        for key, value in self.ui_elements.items():
            if isinstance(value, QPushButton):
                if "MANUAL" in value.text() or "AUTONOMOUS" in value.text():
                    mode_button = value
                    break
        
        # Fallback: search all buttons
        if not mode_button:
            all_buttons = self.findChildren(QPushButton)
            for btn in all_buttons:
                if "MANUAL" in btn.text() or "AUTONOMOUS" in btn.text():
                    mode_button = btn
                    break
        
        if mode_button:
            # Reconnect original handler
            mode_button.clicked.connect(
                lambda checked: self.control_handlers.handle_toggle(
                    mode_button, 
                    checked, 
                    "AUTONOMOUS", 
                    "MANUAL", 
                    self.control_handlers.mode_button_clicked
                )
            )
            print("DEBUG: Mode button click handler reconnected")
