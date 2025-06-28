#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GokHAS Project - Custom PyQt6 Widgets Module
============================================

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

Custom PyQt6 Widgets Module
Provides specialized widgets for robotics control interface
"""

from PyQt6.QtWidgets import QLabel, QWidget
from PyQt6.QtCore import Qt

class ClickableLabel(QLabel):
    """
    Custom QLabel that can capture mouse events for camera image interaction.
    
    This widget is used to display the camera feed and allows users to:
    - Left click: Select target coordinates for robot positioning
    - Right click: Trigger robot action at previously selected coordinates
    
    Mouse events are only processed when system is in the correct state:
    - System must be activated (STM32 communication established)
    - Must be in MANUAL mode (not AUTONOMOUS)
    - No joints should be actively controlled
    - No calibration should be in progress
    """
    
    def __init__(self, main_window, default_text=""):
        """
        Initialize clickable label for camera image display.
        
        Args:
            main_window: Reference to main application window
            default_text: Text to display when no image is available
        """
        super().__init__(default_text)
        self.main_window = main_window
        self.default_text = default_text
        
        # Store last click coordinates for right-click trigger functionality
        self.last_click_x = None
        self.last_click_y = None

    def mousePressEvent(self, event):
        """
        Handle mouse click events on the camera image.
        
        This method processes both left and right mouse clicks:
        - Left click: Sets target coordinates for robot movement
        - Right click: Triggers robot action at stored coordinates
        
        Mouse events are filtered based on system state to prevent
        accidental commands when system is not ready.
        
        Args:
            event: QMouseEvent containing click information
        """
        # Check if mouse events should be processed based on system state
        if not self._should_handle_mouse_events():
            self._handle_blocked_click()
            return
        
        # Process different mouse button clicks
        if event.button() == Qt.MouseButton.LeftButton:
            self._handle_left_click(event)
        elif event.button() == Qt.MouseButton.RightButton:
            self._handle_right_click()
            
        # Call parent implementation to maintain Qt functionality
        super().mousePressEvent(event)

    def _should_handle_mouse_events(self):
        """
        Determine whether mouse events should be processed.
        
        Mouse clicks on camera image are only allowed when:
        1. System is activated (STM32 communication working)
        2. System is in MANUAL mode (user control enabled)
        3. No joint controls are active (prevents conflicting commands)
        4. No calibration is in progress (prevents interference)
        
        Returns:
            bool: True if mouse events should be processed, False otherwise
        """
        # Safety check: ensure control handlers exist
        if not hasattr(self.main_window, 'control_handlers'):
            return False
            
        # Get current system state flags
        system_active = getattr(self.main_window, 'system_activated', False)
        manual_mode = getattr(self.main_window, 'manual_mode_active', False)
        joint_controls_active = getattr(self.main_window, 'joint_controls_active', False)
        calibration_active = getattr(self.main_window, 'calibration_active', False)
        
        # All conditions must be met for mouse events to be processed
        return (system_active and manual_mode and 
                not joint_controls_active and not calibration_active)

    def _handle_left_click(self, event):
        """
        Process left mouse click to select target coordinates.
        
        Left click stores the pixel coordinates where the user clicked
        and sends them to the control handler for target selection.
        These coordinates can later be used for robot positioning.
        
        Args:
            event: QMouseEvent containing click position data
        """
        # Get click coordinates relative to the widget
        self.last_click_x = event.position().x()
        self.last_click_y = event.position().y()
        
        # Send coordinates to control handler for processing
        self.main_window.control_handlers.handle_image_click(
            self.last_click_x, self.last_click_y
        )

    def _handle_right_click(self):
        """
        Process right mouse click to trigger robot action.
        
        Right click uses the coordinates from the last left click
        to trigger a robot action (like moving to that position).
        Requires previous left click to establish target coordinates.
        """
        # Check if target coordinates are available from previous left click
        if self.last_click_x is not None and self.last_click_y is not None:
            # Send trigger command to control handler with stored coordinates
            self.main_window.control_handlers.handle_image_trigger(
                self.last_click_x, self.last_click_y
            )
        else:
            # Inform user that left click is required first
            self.main_window.add_log_message(
                "Right click ignored: No target coordinates (left click first)"
            )

    def _handle_blocked_click(self):
        """
        Handle mouse clicks when they are blocked due to system state.
        
        Provides user feedback explaining why their click was ignored
        and what conditions need to be met for clicks to work.
        Only shows messages for important blocking conditions.
        """
        # Get current system state to determine why click was blocked
        system_active = getattr(self.main_window, 'system_activated', False)
        manual_mode = getattr(self.main_window, 'manual_mode_active', False)
        joint_controls_active = getattr(self.main_window, 'joint_controls_active', False)
        calibration_active = getattr(self.main_window, 'calibration_active', False)
        
        # Only provide feedback for important blocking conditions
        # (when system is ready but something specific is preventing clicks)
        if system_active and manual_mode:
            if joint_controls_active:
                self.main_window.add_log_message(
                    "Image clicks disabled: Deactivate joint controls first"
                )
            elif calibration_active:
                self.main_window.add_log_message(
                    "Image clicks disabled: Wait for calibration to complete"
                )
    
    def mouseMoveEvent(self, event):
        """
        Handle mouse movement events - reserved for future functionality.
        
        Currently not used but could be extended to show coordinates
        while hovering or provide real-time feedback.
        
        Args:
            event: QMouseEvent containing movement information
        """
        super().mouseMoveEvent(event)


class HeightSyncWidget(QWidget):
    """
    Custom widget that automatically synchronizes its height with a reference widget.
    
    This widget is useful for creating responsive layouts where one widget
    needs to match the height of another widget, even when the reference
    widget's size changes due to window resizing or content changes.
    
    Used in the sidebar to ensure action buttons maintain proper proportions
    relative to the main control panels.
    """
    
    def __init__(self, reference_widget, parent=None):
        """
        Initialize height-synchronized widget.
        
        Args:
            reference_widget: Widget whose height this widget should match
            parent: Parent widget (optional)
        """
        super().__init__(parent)
        self.reference_widget = reference_widget

    def showEvent(self, event):
        """
        Handle widget show event to update height when widget becomes visible.
        
        Args:
            event: QShowEvent
        """
        super().showEvent(event)
        self.updateHeight()

    def resizeEvent(self, event):
        """
        Handle widget resize event to update height when layout changes.
        
        This ensures the widget maintains height synchronization even
        when the parent widget or window is resized.
        
        Args:
            event: QResizeEvent
        """
        super().resizeEvent(event)
        self.updateHeight()

    def updateHeight(self):
        """
        Update this widget's height to match the reference widget's height.
        
        This method is called automatically during show and resize events
        to maintain height synchronization. Only updates if reference
        widget has a valid height (> 0).
        """
        # Ensure reference widget exists and has valid height
        if self.reference_widget and self.reference_widget.height() > 0:
            # Set this widget's height to match reference widget
            self.setFixedHeight(self.reference_widget.height())