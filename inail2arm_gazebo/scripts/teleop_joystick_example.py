#!/usr/bin/env python3
"""
ROS2 Joystick Teleop Example - Official Pattern

This demonstrates the proper ROS2 pattern for joystick-controlled robot teleoperation.
Follows the standard ROS2 pattern used by teleop_twist_joy package.

Key principles:
1. Subscribe to /joy topic (from joy_node)
2. Map joystick axes/buttons to robot commands
3. Publish commands based on joystick state
4. Use parameters for configurability

Dependencies:
  sudo apt install ros-<distro>-joy ros-<distro>-joy-linux

Usage:
  ros2 launch inail2arm_gazebo joystick_teleop.launch.py
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped
import math

# For service call to set control mode
try:
    from cartesian_interface_ros.srv import SetControlMode
except ImportError:
    # Fallback if service type not found
    SetControlMode = None


class JoystickTeleopNode(Node):
    """
    ROS2 Node that publishes commands based on joystick input.
    
    This follows the official ROS2 pattern where:
    - Subscribe to /joy topic (published by joy_node)
    - Map joystick axes/buttons to commands
    - Publish commands based on joystick state updates
    """
    
    def __init__(self):
        super().__init__('joystick_teleop_node')
        
        # Declare parameters for axis/button mapping (following ROS2 convention)
        # Default mappings are for Xbox controller
        
        # Linear position axes (translation) - Left stick
        self.declare_parameter('axis_linear_x', 1)      # Left stick horizontal (axis 0)
        self.declare_parameter('axis_linear_y', 0)       # Left stick vertical (axis 1)
        
        # Z position control - Small triggers (bumpers LB/RB) or buttons
        self.declare_parameter('axis_linear_z_up', 4)         # Small left trigger (LB button 6) for Z up
        self.declare_parameter('axis_linear_z_down', 5)        # Small right trigger (RB button 7) for Z down
        self.declare_parameter('axis_linear_z', -1)      # Alternative: axis for Z (set to -1 to disable, use buttons instead)
        
        # D-pad axes for X, Y fine control (d-pad is axes, not buttons)
        self.declare_parameter('axis_dpad_x', 7)         # D-pad left/right (axis 6, negative=left, positive=right)
        self.declare_parameter('axis_dpad_y', 6)         # D-pad up/down (axis 7, negative=down, positive=up)
        
        # Angular orientation axes (rotation) - Right stick and big triggers
        self.declare_parameter('axis_angular_roll', 2)   # Right stick horizontal (axis 2) for roll
        self.declare_parameter('axis_angular_pitch', 3)  # Right stick vertical (axis 3) for pitch
        self.declare_parameter('button_angular_yaw_left', 6)   # Big left trigger (axis 4) for yaw left
        self.declare_parameter('button_angular_yaw_right', 7)  # Big right trigger (axis 5) for yaw right
        
        # Scale factors
        self.declare_parameter('axis_scale_linear', 0.15)  # Scale factor for linear motion (m/s) - increased speed
        self.declare_parameter('axis_scale_angular', 0.3)  # Scale factor for angular motion (rad/s) - increased speed
        self.declare_parameter('button_scale_linear', 0.1) # Scale factor for d-pad linear motion (slower for fine control)
        self.declare_parameter('button_scale_angular', 0.05) # Scale factor for button-based angular motion (rad per button press)
        
        # Rate limiting
        self.declare_parameter('publish_rate', 100.0)      # Max publish rate (Hz)
        
        # Joystick timeout (seconds without messages before considering joystick offline)
        self.declare_parameter('joystick_timeout', 0.5)     # Timeout in seconds (default 0.5s)
        
        # Button mappings
        self.declare_parameter('button_reset', 11)          # Button for reset/home (button 11)
        self.declare_parameter('button_enable', -1)        # Button to enable control (-1 to disable, always enabled)
        self.declare_parameter('button_emergency_stop', 10)  # Button for emergency stop (button 10)
        
        # Gripper control - buttons for continuous incremental control
        self.declare_parameter('button_gripper_open', 0)    # Button to open gripper (incremental)
        self.declare_parameter('button_gripper_close', 1)   # Button to close gripper (incremental)
        self.declare_parameter('gripper_step', 0.05)        # Step size for gripper position change per button press
        self.declare_parameter('gripper_min', 0.0)          # Minimum gripper position (closed)
        self.declare_parameter('gripper_max', 1.0)          # Maximum gripper position (open)
        
        # Topic parameters
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_topic', '/cartesian/ArmCartesian/velocity_reference')
        self.declare_parameter('gripper_topic', '/xbotcore/gripper/dagana_1/command')
        self.declare_parameter('pose_euler_topic', '/teleop_cmd/pose_euler')  # Topic for velocity with Euler angles
        
        # Get parameter values
        self.axis_linear_x = int(self.get_parameter('axis_linear_x').value)
        self.axis_linear_y = int(self.get_parameter('axis_linear_y').value)
        self.axis_linear_z = int(self.get_parameter('axis_linear_z').value)
        self.axis_linear_z_up = int(self.get_parameter('axis_linear_z_up').value)
        self.axis_linear_z_down = int(self.get_parameter('axis_linear_z_down').value)
        self.axis_dpad_x = int(self.get_parameter('axis_dpad_x').value)
        self.axis_dpad_y = int(self.get_parameter('axis_dpad_y').value)
        self.axis_angular_roll = int(self.get_parameter('axis_angular_roll').value)
        self.axis_angular_pitch = int(self.get_parameter('axis_angular_pitch').value)
        self.button_angular_yaw_left = int(self.get_parameter('button_angular_yaw_left').value)
        self.button_angular_yaw_right = int(self.get_parameter('button_angular_yaw_right').value)
        self.scale_linear = self.get_parameter('axis_scale_linear').value
        self.scale_angular = self.get_parameter('axis_scale_angular').value
        self.button_scale_linear = self.get_parameter('button_scale_linear').value
        self.button_scale_angular = self.get_parameter('button_scale_angular').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.joystick_timeout = self.get_parameter('joystick_timeout').value
        self.button_reset = int(self.get_parameter('button_reset').value)
        self.button_enable = int(self.get_parameter('button_enable').value)
        self.button_emergency_stop = int(self.get_parameter('button_emergency_stop').value)
        self.button_gripper_open = int(self.get_parameter('button_gripper_open').value)
        self.button_gripper_close = int(self.get_parameter('button_gripper_close').value)
        self.gripper_step = self.get_parameter('gripper_step').value
        self.gripper_min = self.get_parameter('gripper_min').value
        self.gripper_max = self.get_parameter('gripper_max').value
        
        joy_topic = self.get_parameter('joy_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        gripper_topic = self.get_parameter('gripper_topic').value
        pose_euler_topic = self.get_parameter('pose_euler_topic').value
        
        # Create subscriber for joystick input
        self.joy_sub = self.create_subscription(
            Joy,
            joy_topic,
            self.joy_callback,
            10
        )
        
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            cmd_topic,
            10
        )
        
        self.gripper_pub = self.create_publisher(
            JointState,
            gripper_topic,
            10
        )
        
        from std_msgs.msg import Float64MultiArray
        self.pose_euler_pub = self.create_publisher(
            Float64MultiArray,
            pose_euler_topic,
            10
        )
        
        self.postural_pub = self.create_publisher(
            JointState,
            '/cartesian/ArmPostural/reference',
            10
        )
        
        # Homing configuration (joint positions for reset)
        self.homing_ee_down = {
            'joint_names': ['j_arm1_1', 'j_arm1_2', 'j_arm1_3', 'j_arm1_4', 'j_arm1_5', 'j_arm1_6', 'dagana_1_claw_joint'],
            'joint_positions': [0.0, 0.29, -1.85, 0.0, 0.97, 0.0, 0.0]
        }
        
        # Rate limiter
        from rclpy.qos import QoSProfile
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # State tracking
        self.enabled = False
        self.emergency_stopped = False
        self.last_reset_state = False
        self.last_emergency_state = False
        self.last_gripper_open_state = False
        self.last_gripper_close_state = False
        self.current_gripper_position = 0.0
        self.startup_timer_count = 0
        self.pose_publishing_paused = False  # Flag to pause pose publishing after reset
        self.pose_pause_timer = None  # Timer to re-enable pose publishing after reset
        self.pose_pause_timer_count = 0  # Counter for one-shot timer
        self.last_joy_msg_time = None  # Track last joystick message time
        self.joystick_online = False  # Flag to track if joystick is online
        
        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_linear_z = 0.0
        self.cmd_angular_roll = 0.0
        self.cmd_angular_pitch = 0.0
        self.cmd_angular_yaw = 0.0
        
        # Note: No position tracking needed for velocity control
        # Velocities are published directly from joystick inputs
        
        # Flag to publish
        self.should_publish = False
        
        self.get_logger().info('Joystick teleop node started')
        self.get_logger().info('=== Control Mapping ===')
        self.get_logger().info(f'Left Stick: X (axis {self.axis_linear_x}), Y (axis {self.axis_linear_y})')
        self.get_logger().info(f'D-pad: X left/right (axis {self.axis_dpad_x}), Y up/down (axis {self.axis_dpad_y})')
        self.get_logger().info(f'Z: Up (button {self.axis_linear_z_up}), Down (button {self.axis_linear_z_down})')
        self.get_logger().info(f'Right Stick: Roll (axis {self.axis_angular_roll}), Pitch (axis {self.axis_angular_pitch})')
        self.get_logger().info(f'Big Triggers: Yaw left (axis {self.button_angular_yaw_left}, -180° to 0°), Yaw right (axis {self.button_angular_yaw_right}, 0° to 180°)')
        self.get_logger().info(f'Gripper: Open (button {self.button_gripper_open}), Close (button {self.button_gripper_close}) - incremental')
        self.get_logger().info(f'Reset: Button {self.button_reset}')
        self.get_logger().info(f'Emergency Stop: Button {self.button_emergency_stop}')
        if self.button_enable >= 0:
            self.get_logger().info(f'Enable: Button {self.button_enable} (must hold)')
        else:
            self.get_logger().info('Enable: Always enabled (no button required)')
        
        # Create service client for setting control mode to velocity
        if SetControlMode is not None:
            self.control_mode_client = self.create_client(
                SetControlMode, 
                '/cartesian/ArmCartesian/set_control_mode'
            )
            # Wait for service (with timeout)
            if self.control_mode_client.wait_for_service(timeout_sec=5.0):
                self.set_control_mode_velocity()
            else:
                self.get_logger().warn('Control mode service not available, skipping...')
        else:
            self.control_mode_client = None
            self.get_logger().warn('SetControlMode service type not available, skipping control mode setup...')
        
        # Send reset joint command on startup (with a small delay to ensure publisher is ready)
        # Use a one-shot timer to send reset command after a short delay
        self.startup_timer = self.create_timer(0.5, self.startup_reset_callback)
    
    def joy_callback(self, msg):
        """
        Callback function called whenever a joystick message is received.
        
        This is the standard ROS2 pattern: subscribe to /joy topic and
        map joystick inputs to robot commands.
        """
        # Update last joystick message time
        self.last_joy_msg_time = self.get_clock().now()
        self.joystick_online = True
        
        # Check emergency stop button (highest priority)
        if len(msg.buttons) > self.button_emergency_stop:
            if msg.buttons[self.button_emergency_stop] == 1 and not self.last_emergency_state:
                # Toggle emergency stop
                self.emergency_stopped = not self.emergency_stopped
                if self.emergency_stopped:
                    self.get_logger().warn('EMERGENCY STOP ACTIVATED!')
                else:
                    self.get_logger().info('Emergency stop released')
            self.last_emergency_state = msg.buttons[self.button_emergency_stop] == 1
        else:
            self.last_emergency_state = False
        
        # If emergency stopped, zero all commands and return
        if self.emergency_stopped:
            self.cmd_linear_x = 0.0
            self.cmd_linear_y = 0.0
            self.cmd_linear_z = 0.0
            self.cmd_angular_roll = 0.0
            self.cmd_angular_pitch = 0.0
            self.cmd_angular_yaw = 0.0
            return
        
        # Check enable button (deadman switch) - optional
        if self.button_enable >= 0:
            if len(msg.buttons) > self.button_enable:
                self.enabled = (msg.buttons[self.button_enable] == 1)
            else:
                self.enabled = False
        else:
            # Always enabled if button_enable is -1
            self.enabled = True
        
        if not self.enabled:
            # Keep commands when disabled (don't zero out)
            return
        
        # Check reset button
        if len(msg.buttons) > self.button_reset:
            if msg.buttons[self.button_reset] == 1 and not self.last_reset_state:
                # Reset to home position (both pose and joints)
                # self.reset_joint_positions_to_home()
                pass
            self.last_reset_state = msg.buttons[self.button_reset] == 1
        else:
            self.last_reset_state = False
        
        # Handle gripper control (buttons for continuous incremental control)
        # Gripper changes continuously while buttons are held
        if len(msg.buttons) > max(self.button_gripper_open, self.button_gripper_close):
            gripper_changed = False
            
            # Open button - increment gripper position continuously while held
            if msg.buttons[self.button_gripper_open] == 1:
                self.current_gripper_position = min(self.gripper_max, self.current_gripper_position + self.gripper_step)
                gripper_changed = True
            
            # Close button - decrement gripper position continuously while held
            if msg.buttons[self.button_gripper_close] == 1:
                self.current_gripper_position = max(self.gripper_min, self.current_gripper_position - self.gripper_step)
                gripper_changed = True
            
            # Only publish if gripper position changed
            # if gripper_changed:
            #     self.publish_gripper_command()
            #     self.get_logger().debug(f'Gripper position: {self.current_gripper_position:.3f}')

            # pulbish anyway to satisfiy lerobot record teleoperate processor action features align with robot action features
            self.publish_gripper_command()

        
        # Ensure we have enough axes and buttons
        max_axis = max(
            self.axis_linear_x, self.axis_linear_y,
            self.axis_dpad_x, self.axis_dpad_y,
            self.axis_angular_roll, self.axis_angular_pitch,
            self.axis_linear_z_up, self.axis_linear_z_down
        )
        if self.axis_linear_z >= 0:
            max_axis = max(max_axis, self.axis_linear_z)
        
        max_button = max(
            self.button_angular_yaw_left, self.button_angular_yaw_right
        )
        
        if len(msg.axes) <= max_axis:
            return
        if len(msg.buttons) <= max_button:
            return
        
        # Read joystick values (typically range from -1.0 to 1.0)
        # Apply deadzone to prevent drift
        deadzone = 0.1
        
        def apply_deadzone(value, deadzone):
            if abs(value) < deadzone:
                return 0.0
            return value
        
        # Default values: most axes = 0.0, big triggers = 1.0
        # Only set commands when inputs are actively different from default
        # When inputs return to default, set commands to 0.0
        
        # === LINEAR POSITION CONTROL ===
        
        # Left stick for X, Y (continuous control)
        # Default value: 0.0
        # X: left (negative) = smaller, right (positive) = bigger
        stick_x = apply_deadzone(msg.axes[self.axis_linear_x], deadzone)
        self.cmd_linear_x =  stick_x * self.scale_linear if stick_x != 0.0 else 0.0
        
        # Y: up (negative) = smaller, down (positive) = bigger
        stick_y = apply_deadzone(msg.axes[self.axis_linear_y], deadzone)
        self.cmd_linear_y = stick_y * self.scale_linear if stick_y != 0.0 else 0.0
        
        # D-pad axes for X, Y fine control (axes 6 and 7)
        # Default value: 0.0
        # D-pad X (axis 6): negative = left (smaller), positive = right (bigger)
        dpad_x = apply_deadzone(msg.axes[self.axis_dpad_x], deadzone)
        if dpad_x != 0.0:
            self.cmd_linear_x += dpad_x * self.button_scale_linear
        
        # D-pad Y (axis 7): negative = down (bigger), positive = up (smaller)
        dpad_y = apply_deadzone(msg.axes[self.axis_dpad_y], deadzone)
        if dpad_y != 0.0:
            # Up (positive) should be smaller, down (negative) should be bigger
            self.cmd_linear_y += dpad_y * self.button_scale_linear  # Invert so positive = up = smaller
        
        # Z direction control using triggers (axes)
        # Default value: 1.0 (not pressed)
        # Use alternative axis if configured, otherwise use triggers
        if self.axis_linear_z >= 0 and len(msg.axes) > self.axis_linear_z:
            # Use alternative axis if configured
            z_axis = apply_deadzone(msg.axes[self.axis_linear_z], deadzone)
            self.cmd_linear_z = z_axis * self.scale_linear if z_axis != 0.0 else 0.0
        else:
            # Use triggers for Z control
            z_up_velocity = 0.0
            z_down_velocity = 0.0
            
            # Check if axes are available
            if len(msg.axes) > max(self.axis_linear_z_up, self.axis_linear_z_down):
                # Big left trigger (axis 4) for Z up
                # Value ranges from 1.0 (not pressed/default) to -1.0 (fully pressed)
                trigger_value = msg.axes[self.axis_linear_z_up]
                # Only process if significantly different from default (1.0)
                if abs(trigger_value - 1.0) > deadzone:
                    # Convert trigger value to velocity
                    # Normalize: (1.0 - trigger_value) / 2.0 gives us 0.0 (at default) to 1.0 (fully pressed)
                    normalized = (1.0 - trigger_value) / 2.0  # 1.0 -> 0.0, -1.0 -> 1.0
                    if normalized > deadzone:
                        # Map normalized value to linear velocity (positive for up)
                        z_up_velocity = normalized * self.scale_linear

                # Big right trigger (axis 5) for Z down
                trigger_value = msg.axes[self.axis_linear_z_down]
                # Only process if significantly different from default (1.0)
                if abs(trigger_value - 1.0) > deadzone:
                    # Convert trigger value to velocity
                    # Normalize: (1.0 - trigger_value) / 2.0 gives us 0.0 (at default) to 1.0 (fully pressed)
                    normalized = (1.0 - trigger_value) / 2.0  # 1.0 -> 0.0, -1.0 -> 1.0
                    if normalized > deadzone:
                        # Map normalized value to linear velocity (negative for down)
                        z_down_velocity = -normalized * self.scale_linear
                
                # Combine z velocities from both triggers
                # Up trigger = positive, down trigger = negative
                self.cmd_linear_z = z_up_velocity + z_down_velocity  # Will be 0.0 if both triggers at default
            else:
                self.cmd_linear_z = 0.0

        # === ANGULAR ORIENTATION CONTROL ===
        
        # Right stick for roll and pitch
        # Default value: 0.0
        # Roll: left (negative) = smaller, right (positive) = bigger
        roll_axis = apply_deadzone(msg.axes[self.axis_angular_roll], deadzone)
        self.cmd_angular_roll = -roll_axis * self.scale_angular if roll_axis != 0.0 else 0.0
        
        # Pitch: up (negative) = smaller, down (positive) = bigger
        pitch_axis = apply_deadzone(msg.axes[self.axis_angular_pitch], deadzone)
        self.cmd_angular_pitch = pitch_axis * self.scale_angular if pitch_axis != 0.0 else 0.0
        
        # Yaw control using buttons (LB/RB)
        # Default value: 0.0 (buttons not pressed)
        if len(msg.buttons) > max(self.button_angular_yaw_left, self.button_angular_yaw_right):
            if msg.buttons[self.button_angular_yaw_left] == 1:
                # Left button = counter-clockwise (negative yaw)
                self.cmd_angular_yaw = self.scale_angular
            elif msg.buttons[self.button_angular_yaw_right] == 1:
                # Right button = clockwise (positive yaw)
                self.cmd_angular_yaw = -self.scale_angular
            else:
                self.cmd_angular_yaw = 0.0
        else:
            self.cmd_angular_yaw = 0.0


        # Mark that we should publish
        self.should_publish = True
    
    def timer_callback(self):
        """Timer callback to publish velocity commands at fixed rate."""
        # Check if joystick is online
        if self.last_joy_msg_time is not None:
            time_since_last_msg = (self.get_clock().now() - self.last_joy_msg_time).nanoseconds / 1e9
            if time_since_last_msg > self.joystick_timeout:
                # Joystick is offline - zero commands and don't publish
                if self.joystick_online:
                    self.get_logger().warn(f'Joystick offline, stop sending (no messages for {time_since_last_msg:.2f}s)')
                    self.joystick_online = False
                    # Zero out all commands
                    self.cmd_linear_x = 0.0
                    self.cmd_linear_y = 0.0
                    self.cmd_linear_z = 0.0
                    self.cmd_angular_roll = 0.0
                    self.cmd_angular_pitch = 0.0
                    self.cmd_angular_yaw = 0.0
                return
        else:
            # No joystick messages received yet
            return
        
        # Don't publish velocity commands if publishing is paused (after reset)
        if self.pose_publishing_paused:
            return
        
        # Check if any command is non-zero
        has_command = (abs(self.cmd_linear_x) > 1e-6 or 
                      abs(self.cmd_linear_y) > 1e-6 or 
                      abs(self.cmd_linear_z) > 1e-6 or
                      abs(self.cmd_angular_roll) > 1e-6 or 
                      abs(self.cmd_angular_pitch) > 1e-6 or 
                      abs(self.cmd_angular_yaw) > 1e-6)
        
        # Publish velocity commands directly (no integration needed)
        # For velocity control, we publish velocities even when zero to maintain control
        if self.joystick_online:
            self.publish_velocity_command(
                linear_x=self.cmd_linear_x,
                linear_y=self.cmd_linear_y,
                linear_z=self.cmd_linear_z,
                angular_x=self.cmd_angular_roll,
                angular_y=self.cmd_angular_pitch,
                angular_z=self.cmd_angular_yaw
            )
        
        # Publish velocity with Euler angles for monitoring
        self.publish_velocity_euler()
    
    def publish_velocity_command(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        """Publish a velocity command (TwistStamped)."""
        msg = TwistStamped()
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear_x)
        msg.twist.linear.y = float(linear_y)
        msg.twist.linear.z = float(linear_z)
        msg.twist.angular.x = float(angular_x)
        msg.twist.angular.y = float(angular_y)
        msg.twist.angular.z = float(angular_z)
        
        self.velocity_pub.publish(msg)
        self.get_logger().debug(
            f'Published velocity: linear=({linear_x:.3f}, {linear_y:.3f}, {linear_z:.3f}), '
            f'angular=({angular_x:.3f}, {angular_y:.3f}, {angular_z:.3f})'
        )
    
    def publish_gripper_command(self):
        """Publish a gripper command."""
        msg = JointState()
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = ['dagana_1_claw_joint']
        # Reverse: 0.0 = open, 1.0 = close (opposite of axis input)
        gripper_cmd = 1.0 - self.current_gripper_position
        msg.position = [float(gripper_cmd)]
        msg.velocity = []
        msg.effort = []
        
        self.gripper_pub.publish(msg)
        self.get_logger().debug(f'Gripper command published successfully')
    
    def publish_velocity_euler(self):
        """Publish velocity with Euler angles for monitoring."""
        from std_msgs.msg import Float64MultiArray
        msg = Float64MultiArray()
        # Format: [linear_x, linear_y, linear_z, angular_roll, angular_pitch, angular_yaw]
        msg.data = [
            float(self.cmd_linear_x),
            float(self.cmd_linear_y),
            float(self.cmd_linear_z),
            float(self.cmd_angular_roll),
            float(self.cmd_angular_pitch),
            float(self.cmd_angular_yaw)
        ]
        
        self.pose_euler_pub.publish(msg)
        self.get_logger().debug(f'Velocity Euler published successfully')
    
    def set_control_mode_velocity(self):
        """Set control mode to velocity via service call."""
        if self.control_mode_client is None or SetControlMode is None:
            return
        
        request = SetControlMode.Request()
        request.ctrl_mode = 'velocity'
        
        future = self.control_mode_client.call_async(request)
        
        # Wait for service response
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            try:
                response = future.result()
                self.get_logger().info(f'Control mode set to velocity: success={response.success if hasattr(response, "success") else "unknown"}')
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        else:
            self.get_logger().warn('Service call timed out')


    def startup_reset_callback(self):
        if self.startup_timer_count == 0:
            # self.reset_joint_positions_to_home()
            self.startup_timer_count += 1
        else:
            # Cancel the timer after first execution
            self.startup_timer.cancel()
    
    def reset_joint_positions_to_home(self):
        self.pose_publishing_paused = True
        
        # Cancel any existing pause timer
        if self.pose_pause_timer is not None:
            self.pose_pause_timer.cancel()
            self.pose_pause_timer = None
        
        # Reset counter for one-shot timer
        self.pose_pause_timer_count = 0
        
        # Publish homing joint positions to postural reference
        # Publish multiple times to ensure it's received
        for i in range(3):
            msg = JointState()
            msg.header.stamp = Clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.name = self.homing_ee_down['joint_names']
            msg.position = self.homing_ee_down['joint_positions']
            msg.velocity = []
            msg.effort = []
            
            self.postural_pub.publish(msg)
            if i == 0:  # Only log once
                self.get_logger().info(
                    f'Resetting to home position'
                )
        
        # Wait 10 seconds after publishing for homing to complete
        # Use a one-shot timer pattern
        def re_enable_velocity_publishing():
            self.pose_pause_timer_count += 1
            if self.pose_pause_timer_count >= 1:  # Execute once (10 seconds = 1 call at 10.0s period)
                self.pose_publishing_paused = False
                self.get_logger().info('Velocity publishing re-enabled after reset (10 seconds elapsed)')
                if self.pose_pause_timer is not None:
                    self.pose_pause_timer.cancel()
                    self.pose_pause_timer = None
        
        self.pose_pause_timer = self.create_timer(10.0, re_enable_velocity_publishing)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = JoystickTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

