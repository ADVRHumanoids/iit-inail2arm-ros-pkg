#!/usr/bin/env python3
"""
ROS2 Keyboard Teleop Example - Official Pattern

This demonstrates the proper ROS2 pattern for keyboard-controlled robot teleoperation.
Uses threading to handle keyboard input while ROS2 spins in the background.

Key principles:
1. Use threading to separate input handling from ROS2 spinning
2. Publish commands only when keys are pressed (event-driven, not timer-based)
3. Handle terminal settings properly for non-blocking input
4. Clean shutdown on Ctrl+C
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import sys
import select
import termios
import tty
import threading
from geometry_msgs.msg import PoseStamped

# For other message types, import as needed:
# from xbot_msgs.msg import JointCommand
# from sensor_msgs.msg import JointState


class KeyboardTeleopNode(Node):
    """
    ROS2 Node that publishes commands based on keyboard input.
    
    This follows the official ROS2 pattern where:
    - Keyboard input is handled in a separate thread
    - ROS2 spinning happens in the main thread
    - Commands are published event-driven (on key press) rather than on a timer
    """
    
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        
        # Create publisher(s) for your command topics
        self.pose_pub = self.create_publisher(
            PoseStamped, 
            '/cartesian/ArmCartesian/reference', 
            10
        )
        
        # Example: Uncomment to add joint command publisher
        # from xbot_msgs.msg import JointCommand
        # self.joint_pub = self.create_publisher(
        #     JointCommand,
        #     '/xbotcore/command',
        #     10
        # )
        
        # Control state
        self.running = True
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Thread-safe flag for shutdown
        self._lock = threading.Lock()
        
        self.get_logger().info('Keyboard teleop node started')
        self.print_instructions()
    
    def print_instructions(self):
        """Print control instructions to the user."""
        msg = """
        ========================================
        Keyboard Teleop Control
        ========================================
        Controls:
          w/s    : Move forward/backward (z-axis)
          a/d    : Move left/right (y-axis)
          q/e    : Move up/down (z-axis)
          i/k    : Rotate pitch up/down
          j/l    : Rotate yaw left/right
          u/o    : Rotate roll left/right
          r      : Reset to home position
          SPACE  : Stop/zero velocity
          CTRL-C : Quit
        ========================================
        """
        print(msg)
    
    def get_key(self):
        """Get a single keypress from stdin (non-blocking)."""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None
    
    def setup_terminal(self):
        """Configure terminal for raw input mode."""
        tty.setraw(sys.stdin.fileno())
    
    def restore_terminal(self):
        """Restore terminal to original settings."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def publish_pose_command(self, x=0.0, y=0.0, z=0.7, 
                            qx=0.0, qy=0.0, qz=0.707, qw=0.707):
        """Publish a pose command."""
        msg = PoseStamped()
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        
        self.pose_pub.publish(msg)
        self.get_logger().info(
            f'Published pose: x={x:.3f}, y={y:.3f}, z={z:.3f}'
        )
    
    def process_key(self, key):
        """
        Process a keypress and publish corresponding command.
        
        This is where you map keyboard keys to robot commands.
        """
        # Current pose state (in a real implementation, you'd track this)
        # For this example, we'll use incremental commands
        step = 0.05  # 5cm steps
        angle_step = 0.1  # radian steps
        
        if key == 'w':
            # Move forward (example - you'd need to track current position)
            self.get_logger().info('Moving forward')
            # self.publish_pose_command(x=current_x + step)
            
        elif key == 's':
            self.get_logger().info('Moving backward')
            # self.publish_pose_command(x=current_x - step)
            
        elif key == 'a':
            self.get_logger().info('Moving left')
            
        elif key == 'd':
            self.get_logger().info('Moving right')
            
        elif key == 'q':
            self.get_logger().info('Moving up')
            
        elif key == 'e':
            self.get_logger().info('Moving down')
            
        elif key == 'r':
            # Reset to home position
            self.get_logger().info('Resetting to home position')
            self.publish_pose_command(
                x=0.2, y=0.1, z=0.7,
                qx=0.0, qy=0.0, qz=0.707, qw=0.707
            )
            
        elif key == ' ':
            # Stop - publish zero velocity or current position hold
            self.get_logger().info('Stop command')
            
        elif key == '\x03':  # CTRL-C
            self.get_logger().info('Shutting down...')
            with self._lock:
                self.running = False
            return False
        
        return True
    
    def keyboard_loop(self):
        """Main keyboard input loop (runs in separate thread)."""
        self.setup_terminal()
        
        try:
            while self.running:
                key = self.get_key()
                if key:
                    with self._lock:
                        if not self.running:
                            break
                        self.process_key(key)
                else:
                    # Small sleep to prevent CPU spinning
                    import time
                    time.sleep(0.01)
        finally:
            self.restore_terminal()
    
    def run(self):
        """Run the node with keyboard input handling."""
        # Start keyboard input thread
        keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        keyboard_thread.start()
        
        try:
            # Spin ROS2 node in main thread
            while rclpy.ok() and self.running:
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            with self._lock:
                self.running = False
            self.restore_terminal()
            self.get_logger().info('Node shutdown complete')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = KeyboardTeleopNode()
        node.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

