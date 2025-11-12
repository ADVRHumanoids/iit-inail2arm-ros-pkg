# ROS2 Teleop Guide - Official Patterns

This guide explains the official ROS2 patterns for keyboard and joystick teleoperation.

## Overview

In ROS2, there are standard patterns for controlling robots via keyboard and joystick:

1. **Keyboard Teleop**: Event-driven input handling using threading
2. **Joystick Teleop**: Subscribing to `/joy` topic from `joy_node`
3. **Official Packages**: `teleop_twist_keyboard` and `teleop_twist_joy`

## Key Principles

### 1. Event-Driven Publishing (Not Timer-Based)

**❌ Bad Pattern** (your current scripts):
```python
# Publishing continuously on a timer
self.timer = self.create_timer(0.01, self.timer_callback)
def timer_callback(self):
    self.publisher_.publish(msg)  # Always publishing
```

**✅ Good Pattern** (official):
```python
# Publish only when input changes
def process_key(self, key):
    if key == 'w':
        self.publish_command()  # Only when key pressed
```

### 2. Threading for Keyboard Input

Keyboard input should be handled in a separate thread so ROS2 can spin in the main thread:

```python
# Start keyboard thread
keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
keyboard_thread.start()

# Spin ROS2 in main thread
rclpy.spin(node)
```

### 3. Subscribe to `/joy` Topic for Joystick

The standard ROS2 pattern is to subscribe to joystick messages:

```python
self.joy_sub = self.create_subscription(
    Joy,
    '/joy',
    self.joy_callback,
    10
)
```

## Official ROS2 Packages

### Keyboard Control: `teleop_twist_keyboard`

Install:
```bash
sudo apt install ros-<distro>-teleop-twist-keyboard
```

Run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

This publishes `geometry_msgs/msg/Twist` to `/cmd_vel` topic.

### Joystick Control: `teleop_twist_joy`

Install:
```bash
sudo apt install ros-<distro>-joy ros-<distro>-teleop-twist-joy
```

Run:
```bash
# Terminal 1: Start joy node
ros2 run joy joy

# Terminal 2: Start teleop
ros2 launch teleop_twist_joy teleop-launch.py
```

## Custom Implementation Examples

This package includes two example scripts:

### 1. `teleop_keyboard_example.py`

**Features:**
- Proper threading for keyboard input
- Event-driven command publishing
- Terminal settings management
- Clean shutdown handling

**Usage:**
```bash
python3 teleop_keyboard_example.py
```

**Key Concepts:**
- Uses `select` and `termios` for non-blocking input
- Keyboard loop runs in separate thread
- ROS2 spins in main thread
- Commands published only on key press

### 2. `teleop_joystick_example.py`

**Features:**
- Subscribes to `/joy` topic
- Parameterized axis/button mappings
- Enable/disable button support
- Reset/home position button

**Usage:**
```bash
# Terminal 1: Start joy node
ros2 run joy joy

# Terminal 2: Run teleop
python3 teleop_joystick_example.py

# With custom parameters:
python3 teleop_joystick_example.py \
    --ros-args \
    -p axis_linear_x:=1 \
    -p axis_scale_linear:=0.2
```

**Key Concepts:**
- Subscribe to `/joy` topic (standard ROS2 pattern)
- Map axes/buttons via parameters
- Process joystick state in callback
- Publish commands based on joystick state

## Adapting Your Scripts

### Current Scripts (`send_cartesio_command`, etc.)

Your current scripts use timers to publish continuously. To convert to proper teleop:

**Before:**
```python
self.timer = self.create_timer(0.01, self.timer_callback)
def timer_callback(self):
    msg = PoseStamped()
    # ... set values ...
    self.publisher_.publish(msg)
```

**After (Keyboard):**
```python
def keyboard_loop(self):
    while self.running:
        key = self.get_key()
        if key:
            self.process_key(key)  # Event-driven

def process_key(self, key):
    if key == 'w':
        msg = PoseStamped()
        # ... set values based on key ...
        self.publisher_.publish(msg)
```

**After (Joystick):**
```python
def joy_callback(self, msg):
    # Extract joystick values
    linear_x = msg.axes[1] * scale
    
    # Publish command based on joystick
    msg = PoseStamped()
    # ... set values based on joystick ...
    self.publisher_.publish(msg)
```

## Best Practices

1. **Use Parameters**: Make axis mappings, scales, and topics configurable
2. **Enable/Disable**: Add a button/key to enable/disable control for safety
3. **Deadman Switch**: Consider requiring a button to be held for commands
4. **Rate Limiting**: Limit command publishing rate (e.g., 10-50 Hz)
5. **Error Handling**: Handle missing joystick, terminal issues gracefully
6. **Logging**: Use appropriate log levels (info for state changes, debug for commands)

## Integration with Your Robot

To adapt these examples for your specific robot:

1. **Replace Message Types**: Change `PoseStamped` to your command message type
2. **Update Topics**: Change topic names to match your robot's topics
3. **Customize Key Mappings**: Modify `process_key()` for your control scheme
4. **Add Safety Features**: Implement velocity limits, bounds checking, etc.

## Resources

- [ROS2 teleop_twist_keyboard docs](https://docs.ros.org/en/humble/p/teleop_twist_keyboard/index.html)
- [ROS2 teleop_twist_joy docs](https://docs.ros.org/en/iron/p/teleop_twist_joy/index.html)
- [ROS2 joy package](https://index.ros.org/p/joy/)

