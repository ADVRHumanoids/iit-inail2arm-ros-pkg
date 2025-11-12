import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for joystick teleoperation.
    
    Launches both:
    1. joy_node - ROS2 joystick driver node
    2. joystick_teleop_node - Custom teleop node that maps joystick to robot commands
    
    Usage:
        ros2 launch inail2arm_gazebo joystick_teleop.launch.py
        
        # With custom parameters:
        ros2 launch inail2arm_gazebo joystick_teleop.launch.py \
            axis_linear_x:=0 \
            axis_scale_linear:=0.1
    """
    
    # Get package directory
    pkg_inail2arm_gazebo = get_package_share_directory('inail2arm_gazebo')
    
    # Declare launch arguments for joystick teleop node
    # These can be overridden from command line or other launch files
    
    # Joystick device (default: /dev/input/js0)
    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # Joystick deadzone
    joy_deadzone_arg = DeclareLaunchArgument(
        'joy_deadzone',
        default_value='0.1',
        description='Joystick deadzone threshold'
    )
    
    # Linear axes mapping
    axis_linear_x_arg = DeclareLaunchArgument(
        'axis_linear_x',
        default_value='1',
        description='Joystick axis for linear X motion (left stick horizontal)'
    )
    axis_linear_y_arg = DeclareLaunchArgument(
        'axis_linear_y',
        default_value='0',
        description='Joystick axis for linear Y motion (left stick vertical)'
    )
    # Z position control - Triggers (axes)
    axis_linear_z_up_arg = DeclareLaunchArgument(
        'axis_linear_z_up',
        default_value='4',
        description='Axis for Z up (trigger axis 4)'
    )
    axis_linear_z_down_arg = DeclareLaunchArgument(
        'axis_linear_z_down',
        default_value='5',
        description='Axis for Z down (trigger axis 5)'
    )
    axis_linear_z_arg = DeclareLaunchArgument(
        'axis_linear_z',
        default_value='-1',
        description='Alternative axis for linear Z motion (-1 to disable, use triggers instead)'
    )
    
    # D-pad axes for X, Y fine control (d-pad is axes, not buttons)
    axis_dpad_x_arg = DeclareLaunchArgument(
        'axis_dpad_x',
        default_value='7',
        description='D-pad X axis (axis 7, negative=left, positive=right)'
    )
    axis_dpad_y_arg = DeclareLaunchArgument(
        'axis_dpad_y',
        default_value='6',
        description='D-pad Y axis (axis 6, negative=down, positive=up)'
    )
    
    # Angular axes mapping (Euler angles)
    axis_angular_roll_arg = DeclareLaunchArgument(
        'axis_angular_roll',
        default_value='2',
        description='Joystick axis for roll rotation (right stick horizontal)'
    )
    axis_angular_pitch_arg = DeclareLaunchArgument(
        'axis_angular_pitch',
        default_value='3',
        description='Joystick axis for pitch rotation (right stick vertical)'
    )
    button_angular_yaw_left_arg = DeclareLaunchArgument(
        'button_angular_yaw_left',
        default_value='6',
        description='Button for yaw left (button 6)'
    )
    button_angular_yaw_right_arg = DeclareLaunchArgument(
        'button_angular_yaw_right',
        default_value='7',
        description='Button for yaw right (button 7)'
    )
    
    # Scale factors
    axis_scale_linear_arg = DeclareLaunchArgument(
        'axis_scale_linear',
        default_value='0.15',
        description='Scale factor for linear motion (m/s) - increased speed'
    )
    axis_scale_angular_arg = DeclareLaunchArgument(
        'axis_scale_angular',
        default_value='0.3',
        description='Scale factor for angular motion (rad/s) - increased speed'
    )
    button_scale_linear_arg = DeclareLaunchArgument(
        'button_scale_linear',
        default_value='0.1',
        description='Scale factor for d-pad linear motion (slower for fine control)'
    )
    button_scale_angular_arg = DeclareLaunchArgument(
        'button_scale_angular',
        default_value='0.05',
        description='Scale factor for button-based angular motion (rad per button press)'
    )
    
    # Button mappings
    button_reset_arg = DeclareLaunchArgument(
        'button_reset',
        default_value='11',
        description='Button index for reset/home position (button 11)'
    )
    button_enable_arg = DeclareLaunchArgument(
        'button_enable',
        default_value='-1',
        description='Button index for enable/deadman switch (-1 to disable, always enabled)'
    )
    button_emergency_stop_arg = DeclareLaunchArgument(
        'button_emergency_stop',
        default_value='10',
        description='Button index for emergency stop (button 10)'
    )
    
    # Gripper control parameters (buttons for incremental control)
    button_gripper_open_arg = DeclareLaunchArgument(
        'button_gripper_open',
        default_value='0',
        description='Button to open gripper (incremental)'
    )
    button_gripper_close_arg = DeclareLaunchArgument(
        'button_gripper_close',
        default_value='1',
        description='Button to close gripper (incremental)'
    )
    gripper_step_arg = DeclareLaunchArgument(
        'gripper_step',
        default_value='0.05',
        description='Step size for gripper position change per button press'
    )
    gripper_min_arg = DeclareLaunchArgument(
        'gripper_min',
        default_value='0.0',
        description='Minimum gripper position (closed)'
    )
    gripper_max_arg = DeclareLaunchArgument(
        'gripper_max',
        default_value='1.0',
        description='Maximum gripper position (open)'
    )
    
    # Topic parameters
    joy_topic_arg = DeclareLaunchArgument(
        'joy_topic',
        default_value='/joy',
        description='Topic name for joystick messages'
    )
    cmd_topic_arg = DeclareLaunchArgument(
        'cmd_topic',
        default_value='/cartesian/ArmCartesian/velocity_reference',
        description='Topic name for velocity command messages'
    )
    gripper_topic_arg = DeclareLaunchArgument(
        'gripper_topic',
        default_value='/xbotcore/gripper/dagana_1/command',
        description='Topic name for gripper command messages'
    )
    pose_euler_topic_arg = DeclareLaunchArgument(
        'pose_euler_topic',
        default_value='/teleop_cmd/pose_euler',
        description='Topic name for velocity with Euler angles'
    )
    
    # Publish rate
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='Command publish rate (Hz)'
    )
    
    # Create joy node (official ROS2 joystick driver)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('joy_device'),
            'deadzone': LaunchConfiguration('joy_deadzone'),
            'autorepeat_rate': 20.0,
        }]
    )
    
    # Create joystick teleop node
    joystick_teleop_node = Node(
        package='inail2arm_gazebo',
        executable='teleop_joystick_example.py',
        name='joystick_teleop_node',
        output='screen',
        parameters=[{
            # Axis mappings
            'axis_linear_x': LaunchConfiguration('axis_linear_x'),
            'axis_linear_y': LaunchConfiguration('axis_linear_y'),
            'axis_linear_z': LaunchConfiguration('axis_linear_z'),
            'axis_linear_z_up': LaunchConfiguration('axis_linear_z_up'),
            'axis_linear_z_down': LaunchConfiguration('axis_linear_z_down'),
            'axis_dpad_x': LaunchConfiguration('axis_dpad_x'),
            'axis_dpad_y': LaunchConfiguration('axis_dpad_y'),
            'axis_angular_roll': LaunchConfiguration('axis_angular_roll'),
            'axis_angular_pitch': LaunchConfiguration('axis_angular_pitch'),
            'button_angular_yaw_left': LaunchConfiguration('button_angular_yaw_left'),
            'button_angular_yaw_right': LaunchConfiguration('button_angular_yaw_right'),
            
            # Scale factors
            'axis_scale_linear': LaunchConfiguration('axis_scale_linear'),
            'axis_scale_angular': LaunchConfiguration('axis_scale_angular'),
            'button_scale_linear': LaunchConfiguration('button_scale_linear'),
            'button_scale_angular': LaunchConfiguration('button_scale_angular'),
            
            # Button mappings
            'button_reset': LaunchConfiguration('button_reset'),
            'button_enable': LaunchConfiguration('button_enable'),
            'button_emergency_stop': LaunchConfiguration('button_emergency_stop'),
            
            # Gripper control
            'button_gripper_open': LaunchConfiguration('button_gripper_open'),
            'button_gripper_close': LaunchConfiguration('button_gripper_close'),
            'gripper_step': LaunchConfiguration('gripper_step'),
            'gripper_min': LaunchConfiguration('gripper_min'),
            'gripper_max': LaunchConfiguration('gripper_max'),
            
            # Topics
            'joy_topic': LaunchConfiguration('joy_topic'),
            'cmd_topic': LaunchConfiguration('cmd_topic'),
            'gripper_topic': LaunchConfiguration('gripper_topic'),
            'pose_euler_topic': LaunchConfiguration('pose_euler_topic'),
            
            # Rate
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        joy_device_arg,
        joy_deadzone_arg,
        axis_linear_x_arg,
        axis_linear_y_arg,
        axis_linear_z_arg,
        axis_linear_z_up_arg,
        axis_linear_z_down_arg,
        axis_dpad_x_arg,
        axis_dpad_y_arg,
        axis_angular_roll_arg,
        axis_angular_pitch_arg,
        button_angular_yaw_left_arg,
        button_angular_yaw_right_arg,
        axis_scale_linear_arg,
        axis_scale_angular_arg,
        button_scale_linear_arg,
        button_scale_angular_arg,
        button_reset_arg,
        button_enable_arg,
        button_emergency_stop_arg,
        button_gripper_open_arg,
        button_gripper_close_arg,
        gripper_step_arg,
        gripper_min_arg,
        gripper_max_arg,
        joy_topic_arg,
        cmd_topic_arg,
        gripper_topic_arg,
        pose_euler_topic_arg,
        publish_rate_arg,
        
        # Nodes
        joy_node,
        joystick_teleop_node,
    ])

