import os
from ament_index_python.packages import (
    get_package_share_directory,
    get_package_prefix,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import launch_ros
import xacro


def generate_launch_description():

    #add path for robot meshes
    MDL_ENV_VAR = "IGN_GAZEBO_RESOURCE_PATH"
    urdf_pkg_share = os.path.join(get_package_prefix("inail2arm_urdf"), "share")

    if MDL_ENV_VAR in os.environ:
        os.environ[MDL_ENV_VAR] += ":" + urdf_pkg_share
    else:
        os.environ[MDL_ENV_VAR] = urdf_pkg_share

    # External args
    x_arg = DeclareLaunchArgument("x", default_value="0")    
    y_arg = DeclareLaunchArgument("y", default_value="0")
    z_arg = DeclareLaunchArgument("z", default_value="0")
    R_arg = DeclareLaunchArgument("roll", default_value="0")
    P_arg = DeclareLaunchArgument("pitch", default_value="0")
    Y_arg = DeclareLaunchArgument("yaw", default_value="0")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    end_effector_arg = DeclareLaunchArgument("end_effector", default_value="dagana")
    nicla_arg = DeclareLaunchArgument("nicla", default_value="false")
    nicla_camera_width_arg = DeclareLaunchArgument("nicla_camera_width", default_value="320")
    nicla_camera_height_arg = DeclareLaunchArgument("nicla_camera_height", default_value="240")
    nicla_camera_hz_arg = DeclareLaunchArgument("nicla_camera_hz", default_value="30")
    nicla_camera_P_fx_arg = DeclareLaunchArgument("nicla_camera_P_fx", default_value="421.373566")
    nicla_camera_P_fy_arg = DeclareLaunchArgument("nicla_camera_P_fy", default_value="426.438812")
    nicla_camera_P_cx_arg = DeclareLaunchArgument("nicla_camera_P_cx", default_value="168.731782")
    nicla_camera_P_cy_arg = DeclareLaunchArgument("nicla_camera_P_cy", default_value="102.665989")
    nicla_camera_K_fx_arg = DeclareLaunchArgument("nicla_camera_K_fx", default_value="416.65053")
    nicla_camera_K_fy_arg = DeclareLaunchArgument("nicla_camera_K_fy", default_value="419.40464")
    nicla_camera_K_cx_arg = DeclareLaunchArgument("nicla_camera_K_cx", default_value="166.12451")
    nicla_camera_K_cy_arg = DeclareLaunchArgument("nicla_camera_K_cy", default_value="104.41054")
    proximity_sensors_arg = DeclareLaunchArgument("proximity_sensors", default_value="false")
    pub_world_tf_arg = DeclareLaunchArgument("pub_world_tf", default_value="false")

    # Setup project paths
    pkg_project = get_package_share_directory('inail2arm_gazebo')
    pkg_inail2arm_urdf = get_package_share_directory('inail2arm_urdf')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Opaque function to use the launch argument inside here
    def create_robot_description(context):
        robot_xacro = os.path.join(pkg_inail2arm_urdf, "urdf/", "inail2arm_robot.urdf.xacro")
        assert os.path.exists(robot_xacro), "The inail2arm_robot.urdf.xacro doesnt exist in " + str(robot_xacro)
        robot_description_config = xacro.process_file(
            robot_xacro,
            mappings={
                "end_effector": context.launch_configurations["end_effector"],
                "nicla": context.launch_configurations["nicla"],
                "pub_world_tf": context.launch_configurations["pub_world_tf"],
            },
        )
        robot_desc = robot_description_config.toxml()

        return [SetLaunchConfiguration("robot_desc", robot_desc)]

    create_robot_description_arg = OpaqueFunction(
        function=create_robot_description
    )

    pub_robot_description = Node (
        package="inail2arm_gazebo",
        executable="pub_robot_description",
        name="pub_robot_description",
        output="screen",
        parameters=[
            {"urdf_string": launch_ros.parameter_descriptions.ParameterValue(LaunchConfiguration("robot_desc"), value_type=str)},
            {"topic_name": "robot_description"},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_inail2arm_urdf, "rviz", "inail2arm_rviz.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[
            {"use_sim_time": True},
        ],
    )


    # Simulator launch file
    world_file = pkg_project + '/worlds/inail2arm.world'
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            'gz_args': " -r " + world_file
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='inail2arm_urdf_spawner',
        arguments=[
            "-name", "inail2arm",
            "-entity", "model",
            "-x", LaunchConfiguration('x'),
            "-y", LaunchConfiguration('y'),
            "-z", LaunchConfiguration('z'),
            "-R", LaunchConfiguration('roll'),
            "-P", LaunchConfiguration('pitch'),
            "-Y", LaunchConfiguration('yaw'),
            "-topic", "/robot_description",
        ],
        output="screen",
    )

    return LaunchDescription([
        rviz_arg,
        x_arg,
        y_arg,
        z_arg,
        R_arg,
        P_arg,
        Y_arg,
        end_effector_arg,
        nicla_arg,
        nicla_camera_width_arg,
        nicla_camera_height_arg,
        nicla_camera_hz_arg,
        nicla_camera_P_fx_arg,
        nicla_camera_P_fy_arg,
        nicla_camera_P_cx_arg,
        nicla_camera_P_cy_arg,
        nicla_camera_K_fx_arg,
        nicla_camera_K_fy_arg,
        nicla_camera_K_cx_arg,
        nicla_camera_K_cy_arg,
        proximity_sensors_arg,
        pub_world_tf_arg,
        create_robot_description_arg,
        gz_sim,
        pub_robot_description,
        spawn_robot,
        rviz,
    ])